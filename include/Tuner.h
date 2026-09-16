#pragma once
//
// Non-blocking tuning engine.
//
// Hardware glue around the search strategy in core/TuneSearch.h: it applies
// each probe to the relays, waits out the settle time without blocking, asks
// the sensor task for a measurement (more samples for precise probes) and
// feeds the result back. Power and SWR limits are checked on every single
// measurement.
//
// The engine never moves the relays when a tune ends. It reports the state it
// wants and main.cpp applies it through the RF-safe path, so an abort or a
// power surge cannot make it switch relays under full power.
//

#include <Arduino.h>

#include "Config.h"
#include "Network.h"
#include "Relays.h"
#include "Sensor.h"
#include "Settings.h"
#include "TuneSearch.h"
#include "Types.h"

namespace atu {

class TuningEngine {
 public:
  void begin(Settings* settings, RelayController* relays, BridgeSensor* sensor) {
    cfg_ = settings;
    relays_ = relays;
    sensor_ = sensor;
  }

  // hint may be null. Returns false if a tune is already running.
  bool start(uint32_t freqHz, bool force, const RelayState* hint) {
    if (phase_ != Phase::Idle) return false;

    force_ = force;
    entry_ = relays_->current();
    entry_.bypass = false;
    invalidRun_ = 0;
    abortRequested_ = false;
    resultReady_ = false;
    lastReading_ = SensorReading{};
    startMs_ = millis();

    SearchParams p;
    p.freqHz = freqHz;
    p.targetGamma = swrToGamma(cfg_->targetSWR);
    p.goodGamma = swrToGamma(cfg_->goodSWR);
    p.refinePasses = cfg_->refinePasses;
    p.maxSteps = cfg_->maxTuneSteps;
    p.useModel = cfg_->tuneModelFit;
    search_.start(p, entry_, hint);

    // Give the radio the configured lead time after TX-request is asserted
    // before the first measurement, without blocking the loop for it.
    step_ = Step::Next;
    phase_ = Phase::Lead;
    return true;
  }

  void abort() {
    if (phase_ != Phase::Idle) abortRequested_ = true;
  }

  bool running() const { return phase_ != Phase::Idle; }

  uint8_t percent() const {
    if (phase_ == Phase::Idle) return 0;
    uint32_t p = (static_cast<uint32_t>(search_.steps()) * 100U) / search_.plannedSteps();
    return static_cast<uint8_t>(p > 99 ? 99 : p);
  }

  const SensorReading& lastReading() const { return lastReading_; }
  // The state the last tune started from.
  const RelayState& entryState() const { return entry_; }
  const char* phaseName() const { return search_.phaseName(); }

  bool takeResult(TuneResult& r, RelayState& state, float& swr) {
    if (!resultReady_) return false;
    resultReady_ = false;
    r = result_;
    state = resultState_;
    swr = resultSwr_;
    return true;
  }

  // Details of the last completed tune, for the console.
  uint16_t lastSteps() const { return lastSteps_; }
  uint32_t lastDurationMs() const { return lastDurationMs_; }
  uint32_t lastModelFrequency() const { return lastModelFreq_; }

  // External safety stop (overload / over-temperature). Leaves the relays
  // exactly where they are: switching them now would be hot switching.
  void emergencyStop(TuneResult why) {
    if (phase_ == Phase::Idle) return;
    finish(why, relays_->current(), 99.0f);
  }

  void loop() {
    if (phase_ == Phase::Idle) return;

    if (abortRequested_) {
      finish(TuneResult::Aborted, entry_, gammaToSwr(search_.bestGamma()));
      return;
    }

    if (phase_ == Phase::Lead) {
      if (elapsed(startMs_) < cfg_->txRequestLeadMs) return;
      phase_ = Phase::Run;
    }

    // A few non-measuring transitions per call keep the tune moving without
    // letting one loop() iteration run long.
    for (int i = 0; i < 4 && phase_ == Phase::Run; ++i) {
      switch (step_) {
        case Step::Next: {
          SearchStatus st = search_.next(probe_);
          if (st == SearchStatus::Busy) return;
          if (st == SearchStatus::Done) {
            complete();
            return;
          }
          relays_->apply(probe_.state);
          step_ = Step::Settle;
          break;
        }
        case Step::Settle:
          if (!relays_->settled()) return;
          requestMeasurement();
          step_ = Step::Sample;
          return;
        case Step::Sample: {
          SensorReading r;
          if (!sensor_->takeMeasurement(r)) {
            // Anyone else asking the sensor for a measurement cancels ours;
            // ask again rather than waiting forever.
            if (elapsed(requestMs_) > kMeasureTimeoutMs) requestMeasurement();
            return;
          }
          step_ = Step::Next;
          handleReading(r);
          return;
        }
      }
    }
  }

 private:
  enum class Phase : uint8_t { Idle, Lead, Run };
  enum class Step : uint8_t { Next, Settle, Sample };

  // Mid-tune, allow some headroom over pwrmax for SSB/ALC overshoot before
  // stopping; the baseline check is strict.
  static constexpr float kMidTunePowerMargin = 1.25f;
  static constexpr uint32_t kMeasureTimeoutMs = 500;

  Settings* cfg_ = nullptr;
  RelayController* relays_ = nullptr;
  BridgeSensor* sensor_ = nullptr;

  TuneSearch search_;
  Probe probe_;

  Phase phase_ = Phase::Idle;
  Step step_ = Step::Next;
  bool force_ = false;
  bool abortRequested_ = false;
  RelayState entry_;
  uint32_t startMs_ = 0;
  uint32_t requestMs_ = 0;
  uint8_t invalidRun_ = 0;

  void requestMeasurement() {
    sensor_->requestMeasurement(probe_.precise ? cfg_->preciseSamples : cfg_->measureSamples);
    requestMs_ = millis();
  }

  SensorReading lastReading_;

  bool resultReady_ = false;
  TuneResult result_ = TuneResult::None;
  RelayState resultState_;
  float resultSwr_ = 99.0f;

  uint16_t lastSteps_ = 0;
  uint32_t lastDurationMs_ = 0;
  uint32_t lastModelFreq_ = 0;

  void handleReading(const SensorReading& r) {
    lastReading_ = r;
    bool baseline = search_.steps() == 0;

    if (cfg_->powerProtEnabled && r.powerW >= cfg_->powerLimitW) {
      finish(TuneResult::PowerOverload, relays_->current(), 99.0f);
      return;
    }
    float ceiling = cfg_->maxTunePowerW * (baseline ? 1.0f : kMidTunePowerMargin);
    if (r.valid && r.powerW > ceiling) {
      // Stop switching now; main.cpp restores the entry state once RF drops.
      finish(TuneResult::PowerHigh, entry_, r.swr);
      return;
    }

    if (!force_) {
      if (baseline && (!r.valid || r.powerW < cfg_->minTunePowerW)) {
        finish(TuneResult::NoPower, entry_, 99.0f);
        return;
      }
      // A carrier that drops out mid-tune would otherwise make every remaining
      // measurement score as a total mismatch and send the search wandering.
      if (!r.valid) {
        if (++invalidRun_ >= 4) {
          finish(TuneResult::NoPower, entry_, 99.0f);
          return;
        }
      } else {
        invalidRun_ = 0;
      }
    }

    search_.report(r.valid, r.gamma, r.gammaNoise);
  }

  void complete() {
    if (!search_.anyValid()) {
      finish(force_ ? TuneResult::Failed : TuneResult::NoPower, entry_, 99.0f);
      return;
    }
    float g = search_.finalGamma();
    float swr = gammaToSwr(g);
    if (g <= swrToGamma(cfg_->targetSWR)) finish(TuneResult::Success, search_.best(), swr);
    else if (g <= swrToGamma(cfg_->goodSWR)) finish(TuneResult::GoodEnough, search_.best(), swr);
    else finish(TuneResult::Failed, search_.best(), swr);
  }

  void finish(TuneResult r, const RelayState& state, float swr) {
    phase_ = Phase::Idle;
    step_ = Step::Next;
    abortRequested_ = false;
    result_ = r;
    resultState_ = state;
    resultSwr_ = swr;
    resultReady_ = true;
    lastSteps_ = search_.steps();
    lastDurationMs_ = elapsed(startMs_);
    lastModelFreq_ = search_.modelFrequency();
  }
};

}  // namespace atu
