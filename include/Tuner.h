#pragma once
//
// Non-blocking tuning engine.
//
// The original tune() ran to completion inside a single loop() call. Worst case
// was roughly 560 relay-settle-and-measure cycles - about fifteen seconds of
// continuous key-down - during which nothing else ran: no buttons, no serial,
// no display, and only eight power checks in the whole run. There was no way to
// abort and the overload path could not interrupt a tune in progress.
//
// This version advances one measurement per loop() call, so:
//   * the TUNE button aborts,
//   * power and thermal limits are checked on *every* measurement,
//   * the display shows live progress,
//   * a failed tune restores the state you started with instead of leaving the
//     relays wherever the search happened to stop.
//
// Search: analytic seeds (Solver.h) -> successive approximation MSB-first on C
// then L -> local single-bit refinement. Typically 40-90 measurements.
//

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "MemStore.h"
#include "Relays.h"
#include "Sensor.h"
#include "Settings.h"
#include "Solver.h"
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

    freqHz_ = freqHz;
    force_ = force;
    entryState_ = relays_->current();
    entryState_.bypass = false;

    best_ = entryState_;
    bestSwr_ = 99.0f;
    steps_ = 0;
    invalidRun_ = 0;
    abortRequested_ = false;
    measurePending_ = false;
    resultReady_ = false;
    result_ = TuneResult::None;

    buildCandidates(hint);
    candidateIdx_ = 0;
    saBit_ = kRelayCount - 1;
    saOnC_ = true;
    refinePass_ = 0;
    refineBit_ = 0;
    refineOnC_ = true;
    refineImproved_ = false;

    plannedSteps_ = static_cast<uint16_t>(candidates_.size()) +
                    (kRelayCount * 2) +
                    static_cast<uint16_t>(cfg_->refinePasses) * (kRelayCount * 2) + 2;
    if (plannedSteps_ > cfg_->maxTuneSteps) plannedSteps_ = cfg_->maxTuneSteps;
    if (plannedSteps_ == 0) plannedSteps_ = 1;

    // Give the radio the configured lead time after TX-request is asserted
    // before the first measurement, without blocking the loop for it.
    leadStartMs_ = millis();
    phase_ = Phase::Lead;
    return true;
  }

  void abort() {
    if (phase_ != Phase::Idle) abortRequested_ = true;
  }

  bool running() const { return phase_ != Phase::Idle; }

  uint8_t percent() const {
    if (phase_ == Phase::Idle) return 0;
    uint32_t p = (static_cast<uint32_t>(steps_) * 100U) / plannedSteps_;
    return static_cast<uint8_t>(p > 99 ? 99 : p);
  }

  float bestSwr() const { return bestSwr_; }
  const SensorReading& lastReading() const { return lastReading_; }

  bool takeResult(TuneResult& r, RelayState& state, float& swr) {
    if (!resultReady_) return false;
    resultReady_ = false;
    r = result_;
    state = resultState_;
    swr = resultSwr_;
    return true;
  }

  // External safety stop (overload / over-temperature).
  void emergencyStop(TuneResult why) {
    if (phase_ == Phase::Idle) return;
    finish(why, entryState_, 99.0f);
  }

  void loop() {
    if (phase_ == Phase::Idle) return;

    if (abortRequested_) {
      finish(TuneResult::Aborted, entryState_, bestSwr_);
      return;
    }
    if (steps_ >= cfg_->maxTuneSteps && phase_ != Phase::Confirm) {
      phase_ = Phase::Confirm;
      measurePending_ = false;
    }

    switch (phase_) {
      case Phase::Lead:
        if (elapsed(leadStartMs_) >= cfg_->txRequestLeadMs) phase_ = Phase::Baseline;
        break;
      case Phase::Baseline:   stepBaseline(); break;
      case Phase::Candidates: stepCandidates(); break;
      case Phase::SuccApprox: stepSuccApprox(); break;
      case Phase::Refine:     stepRefine(); break;
      case Phase::Confirm:    stepConfirm(); break;
      default: break;
    }
  }

 private:
  enum class Phase : uint8_t { Idle, Lead, Baseline, Candidates, SuccApprox, Refine, Confirm };

  Settings* cfg_ = nullptr;
  RelayController* relays_ = nullptr;
  BridgeSensor* sensor_ = nullptr;

  Phase phase_ = Phase::Idle;
  uint32_t freqHz_ = 0;
  bool force_ = false;
  bool abortRequested_ = false;

  RelayState entryState_;
  RelayState best_;
  float bestSwr_ = 99.0f;

  std::vector<RelayState> candidates_;
  size_t candidateIdx_ = 0;

  int8_t saBit_ = 0;
  bool saOnC_ = true;

  uint8_t refinePass_ = 0;
  uint8_t refineBit_ = 0;
  bool refineOnC_ = true;
  bool refineImproved_ = false;

  bool measurePending_ = false;
  uint32_t leadStartMs_ = 0;
  uint16_t steps_ = 0;
  uint16_t plannedSteps_ = 1;
  uint8_t invalidRun_ = 0;

  SensorReading lastReading_;

  bool resultReady_ = false;
  TuneResult result_ = TuneResult::None;
  RelayState resultState_;
  float resultSwr_ = 99.0f;

  // -- candidate list -------------------------------------------------------

  void buildCandidates(const RelayState* hint) {
    candidates_.clear();
    candidates_.reserve(20);

    auto push = [this](const RelayState& s) {
      RelayState t = s;
      t.bypass = false;
      t.lMask &= 0x7F;
      t.cMask &= 0x7F;
      for (size_t i = 0; i < candidates_.size(); ++i) {
        if (candidates_[i] == t) return;
      }
      candidates_.push_back(t);
    };

    // Anything we already know about comes first, so a good memory entry ends
    // the search almost immediately.
    if (hint && !hint->bypass) push(*hint);
    push(entryState_);

    const BandInfo* band = findBand(freqHz_);
    if (band) {
      RelayState s;
      s.lMask = band->typicalLMask;
      s.cMask = band->typicalCMask;
      s.topology = band->typicalTopology;
      push(s);
    }

    // Analytic seeds for both topologies.
    RelayState seeds[kSeedResistanceCount];
    for (uint8_t t = 0; t < 2; ++t) {
      size_t n = buildAnalyticSeeds(freqHz_, t == 1, seeds, kSeedResistanceCount);
      for (size_t i = 0; i < n; ++i) push(seeds[i]);
    }

    // Straight-through, as a floor.
    RelayState zero;
    zero.topology = entryState_.topology;
    push(zero);
  }

  // -- measurement ----------------------------------------------------------

  // Applies `st`, waits out the relay settle without blocking, then samples.
  // Returns true exactly once per measurement, with the score in `outScore`.
  bool measure(const RelayState& st, float& outScore) {
    if (!measurePending_) {
      relays_->apply(st);
      measurePending_ = true;
      return false;
    }
    if (!relays_->settled()) return false;

    lastReading_ = sensor_->readDefault();
    measurePending_ = false;
    ++steps_;

    // Safety is checked on every single measurement now, not once per seed.
    if (cfg_->powerProtEnabled && lastReading_.powerW >= cfg_->powerLimitW) {
      finish(TuneResult::PowerOverload, entryState_, 99.0f);
      return false;
    }

    if (!lastReading_.valid) {
      // A carrier that drops out mid-tune would otherwise make every remaining
      // measurement score 99 and send the search wandering.
      if (!force_ && ++invalidRun_ >= 4) {
        finish(TuneResult::NoPower, entryState_, 99.0f);
        return false;
      }
      outScore = 99.0f;
      return true;
    }

    invalidRun_ = 0;
    // Penalise anything above the safe tuning power so the search never parks
    // on a setting that happens to read well while overdriving the network.
    outScore = (lastReading_.powerW > cfg_->maxTunePowerW) ? 99.0f : lastReading_.swr;
    return true;
  }

  // -- phases ---------------------------------------------------------------

  void stepBaseline() {
    float score;
    if (!measure(entryState_, score)) return;

    if (!force_) {
      if (!lastReading_.valid || lastReading_.powerW < cfg_->minTunePowerW) {
        finish(TuneResult::NoPower, entryState_, 99.0f);
        return;
      }
      if (lastReading_.powerW > cfg_->maxTunePowerW) {
        finish(TuneResult::PowerHigh, entryState_, lastReading_.swr);
        return;
      }
    }

    best_ = entryState_;
    bestSwr_ = score;

    if (bestSwr_ <= cfg_->targetSWR) {
      finish(TuneResult::Success, best_, bestSwr_);
      return;
    }
    phase_ = Phase::Candidates;
  }

  void stepCandidates() {
    if (candidateIdx_ >= candidates_.size()) {
      phase_ = Phase::SuccApprox;
      saOnC_ = true;
      saBit_ = kRelayCount - 1;
      return;
    }

    float score;
    if (!measure(candidates_[candidateIdx_], score)) return;

    if (score < bestSwr_) {
      best_ = candidates_[candidateIdx_];
      bestSwr_ = score;
    }
    ++candidateIdx_;

    if (bestSwr_ <= cfg_->targetSWR) {
      phase_ = Phase::Confirm;
      measurePending_ = false;
    }
  }

  // Successive approximation, MSB first: try each bit set, keep it if it helps.
  // Fourteen measurements total instead of the old two-pass 28-plus-42.
  void stepSuccApprox() {
    if (saBit_ < 0) {
      if (saOnC_) {
        saOnC_ = false;
        saBit_ = kRelayCount - 1;
        return;
      }
      phase_ = Phase::Refine;
      refinePass_ = 0;
      refineBit_ = 0;
      refineOnC_ = true;
      refineImproved_ = false;
      return;
    }

    RelayState test = best_;
    if (saOnC_) test.cMask ^= static_cast<uint8_t>(1u << saBit_);
    else test.lMask ^= static_cast<uint8_t>(1u << saBit_);

    float score;
    if (!measure(test, score)) return;

    if (score + 0.01f < bestSwr_) {
      best_ = test;
      bestSwr_ = score;
    }
    --saBit_;

    if (bestSwr_ <= cfg_->targetSWR) {
      phase_ = Phase::Confirm;
      measurePending_ = false;
    }
  }

  // Local hill climb, LSB first, until a whole pass yields no improvement.
  void stepRefine() {
    if (refinePass_ >= cfg_->refinePasses) {
      phase_ = Phase::Confirm;
      measurePending_ = false;
      return;
    }

    if (refineBit_ >= kRelayCount) {
      if (refineOnC_) {
        refineOnC_ = false;
        refineBit_ = 0;
        return;
      }
      if (!refineImproved_) {
        phase_ = Phase::Confirm;
        measurePending_ = false;
        return;
      }
      ++refinePass_;
      refineOnC_ = true;
      refineBit_ = 0;
      refineImproved_ = false;
      return;
    }

    RelayState test = best_;
    if (refineOnC_) test.cMask ^= static_cast<uint8_t>(1u << refineBit_);
    else test.lMask ^= static_cast<uint8_t>(1u << refineBit_);

    float score;
    if (!measure(test, score)) return;

    if (score + 0.005f < bestSwr_) {
      best_ = test;
      bestSwr_ = score;
      refineImproved_ = true;
    }
    ++refineBit_;

    if (bestSwr_ <= cfg_->targetSWR) {
      phase_ = Phase::Confirm;
      measurePending_ = false;
    }
  }

  void stepConfirm() {
    float score;
    if (!measure(best_, score)) return;

    float finalSwr = lastReading_.valid ? lastReading_.swr : bestSwr_;

    // If the search never beat the state we came in with, put that back rather
    // than leaving the relays wherever the last probe landed.
    if (finalSwr > cfg_->goodSWR && bestSwr_ >= 99.0f) {
      finish(TuneResult::Failed, entryState_, finalSwr);
      return;
    }

    if (finalSwr <= cfg_->targetSWR) finish(TuneResult::Success, best_, finalSwr);
    else if (finalSwr <= cfg_->goodSWR) finish(TuneResult::GoodEnough, best_, finalSwr);
    else finish(TuneResult::Failed, best_, finalSwr);
  }

  void finish(TuneResult r, const RelayState& state, float swr) {
    relays_->apply(state);
    phase_ = Phase::Idle;
    measurePending_ = false;
    abortRequested_ = false;
    result_ = r;
    resultState_ = state;
    resultSwr_ = swr;
    resultReady_ = true;
  }
};

}  // namespace atu
