#pragma once
//
// SWR bridge front end.
//
// A dedicated FreeRTOS task on core 0 owns the ADC. It reads one FWD/REV pair
// per millisecond and:
//
//   * trips the TX inhibit line within ~2 ms of an overload, independently of
//     the main loop (which may be busy drawing the display or serving the
//     web UI),
//   * keeps a rolling "latest" reading for the display and protection logic,
//   * serves averaged measurements on request to the tuner, the sweep and the
//     calibration commands,
//   * records a fast trace for the relay settle-time test.
//
// Each FWD/REV pair is converted to |Gamma| on its own and a measurement
// reports the median of those ratios. On SSB or a keyed CW carrier the
// forward level moves between samples; the ratio within a pair barely does.
//
// Power formula (matches README and the calibration wizard):
//   Vf = max(0, (fwdMv - fwdOffsetMv)) * fwdScale
//   P  = (Vf / 1000)^2 / powerScale
//

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <atomic>
#include <cmath>

#include "Config.h"
#include "Controls.h"
#include "Network.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

struct TracePoint {
  uint32_t us;      // micros() at the sample
  float fwdMv;
  float revMv;
};

class BridgeSensor {
 public:
  static constexpr uint8_t kMaxSamples = 64;

  void begin(Settings* settings, TxLine* tx) {
    cfg_ = settings;
    tx_ = tx;
    if (kPins.fwdAdc >= 0) analogSetPinAttenuation(kPins.fwdAdc, ADC_11db);
    if (kPins.revAdc >= 0) analogSetPinAttenuation(kPins.revAdc, ADC_11db);
    if (kPins.ntcAdc >= 0) analogSetPinAttenuation(kPins.ntcAdc, ADC_11db);
    analogReadResolution(12);
    // Core 0, above the Arduino loop's priority so a busy loop never delays an
    // overload trip. It sleeps 1 ms per pair, so the idle task still runs.
    xTaskCreatePinnedToCore(&BridgeSensor::taskEntry, "bridge", 4096, this,
                            configMAX_PRIORITIES - 5, &task_, 0);
  }

  // Most recent rolling reading (about the last 32 ms).
  SensorReading latest() const {
    portENTER_CRITICAL(&mux_);
    SensorReading r = latest_;
    portEXIT_CRITICAL(&mux_);
    return r;
  }

  // Starts an averaged measurement over `pairs` FWD/REV pairs taken from now
  // on. Replaces any request still in progress.
  void requestMeasurement(uint8_t pairs) {
    if (pairs < 1) pairs = 1;
    if (pairs > kMaxSamples) pairs = kMaxSamples;
    portENTER_CRITICAL(&mux_);
    accWant_ = pairs;
    accReset_ = true;
    accReady_ = false;
    portEXIT_CRITICAL(&mux_);
  }

  bool takeMeasurement(SensorReading& out) {
    portENTER_CRITICAL(&mux_);
    bool ready = accReady_;
    if (ready) {
      out = accResult_;
      accReady_ = false;
    }
    portEXIT_CRITICAL(&mux_);
    return ready;
  }

  // For console commands only: waits for the measurement to complete.
  SensorReading measureBlocking(uint8_t pairs) {
    requestMeasurement(pairs);
    SensorReading r;
    uint32_t t0 = millis();
    while (!takeMeasurement(r)) {
      if (elapsed(t0) > 2000) break;
      delay(1);
    }
    return r;
  }

  // True once after the task has seen an overload. The task has already
  // latched the TX inhibit line by the time this returns true.
  bool takeTrip() { return trip_.exchange(false); }

  // Records `n` raw pairs at the ADC's natural rate. The buffer must stay valid
  // until traceDone() returns true.
  void startTrace(TracePoint* buf, uint16_t n) {
    traceN_ = n;
    traceIdx_ = 0;
    traceBuf_ = buf;
  }
  bool traceDone() const { return traceBuf_ == nullptr; }

  // Solve powerScale from a measurement at a known power into a dummy load.
  // Returns false if there is not enough forward voltage to calibrate against.
  bool solvePowerScale(const SensorReading& r, float actualWatts, float& outScale) const {
    if (actualWatts <= 0.0f) return false;
    float vf = forwardVolts(r);
    if (vf <= 0.0f) return false;
    outScale = (vf * vf) / actualWatts;
    return outScale > 0.0f;
  }

  float forwardVolts(const SensorReading& r) const {
    float mv = (r.fwdMv - cfg_->fwdOffsetMv) * cfg_->fwdScale;
    return (mv > 0.0f) ? mv / 1000.0f : 0.0f;
  }

  // Detector voltage after offset and scale (mV). Used by `cal rev <swr>`.
  float reverseCorrectedMv(float revMv) const {
    return std::max(0.0f, (revMv - cfg_->revOffsetMv) * cfg_->revScale);
  }
  float forwardCorrectedMv(float fwdMv) const {
    return std::max(0.0f, (fwdMv - cfg_->fwdOffsetMv) * cfg_->fwdScale);
  }

  // Converts a pair of averaged detector voltages to a reading.
  SensorReading fromMillivolts(float fwdMv, float revMv) const {
    SensorReading r;
    r.fwdMv = fwdMv;
    r.revMv = revMv;
    r.fwdRaw = fwdMv * (4095.0f / 3100.0f);
    r.revRaw = revMv * (4095.0f / 3100.0f);
    r.pairs = 1;
    Pair p = convert(fwdMv, revMv);
    r.powerW = p.valid ? p.powerW : 0.0f;
    r.peakPowerW = p.powerW;
    r.valid = p.valid;
    r.gamma = p.valid ? p.gamma : 0.0f;
    r.swr = p.valid ? gammaToSwr(p.gamma) : 1.0f;
    return r;
  }

 private:
  struct Pair {
    float fwdMv, revMv;
    float gamma;
    float powerW;
    bool valid;
  };

  static constexpr uint8_t kWindow = 32;
  static constexpr uint8_t kPublishEvery = 8;
  static constexpr uint8_t kTripPairs = 2;

  Settings* cfg_ = nullptr;
  TxLine* tx_ = nullptr;
  TaskHandle_t task_ = nullptr;
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  SensorReading latest_;

  // Rolling window (task-only)
  float winFwd_[kWindow]{};
  float winRev_[kWindow]{};
  float winPow_[kWindow]{};
  uint8_t winIdx_ = 0;
  uint8_t winFill_ = 0;
  uint8_t sincePublish_ = 0;

  // Measurement accumulator. accWant_/accReset_/accReady_/accResult_ are
  // shared and guarded by mux_; the rest are task-only.
  uint8_t accWant_ = 0;
  bool accReset_ = false;
  bool accReady_ = false;
  SensorReading accResult_;
  uint8_t accTarget_ = 0;
  uint8_t accN_ = 0;
  uint8_t accValid_ = 0;
  float accFwd_ = 0, accRev_ = 0, accPow_ = 0, accPeak_ = 0;
  float accGamma_[kMaxSamples]{};

  std::atomic<bool> trip_{false};
  uint8_t tripRun_ = 0;

  std::atomic<TracePoint*> traceBuf_{nullptr};
  uint16_t traceN_ = 0;
  uint16_t traceIdx_ = 0;

  static void taskEntry(void* arg) { static_cast<BridgeSensor*>(arg)->run(); }

  static float readMv(int pin) {
    return pin >= 0 ? static_cast<float>(analogReadMilliVolts(pin)) : 0.0f;
  }

  Pair convert(float fwdMv, float revMv) const {
    Pair p;
    p.fwdMv = fwdMv;
    p.revMv = revMv;
    float vf = std::max(0.0f, (fwdMv - cfg_->fwdOffsetMv) * cfg_->fwdScale);
    float vr = std::max(0.0f, (revMv - cfg_->revOffsetMv) * cfg_->revScale);
    p.valid = vf >= cfg_->swrMinForward;
    // Reverse can exceed forward on a badly built bridge; clamp so the SWR
    // maths stays finite.
    p.gamma = p.valid ? std::min(vr / vf, kGammaCeiling) : 0.0f;
    float vfV = vf / 1000.0f;
    p.powerW = std::max(0.0f, vfV * vfV / cfg_->powerScale);
    return p;
  }

  void run() {
    esp_task_wdt_add(nullptr);
    for (;;) {
      esp_task_wdt_reset();

      TracePoint* tb = traceBuf_.load();
      if (tb) {
        // Tight sampling for the settle test; protection still runs on every
        // pair.
        TracePoint& t = tb[traceIdx_];
        t.fwdMv = readMv(kPins.fwdAdc);
        t.revMv = readMv(kPins.revAdc);
        t.us = micros();
        protect(convert(t.fwdMv, t.revMv));
        if (++traceIdx_ >= traceN_) traceBuf_ = nullptr;
        if ((traceIdx_ & 0x3F) == 0) vTaskDelay(1);
        continue;
      }

      float f = readMv(kPins.fwdAdc);
      float r = readMv(kPins.revAdc);
      Pair p = convert(f, r);
      protect(p);
      window(p);
      accumulate(p);
      vTaskDelay(1);
    }
  }

  void protect(const Pair& p) {
    if (!cfg_->powerProtEnabled) {
      tripRun_ = 0;
      return;
    }
    if (p.powerW >= cfg_->powerLimitW) {
      if (tripRun_ < 255) ++tripRun_;
      // Two consecutive pairs, so a single ADC glitch cannot trip it. Checking
      // the latch rather than the edge means an overload that is still present
      // after 'power reset' trips again straight away.
      if (tripRun_ >= kTripPairs && tx_ && !tx_->latched()) {
        tx_->latch();
        trip_ = true;
      }
    } else {
      tripRun_ = 0;
    }
  }

  void window(const Pair& p) {
    winFwd_[winIdx_] = p.fwdMv;
    winRev_[winIdx_] = p.revMv;
    winPow_[winIdx_] = p.powerW;
    winIdx_ = static_cast<uint8_t>((winIdx_ + 1) % kWindow);
    if (winFill_ < kWindow) ++winFill_;
    if (++sincePublish_ < kPublishEvery) return;
    sincePublish_ = 0;

    float fs = 0, rs = 0, peak = 0;
    for (uint8_t i = 0; i < winFill_; ++i) {
      fs += winFwd_[i];
      rs += winRev_[i];
      if (winPow_[i] > peak) peak = winPow_[i];
    }
    SensorReading out = fromMillivolts(fs / winFill_, rs / winFill_);
    out.pairs = winFill_;
    // Squaring the mean voltage understates SSB peaks badly; relay safety
    // decisions use the peak instead.
    out.peakPowerW = peak;
    portENTER_CRITICAL(&mux_);
    latest_ = out;
    portEXIT_CRITICAL(&mux_);
  }

  void accumulate(const Pair& p) {
    portENTER_CRITICAL(&mux_);
    bool reset = accReset_;
    uint8_t want = accWant_;
    accReset_ = false;
    portEXIT_CRITICAL(&mux_);

    if (reset) {
      accTarget_ = want;
      accN_ = accValid_ = 0;
      accFwd_ = accRev_ = accPow_ = accPeak_ = 0.0f;
    }
    if (accTarget_ == 0) return;

    accFwd_ += p.fwdMv;
    accRev_ += p.revMv;
    accPow_ += p.valid ? p.powerW : 0.0f;
    if (p.powerW > accPeak_) accPeak_ = p.powerW;
    if (p.valid) accGamma_[accValid_++] = p.gamma;
    if (++accN_ < accTarget_) return;

    SensorReading out;
    out.fwdMv = accFwd_ / accN_;
    out.revMv = accRev_ / accN_;
    out.fwdRaw = out.fwdMv * (4095.0f / 3100.0f);
    out.revRaw = out.revMv * (4095.0f / 3100.0f);
    out.pairs = accN_;
    out.valid = accValid_ * 2 >= accN_;
    out.powerW = out.valid ? accPow_ / accN_ : 0.0f;
    out.peakPowerW = accPeak_;
    if (out.valid) {
      out.gamma = median(accGamma_, accValid_);
      out.gammaNoise = noiseOf(accGamma_, accValid_, out.gamma);
      out.swr = gammaToSwr(out.gamma);
    }
    accTarget_ = 0;

    portENTER_CRITICAL(&mux_);
    if (!accReset_) {       // a newer request supersedes this result
      accResult_ = out;
      accReady_ = true;
    }
    portEXIT_CRITICAL(&mux_);
  }

  // Sorts in place; n <= kMaxSamples.
  static float median(float* v, uint8_t n) {
    for (uint8_t i = 1; i < n; ++i) {
      float k = v[i];
      int j = i - 1;
      while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; --j; }
      v[j + 1] = k;
    }
    return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
  }

  // Standard error of the median from the median absolute deviation.
  static float noiseOf(const float* sorted, uint8_t n, float med) {
    if (n < 3) return 0.05f;
    float dev[kMaxSamples];
    for (uint8_t i = 0; i < n; ++i) dev[i] = std::fabs(sorted[i] - med);
    float mad = median(dev, n);
    float sigma = 1.4826f * mad;
    float se = 1.2533f * sigma / std::sqrt(static_cast<float>(n));
    return std::max(se, 0.001f);
  }
};

}  // namespace atu
