#pragma once
//
// SWR bridge front end.
//
// Power formula (matches README and the calibration wizard):
//   Vf = max(0, (fwdMv - fwdOffsetMv)) * fwdScale
//   P  = (Vf / 1000)^2 / powerScale
//

#include <Arduino.h>

#include <algorithm>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

class BridgeSensor {
 public:
  void begin(Settings* settings) {
    cfg_ = settings;
    if (kPins.fwdAdc >= 0) analogSetPinAttenuation(kPins.fwdAdc, ADC_11db);
    if (kPins.revAdc >= 0) analogSetPinAttenuation(kPins.revAdc, ADC_11db);
    if (kPins.ntcAdc >= 0) analogSetPinAttenuation(kPins.ntcAdc, ADC_11db);
    analogReadResolution(12);
  }

  SensorReading readOnce() const {
    SensorReading out;
    if (kPins.fwdAdc >= 0) {
      out.fwdMv = static_cast<float>(analogReadMilliVolts(kPins.fwdAdc));
      // Reconstruct the raw count from the calibrated mV rather than paying for
      // a second conversion per channel per sample.
      out.fwdRaw = out.fwdMv * (4095.0f / 3100.0f);
    }
    if (kPins.revAdc >= 0) {
      out.revMv = static_cast<float>(analogReadMilliVolts(kPins.revAdc));
      out.revRaw = out.revMv * (4095.0f / 3100.0f);
    }
    calculate(out);
    return out;
  }

  // Blocking, but only for samples*spacing ms (~9 ms at the defaults). The
  // long relay settle is handled by the caller's state machine, not here.
  SensorReading readAverage(uint8_t samples, uint16_t spacingMs) const {
    if (samples == 0) return SensorReading{};

    float fwdSum = 0, revSum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
      if (kPins.fwdAdc >= 0) fwdSum += static_cast<float>(analogReadMilliVolts(kPins.fwdAdc));
      if (kPins.revAdc >= 0) revSum += static_cast<float>(analogReadMilliVolts(kPins.revAdc));
      if (spacingMs > 0 && i + 1 < samples) delay(spacingMs);
    }

    SensorReading out;
    out.fwdMv = fwdSum / samples;
    out.revMv = revSum / samples;
    out.fwdRaw = out.fwdMv * (4095.0f / 3100.0f);
    out.revRaw = out.revMv * (4095.0f / 3100.0f);
    calculate(out);
    return out;
  }

  SensorReading readDefault() const {
    return readAverage(cfg_->measureSamples, cfg_->measureSampleSpacingMs);
  }

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

 private:
  Settings* cfg_ = nullptr;

  void calculate(SensorReading& r) const {
    float vf = std::max(0.0f, (r.fwdMv - cfg_->fwdOffsetMv) * cfg_->fwdScale);
    float vr = std::max(0.0f, (r.revMv - cfg_->revOffsetMv) * cfg_->revScale);

    if (vf < cfg_->swrMinForward) {
      // No usable forward power. Report "unknown" rather than 99:1, so the UI
      // can draw a blank bar instead of a pegged one.
      r.swr = 1.0f;
      r.powerW = 0.0f;
      r.valid = false;
      return;
    }

    r.valid = true;
    // Reverse can exceed forward on a badly built bridge; clamp so the SWR
    // maths stays finite.
    float gamma = clampf(vr / vf, 0.0f, 0.99f);
    r.swr = clampf((1.0f + gamma) / (1.0f - gamma), 1.0f, 99.0f);

    float vfV = vf / 1000.0f;
    r.powerW = std::max(0.0f, vfV * vfV / cfg_->powerScale);
  }
};

}  // namespace atu
