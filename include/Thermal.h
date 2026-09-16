#pragma once
//
// Temperature monitoring and thermal foldback.
//
// Source priority:
//   1. An LM75 / TMP102 / TMP75 compatible I2C sensor on the display bus
//      (auto-probed at 0x48..0x4F). Costs no extra GPIO, which matters because
//      every ADC1 pin on this board is already spoken for.
//   2. An NTC thermistor on kPins.ntcAdc, if you free up an ADC1 pin.
//   3. The ESP32-S3 internal die sensor, reported for information only.
//
// Escalation: warn -> foldback (inhibit TX and refuse to tune) -> limit
// (engage bypass). The die sensor never escalates: it measures the chip, which
// runs warm on its own with Wi-Fi up, not the relays or the inductors.
//

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

enum class TempSource : uint8_t { None, I2c, Ntc, Internal };

enum class ThermalLevel : uint8_t { Ok, Warn, Foldback, Shutdown };

class Thermal {
 public:
  // Call after Wire.begin(). `found` addresses come from the shared I2C scan.
  void begin(Settings* settings) {
    cfg_ = settings;
    source_ = TempSource::None;
    if (!cfg_->thermalEnabled) return;

    // Only touch the bus if it was actually brought up by the display layer.
    if (kPins.i2cSda >= 0 && kPins.i2cScl >= 0) {
      if (cfg_->tempAddress != 0 && probe(cfg_->tempAddress)) {
        addr_ = cfg_->tempAddress;
        source_ = TempSource::I2c;
      } else {
        for (uint8_t a = 0x48; a <= 0x4F; ++a) {
          if (probe(a)) { addr_ = a; source_ = TempSource::I2c; break; }
        }
      }
    }

    if (source_ == TempSource::None && kPins.ntcAdc >= 0) source_ = TempSource::Ntc;
    if (source_ == TempSource::None) source_ = TempSource::Internal;

    lastReadMs_ = millis() - 1000;
    loop();
  }

  void loop() {
    if (!cfg_->thermalEnabled || source_ == TempSource::None) {
      level_ = ThermalLevel::Ok;
      return;
    }
    if (elapsed(lastReadMs_) < 1000) return;
    lastReadMs_ = millis();

    float t;
    if (!read(t)) return;

    // Light smoothing; a 1 Hz sample rate on a lump of aluminium does not need
    // more than this.
    tempC_ = valid_ ? (tempC_ * 0.7f + t * 0.3f) : t;
    valid_ = true;

    if (source_ == TempSource::Internal) {
      level_ = ThermalLevel::Ok;
      return;
    }

    // Hysteresis of 3 C on the way back down, so a sensor sitting exactly on a
    // threshold does not chatter the TX inhibit line.
    const float h = 3.0f;
    if (tempC_ >= cfg_->tempLimitC) level_ = ThermalLevel::Shutdown;
    else if (tempC_ >= cfg_->tempFoldbackC) level_ = maxLevel(level_, ThermalLevel::Foldback);
    else if (tempC_ >= cfg_->tempWarnC) level_ = maxLevel(level_, ThermalLevel::Warn);

    if (level_ == ThermalLevel::Shutdown && tempC_ < cfg_->tempLimitC - h) level_ = ThermalLevel::Foldback;
    if (level_ == ThermalLevel::Foldback && tempC_ < cfg_->tempFoldbackC - h) level_ = ThermalLevel::Warn;
    if (level_ == ThermalLevel::Warn && tempC_ < cfg_->tempWarnC - h) level_ = ThermalLevel::Ok;
  }

  bool available() const { return source_ != TempSource::None && valid_; }
  // False for the die sensor, which is informational only.
  bool protects() const { return source_ == TempSource::I2c || source_ == TempSource::Ntc; }
  float tempC() const { return tempC_; }
  ThermalLevel level() const { return cfg_ && cfg_->thermalEnabled ? level_ : ThermalLevel::Ok; }
  bool inhibitsTx() const { return level() >= ThermalLevel::Foldback; }
  bool demandsBypass() const { return level() == ThermalLevel::Shutdown; }
  uint8_t address() const { return addr_; }

  const char* sourceName() const {
    switch (source_) {
      case TempSource::I2c: return "I2C";
      case TempSource::Ntc: return "NTC";
      case TempSource::Internal: return "chip";
      default: return "none";
    }
  }

  static const char* levelName(ThermalLevel l) {
    switch (l) {
      case ThermalLevel::Warn: return "WARM";
      case ThermalLevel::Foldback: return "HOT";
      case ThermalLevel::Shutdown: return "OVERTEMP";
      default: return "OK";
    }
  }

 private:
  Settings* cfg_ = nullptr;
  TempSource source_ = TempSource::None;
  ThermalLevel level_ = ThermalLevel::Ok;
  uint8_t addr_ = 0;
  float tempC_ = 0.0f;
  bool valid_ = false;
  uint32_t lastReadMs_ = 0;

  static ThermalLevel maxLevel(ThermalLevel a, ThermalLevel b) {
    return (static_cast<uint8_t>(a) > static_cast<uint8_t>(b)) ? a : b;
  }

  static bool probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) return false;
    // LM75/TMP102 register 0 is the 16-bit temperature. A device that answers
    // with a plausible -40..125 C reading is almost certainly one of them.
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(static_cast<int>(addr), 2) != 2) return false;
    int hi = Wire.read();
    int lo = Wire.read();
    float t = decode(static_cast<uint8_t>(hi), static_cast<uint8_t>(lo));
    return t > -40.0f && t < 125.0f;
  }

  // Both families put the MSB first and left-justify: LM75 is 9-bit (0.5 C),
  // TMP102 is 12-bit (0.0625 C). Treating everything as 12-bit and masking the
  // unused low bits reads both correctly.
  static float decode(uint8_t hi, uint8_t lo) {
    int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    return static_cast<float>(raw >> 4) * 0.0625f;
  }

  bool read(float& out) {
    switch (source_) {
      case TempSource::I2c: {
        Wire.beginTransmission(addr_);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) return false;
        if (Wire.requestFrom(static_cast<int>(addr_), 2) != 2) return false;
        uint8_t hi = static_cast<uint8_t>(Wire.read());
        uint8_t lo = static_cast<uint8_t>(Wire.read());
        out = decode(hi, lo);
        return out > -55.0f && out < 150.0f;
      }
      case TempSource::Ntc: {
        if (kPins.ntcAdc < 0) return false;
        float mv = static_cast<float>(analogReadMilliVolts(kPins.ntcAdc));
        if (mv <= 1.0f || mv >= 3200.0f) return false;
        // Divider: 3.3V - Rseries - NTC - GND, ADC across the NTC.
        float v = mv / 1000.0f;
        float rNtc = cfg_->ntcSeriesOhms * v / (3.3f - v);
        if (rNtc <= 0.0f) return false;
        // Beta equation, referenced to 25 C = 298.15 K.
        float steinhart = logf(rNtc / cfg_->ntcNominalOhms) / cfg_->ntcBeta;
        steinhart += 1.0f / 298.15f;
        out = (1.0f / steinhart) - 273.15f;
        return out > -55.0f && out < 150.0f;
      }
      case TempSource::Internal: {
        out = temperatureRead();
        return out > -55.0f && out < 150.0f;
      }
      default:
        return false;
    }
  }
};

}  // namespace atu
