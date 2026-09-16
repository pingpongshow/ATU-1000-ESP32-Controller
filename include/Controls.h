#pragma once
//
// Buttons, status LED, the TX request / inhibit line and the power warning
// state machine.
//

#include <Arduino.h>

#include <atomic>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

class DebouncedButton {
 public:
  void begin(int pin, Settings* settings) {
    pin_ = pin;
    cfg_ = settings;
    if (pin_ < 0) { enabled_ = false; return; }
    enabled_ = true;
    // The original always enabled a pull-up, so setting buttonsActiveLow=false
    // left the input floating.
    pinMode(pin_, cfg_->buttonsActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
    stable_ = readRaw();
    lastRaw_ = stable_;
    lastDebounceMs_ = millis();
  }

  void loop() {
    shortEvent_ = longEvent_ = false;
    if (!enabled_) return;

    bool raw = readRaw();
    if (raw != lastRaw_) { lastRaw_ = raw; lastDebounceMs_ = millis(); }
    if (elapsed(lastDebounceMs_) < kDebounceMs) return;

    if (raw != stable_) {
      stable_ = raw;
      if (stable_) {
        pressedMs_ = millis();
        longFired_ = false;
      } else {
        uint32_t held = elapsed(pressedMs_);
        if (held >= kDebounceMs && held < kLongPressMs && !longFired_) shortEvent_ = true;
      }
      return;
    }

    if (stable_ && !longFired_ && elapsed(pressedMs_) >= kLongPressMs) {
      longFired_ = true;
      longEvent_ = true;
    }
  }

  bool shortPress() const { return shortEvent_; }
  bool longPress() const { return longEvent_; }
  bool anyPress() const { return shortEvent_ || longEvent_; }
  bool isPressed() const { return enabled_ && stable_; }

 private:
  static constexpr uint32_t kDebounceMs = 30;
  static constexpr uint32_t kLongPressMs = 800;

  int pin_ = -1;
  Settings* cfg_ = nullptr;
  bool enabled_ = false;
  bool stable_ = false;
  bool lastRaw_ = false;
  bool shortEvent_ = false;
  bool longEvent_ = false;
  bool longFired_ = false;
  uint32_t lastDebounceMs_ = 0;
  uint32_t pressedMs_ = 0;

  bool readRaw() const {
    bool low = digitalRead(pin_) == LOW;
    return cfg_->buttonsActiveLow ? low : !low;
  }
};

// -----------------------------------------------------------------------------

// The TX request line (and its inverted twin) is driven from two cores: the
// main loop asserts it around a tune, and the sensor task asserts it the
// moment it sees an overload. A safety latch must never be released by the
// tune logic finishing, so the two sources are kept separate and OR-ed.
class TxLine {
 public:
  void begin(Settings* settings) {
    cfg_ = settings;
    write(false);
    if (kPins.txReq >= 0) pinMode(kPins.txReq, OUTPUT);
    if (kPins.txReqInv >= 0) pinMode(kPins.txReqInv, OUTPUT);
    write(false);
  }

  // Tune logic. Cannot release a safety latch.
  void request(bool active) {
    requested_ = active;
    write(active || latched_);
    // The sensor task may have latched between the read above and the write.
    if (!active && latched_) write(true);
  }

  // Safety inhibit. Safe to call from the sensor task.
  void latch() {
    latched_ = true;
    write(true);
  }

  void releaseLatch() {
    latched_ = false;
    write(requested_);
    // The sensor task may have latched again between the two lines above.
    if (latched_) write(true);
  }

  bool latched() const { return latched_; }
  bool asserted() const { return requested_ || latched_; }

 private:
  Settings* cfg_ = nullptr;
  std::atomic<bool> requested_{false};
  std::atomic<bool> latched_{false};

  void write(bool active) const {
    if (!cfg_) return;
    bool level = cfg_->requestTxActiveHigh ? active : !active;
    if (kPins.txReq >= 0) digitalWrite(kPins.txReq, level ? HIGH : LOW);
    if (kPins.txReqInv >= 0) digitalWrite(kPins.txReqInv, level ? LOW : HIGH);
  }
};

// -----------------------------------------------------------------------------

class StatusLed {
 public:
  void begin() {
    if (kPins.ledStatus < 0) return;
    pinMode(kPins.ledStatus, OUTPUT);
    off();
  }
  void on() { if (kPins.ledStatus >= 0) digitalWrite(kPins.ledStatus, HIGH); }
  void off() { if (kPins.ledStatus >= 0) digitalWrite(kPins.ledStatus, LOW); }
  void blink(uint16_t onMs, uint16_t offMs) {
    if (kPins.ledStatus < 0 || (onMs + offMs) == 0) return;
    uint32_t phase = millis() % (onMs + offMs);
    if (phase < onMs) on(); else off();
  }
};

// -----------------------------------------------------------------------------

// Warning level and the slow (loop-rate) overload check. The fast overload
// trip lives in the sensor task (Sensor.h).
class PowerProtection {
 public:
  void begin(Settings* settings) { cfg_ = settings; }

  void update(float powerW) {
    if (!cfg_->powerProtEnabled) {
      warning_ = false;
      overload_ = false;
      return;
    }

    warning_ = (powerW >= cfg_->powerWarningW && powerW < cfg_->powerLimitW);

    if (powerW >= cfg_->powerLimitW) {
      if (!overload_) {
        overload_ = true;
        triggered_ = true;
      }
      overloadMs_ = millis();   // keep the cooldown alive while power stays up
    } else if (overload_) {
      if (elapsed(overloadMs_) >= cfg_->powerOverloadHoldoffMs) overload_ = false;
    }
  }

  // Consumed once by the caller, which then engages bypass.
  bool takeTrigger() {
    bool t = triggered_;
    triggered_ = false;
    return t;
  }

  bool warning() const { return warning_; }
  bool overload() const { return overload_; }
  void reset() { overload_ = false; triggered_ = false; warning_ = false; }

 private:
  Settings* cfg_ = nullptr;
  bool warning_ = false;
  bool overload_ = false;
  bool triggered_ = false;
  uint32_t overloadMs_ = 0;
};

}  // namespace atu
