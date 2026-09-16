#pragma once
//
// Relay bank driver.
//
// Adds over the original: pulsed (impulse-relay) drive mode, per-relay cycle
// counters persisted to NVS, and an antenna selector output.
//

#include <Arduino.h>
#include <Preferences.h>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

// 7 L + 7 C + 1 topology
constexpr uint8_t kTrackedRelayCount = kRelayCount * 2 + 1;

class RelayController {
 public:
  void begin(Settings* settings) {
    cfg_ = settings;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      setupOutput(kPins.lRelays[i]);
      setupOutput(kPins.cRelays[i]);
    }
    setupOutput(kPins.topologyRelay);

    prefs_.begin("atu_cycles", false);
    size_t got = prefs_.getBytes("cycles", cycles_, sizeof(cycles_));
    if (got != sizeof(cycles_)) memset(cycles_, 0, sizeof(cycles_));

    for (uint8_t i = 0; i < kAntennaPinCount; ++i) setupOutput(kPins.antenna[i]);

    applied_ = RelayState{};
    driven_ = RelayState{};
    writeAll(applied_, /*force=*/true);
  }

  // Requested state. In pulsed mode the physical lines are released again by
  // loop() once latchPulseMs has expired.
  void apply(const RelayState& state) {
    RelayState next = state;
    if (next != applied_) {
      countTransitions(applied_, next);
      applied_ = next;
      writeAll(next, /*force=*/false);
      pulseStartMs_ = millis();
      pulseActive_ = (mode() == RelayMode::Pulsed);
      settleStartMs_ = millis();
    }
  }

  void loop() {
    if (pulseActive_ && elapsed(pulseStartMs_) >= cfg_->latchPulseMs) {
      pulseActive_ = false;
      // Release every coil; a latching relay holds its last commanded position.
      for (uint8_t i = 0; i < kRelayCount; ++i) {
        writeRelay(kPins.lRelays[i], false);
        writeRelay(kPins.cRelays[i], false);
      }
      writeRelay(kPins.topologyRelay, false);
    }
    if (cyclesDirty_ && elapsed(cyclesDirtyMs_) > 30000) saveCycles();
  }

  // True once the relays have had time to physically settle after the last
  // change. Callers poll this instead of blocking on delay(). In pulsed mode
  // the coil drive itself outlasts the nominal settle time, so wait for the
  // pulse to finish as well or measurements land mid-transition.
  uint32_t settleMs() const {
    uint32_t s = cfg_->relaySettleMs;
    if (mode() == RelayMode::Pulsed) {
      uint32_t p = static_cast<uint32_t>(cfg_->latchPulseMs) + 5U;
      if (p > s) s = p;
    }
    return s;
  }
  bool settled() const { return elapsed(settleStartMs_) >= settleMs(); }
  uint32_t settleRemainingMs() const {
    uint32_t e = elapsed(settleStartMs_);
    uint32_t s = settleMs();
    return (e >= s) ? 0 : (s - e);
  }

  const RelayState& current() const { return applied_; }

  float totalL() const {
    float sum = 0.0f;
    if (applied_.bypass) return 0.0f;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      if ((applied_.lMask >> i) & 0x01) sum += kLValuesUh[i];
    }
    return sum;
  }

  uint16_t totalC() const {
    uint16_t sum = 0;
    if (applied_.bypass) return 0;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      if ((applied_.cMask >> i) & 0x01) sum += kCValuesPf[i];
    }
    return sum;
  }

  uint32_t cycles(uint8_t idx) const {
    return (idx < kTrackedRelayCount) ? cycles_[idx] : 0;
  }
  uint32_t totalCycles() const {
    uint32_t t = 0;
    for (uint8_t i = 0; i < kTrackedRelayCount; ++i) t += cycles_[i];
    return t;
  }
  void resetCycles() {
    memset(cycles_, 0, sizeof(cycles_));
    saveCycles();
  }
  void saveCycles() {
    prefs_.putBytes("cycles", cycles_, sizeof(cycles_));
    cyclesDirty_ = false;
  }

  // Antenna selector: binary coded across however many pins are configured.
  void setAntenna(uint8_t index) {
    for (uint8_t i = 0; i < kAntennaPinCount; ++i) {
      if (kPins.antenna[i] < 0) continue;
      digitalWrite(kPins.antenna[i], ((index >> i) & 0x01) ? HIGH : LOW);
    }
    antenna_ = index;
  }
  uint8_t antenna() const { return antenna_; }
  static bool antennaSupported() {
    for (uint8_t i = 0; i < kAntennaPinCount; ++i) {
      if (kPins.antenna[i] >= 0) return true;
    }
    return false;
  }

 private:
  Settings* cfg_ = nullptr;
  Preferences prefs_;
  RelayState applied_{};
  RelayState driven_{};
  uint32_t cycles_[kTrackedRelayCount]{};
  bool cyclesDirty_ = false;
  uint32_t cyclesDirtyMs_ = 0;
  bool pulseActive_ = false;
  uint32_t pulseStartMs_ = 0;
  uint32_t settleStartMs_ = 0;
  uint8_t antenna_ = 0;

  RelayMode mode() const { return static_cast<RelayMode>(cfg_->relayMode); }

  void setupOutput(int pin) const {
    if (pin < 0) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, cfg_ && !cfg_->relayActiveHigh ? HIGH : LOW);
  }

  void writeRelay(int pin, bool on) const {
    if (pin < 0) return;
    bool level = cfg_->relayActiveHigh ? on : !on;
    digitalWrite(pin, level ? HIGH : LOW);
  }

  void writeAll(const RelayState& state, bool force) {
    (void)force;
    uint8_t lMask = state.bypass ? 0 : state.lMask;
    uint8_t cMask = state.bypass ? 0 : state.cMask;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      writeRelay(kPins.lRelays[i], (lMask >> i) & 0x01);
      writeRelay(kPins.cRelays[i], (cMask >> i) & 0x01);
    }
    writeRelay(kPins.topologyRelay, state.topology);
    driven_ = state;
  }

  void countTransitions(const RelayState& from, const RelayState& to) {
    uint8_t fromL = from.bypass ? 0 : from.lMask;
    uint8_t fromC = from.bypass ? 0 : from.cMask;
    uint8_t toL = to.bypass ? 0 : to.lMask;
    uint8_t toC = to.bypass ? 0 : to.cMask;

    for (uint8_t i = 0; i < kRelayCount; ++i) {
      if (((fromL >> i) & 1) != ((toL >> i) & 1)) bump(i);
      if (((fromC >> i) & 1) != ((toC >> i) & 1)) bump(kRelayCount + i);
    }
    if (from.topology != to.topology) bump(kRelayCount * 2);
  }

  void bump(uint8_t idx) {
    if (idx >= kTrackedRelayCount) return;
    if (cycles_[idx] < UINT32_MAX) ++cycles_[idx];
    if (!cyclesDirty_) { cyclesDirty_ = true; cyclesDirtyMs_ = millis(); }
  }
};

}  // namespace atu
