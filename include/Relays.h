#pragma once
//
// Relay bank driver.
//
// Adds over the original: per-relay cycle counters persisted to NVS and an
// antenna selector output.
//
// There is deliberately no latching-relay mode. Each relay has a single
// low-side driver, which can pulse a coil on but has no way to deliver the
// reset pulse (second coil or reversed polarity) a latching relay needs to
// turn off again, so such a mode could never release a relay.
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
  // `initial` is driven straight away, so a restored tune comes back without
  // first dropping every relay and picking them up again.
  void begin(Settings* settings, const RelayState& initial) {
    cfg_ = settings;
    applied_ = initial;

    prefs_.begin("atu_cycles", false);
    size_t got = prefs_.getBytes("cycles", cycles_, sizeof(cycles_));
    if (got != sizeof(cycles_)) memset(cycles_, 0, sizeof(cycles_));

    // Set the output latch before enabling the driver so the line never
    // glitches to the wrong level.
    writeAll(applied_);
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      enableOutput(kPins.lRelays[i]);
      enableOutput(kPins.cRelays[i]);
    }
    enableOutput(kPins.topologyRelay);
    for (uint8_t i = 0; i < kAntennaPinCount; ++i) {
      if (kPins.antenna[i] >= 0) digitalWrite(kPins.antenna[i], LOW);
      enableOutput(kPins.antenna[i]);
    }
    settleStartMs_ = millis();
  }

  void apply(const RelayState& state) {
    if (state == applied_) return;
    countTransitions(applied_, state);
    applied_ = state;
    writeAll(state);
    settleStartMs_ = millis();
  }

  void loop() {
    if (cyclesDirty_ && elapsed(cyclesDirtyMs_) > 30000) saveCycles();
  }

  // True once the relays have had time to physically settle after the last
  // change. Callers poll this instead of blocking on delay().
  uint32_t settleMs() const { return cfg_->relaySettleMs; }
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
  uint32_t cycles_[kTrackedRelayCount]{};
  bool cyclesDirty_ = false;
  uint32_t cyclesDirtyMs_ = 0;
  uint32_t settleStartMs_ = 0;
  uint8_t antenna_ = 0;

  static void enableOutput(int pin) {
    if (pin >= 0) pinMode(pin, OUTPUT);
  }

  void writeRelay(int pin, bool on) const {
    if (pin < 0) return;
    bool level = cfg_->relayActiveHigh ? on : !on;
    digitalWrite(pin, level ? HIGH : LOW);
  }

  void writeAll(const RelayState& state) {
    uint8_t lMask = state.bypass ? 0 : state.lMask;
    uint8_t cMask = state.bypass ? 0 : state.cMask;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      writeRelay(kPins.lRelays[i], (lMask >> i) & 0x01);
      writeRelay(kPins.cRelays[i], (cMask >> i) & 0x01);
    }
    writeRelay(kPins.topologyRelay, state.topology);
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
