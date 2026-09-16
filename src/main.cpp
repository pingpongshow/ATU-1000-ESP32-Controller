// =============================================================================
// ATU-1000 ESP32-S3 Controller
//
// Single firmware image for the LCD and OLED hardware variants; the display is
// probed on the I2C bus at boot and the matching driver is instantiated.
//
// Everything in loop() is non-blocking, so buttons, the console and the web UI
// stay responsive during a tune or a sweep. The SWR bridge and the fast
// overload trip run in their own task on the other core (Sensor.h), so
// protection does not depend on the loop at all. Both are covered by the task
// watchdog.
//
// Relay safety rule: outside a tune (which runs at or below pwrmax), the relays
// are only ever moved while forward power is at or below pwrmax. A request
// made while RF is higher is held and applied once RF drops.
// =============================================================================

#include <Arduino.h>

#include "App.h"
#include "Commands.h"
#include "Config.h"
#include "WebUi.h"

namespace atu {

App gApp;
WebUi gWeb;

namespace {

char gSerialLine[160];
size_t gSerialIndex = 0;

constexpr uint32_t kSwrAlarmOnMs = 500;
// SSB has gaps between syllables; RF must stay low this long before a held
// relay change or protection bypass is allowed to switch.
constexpr uint32_t kRfQuietMs = 300;
constexpr uint32_t kSwrAlarmOffMs = 1000;

}  // namespace

// -----------------------------------------------------------------------------
// Persistence of the live relay state
// -----------------------------------------------------------------------------

void saveRuntimeState() {
  RuntimeState s{};
  s.version = kRuntimeStateVersion;
  s.lMask = gApp.state.lMask;
  s.cMask = gApp.state.cMask;
  s.flags = packFlags(gApp.state);
  s.antenna = gApp.cfg().antenna;
  s.checksum = crc32(reinterpret_cast<uint8_t*>(&s), sizeof(s) - sizeof(s.checksum));
  gApp.statePrefs.putBytes("state", &s, sizeof(s));
}

void loadRuntimeState() {
  RuntimeState s{};
  if (gApp.statePrefs.getBytes("state", &s, sizeof(s)) != sizeof(s)) return;
  uint32_t calc = crc32(reinterpret_cast<uint8_t*>(&s), sizeof(s) - sizeof(s.checksum));
  if (calc != s.checksum || s.version != kRuntimeStateVersion) return;
  gApp.state.lMask = s.lMask & 0x7F;
  gApp.state.cMask = s.cMask & 0x7F;
  unpackFlags(s.flags, gApp.state);
  if (s.antenna < gApp.cfg().antennaCount) gApp.cfg().antenna = s.antenna;
}

// -----------------------------------------------------------------------------
// Shared actions
// -----------------------------------------------------------------------------

void setStatus(const char* text) { gApp.status.set(text); }

namespace {

bool rfLowNow() {
  return gApp.sensor.latest().peakPowerW <= gApp.cfg().maxTunePowerW;
}

// Called every loop so the quiet period is measured continuously.
void trackRfQuiet() {
  if (!rfLowNow()) {
    gApp.rfQuietSinceMs = 0;
  } else if (gApp.rfQuietSinceMs == 0) {
    gApp.rfQuietSinceMs = millis() | 1;
  }
}

}  // namespace

// Peak (not average) power at or below pwrmax, continuously for kRfQuietMs.
bool rfSafeToSwitch() {
  return rfLowNow() && gApp.rfQuietSinceMs != 0 && elapsed(gApp.rfQuietSinceMs) >= kRfQuietMs;
}

namespace {

void applyRelayStateNow(const RelayState& s) {
  gApp.state = s;
  gApp.relays.apply(s);
  gApp.relayPending = false;
}

}  // namespace

bool applyRelayState(const RelayState& s) {
  // Already there (typically the end of a tune): nothing physically switches.
  if (s == gApp.relays.current()) {
    applyRelayStateNow(s);
    return true;
  }
  // While protection is active nothing may take the network out of bypass or
  // move it; the request is kept and applied after 'power reset'.
  if (gApp.protectState != ProtectState::Clear) {
    gApp.pendingState = s;
    gApp.relayPending = true;
    Serial.println("Relay change held: protection is active");
    return false;
  }
  if (rfSafeToSwitch()) {
    applyRelayStateNow(s);
    return true;
  }
  gApp.pendingState = s;
  gApp.relayPending = true;
  gApp.status.set("RF HIGH - WAITING");
  Serial.printf("Relay change held: RF above %.0f W (pwrmax)\n", gApp.cfg().maxTunePowerW);
  return false;
}

RelayState desiredRelayState() {
  return gApp.relayPending ? gApp.pendingState : gApp.state;
}

void selectAntenna(uint8_t index) {
  Settings& c = gApp.cfg();
  if (index >= c.antennaCount) index = 0;
  c.antenna = index;
  gApp.relays.setAntenna(index);
  gApp.settingsStore.markDirty();
  // A different antenna is a different match; forget which memory entry we
  // already tried for this frequency.
  gApp.memTriedBin = UINT32_MAX;

  MemLookup m = gApp.memory.lookup(gApp.currentFreqHz, index);
  if (m.hit != MemHit::Miss) {
    applyRelayState(m.state);
    gApp.memory.noteUse(m.bin, index);
    gApp.status.printf("ANT%u MEM", index + 1);
  } else {
    gApp.status.printf("ANT%u", index + 1);
  }
  saveRuntimeState();
}

// Protection sequence:
//   1. assert TX inhibit (the sensor task has usually done this already),
//      unkey a CAT tune, stop any tune or sweep without touching the relays;
//   2. wait for forward power to fall to pwrmax, then engage bypass;
//   3. if RF is still present after `protwait` ms, keep holding the current
//      match rather than hot-switching the relay bank, and bypass as soon as
//      RF does drop.
void engageProtection(const char* reason) {
  App& a = gApp;
  if (a.protectState != ProtectState::Clear) return;

  strncpy(a.protectionReason, reason, sizeof(a.protectionReason) - 1);
  a.protectionReason[sizeof(a.protectionReason) - 1] = '\0';
  a.protectionLatched = true;
  a.tx.latch();

  a.catTune.emergencyUnkey();
  a.tuner.emergencyStop(strstr(reason, "TEMP") ? TuneResult::OverTemp
                                               : TuneResult::PowerOverload);
  a.sweep.stop();
  a.relayPending = false;
  // The tuner may have left the relays on a search probe.
  a.state = a.relays.current();

  a.protectState = ProtectState::WaitRfDrop;
  a.protectSinceMs = millis();
  a.protectHoldAnnounced = false;

  a.status.set(reason);
  a.display.noteActivity();
  a.display->overload(reason);
  // Hold the full-screen warning long enough to actually be read.
  a.showingProtection = true;
  a.showingSweep = false;
  a.protectionShownMs = millis();
  Serial.printf("*** PROTECTION: %s - TX inhibit asserted ***\n", reason);
}

void clearProtection() {
  App& a = gApp;
  a.powerProt.reset();
  a.sensor.takeTrip();
  bool wasBypassed = a.protectState == ProtectState::Bypassed;
  a.protectState = ProtectState::Clear;
  a.protectionLatched = false;
  a.protectionReason[0] = '\0';
  a.tx.releaseLatch();
  a.showingProtection = false;
  a.status.set("READY");
  a.display.noteActivity();
  if (a.relayPending) {
    // A change requested during protection, exactly as asked.
    RelayState s = a.pendingState;
    a.relayPending = false;
    applyRelayState(s);
  } else if (wasBypassed) {
    RelayState s = a.state;
    s.bypass = false;
    applyRelayState(s);
  }
}

// Everything that makes a tune refuse, apart from the RF level itself. Sets the
// status line when it refuses.
bool tuneAllowed(bool force) {
  App& a = gApp;
  Settings& c = a.cfg();

  if (a.tuner.running() || a.sweep.isRunning()) {
    a.status.set("BUSY");
    return false;
  }
  if (a.protectionLatched || a.powerProt.overload()) {
    a.status.set("PROTECT - RESET");
    return false;
  }
  if (a.thermal.inhibitsTx()) {
    a.status.set("TOO HOT");
    return false;
  }
  if (!force && elapsed(a.lastTuneMs) < c.retuneHoldoffMs) {
    a.status.set("WAIT - HOLDOFF");
    return false;
  }
  if (a.relayPending) {
    a.status.set("RF HIGH - WAITING");
    return false;
  }
  return true;
}

bool beginTune(bool force) {
  App& a = gApp;
  Settings& c = a.cfg();

  if (!tuneAllowed(force)) return false;

  // Checked before the TX request line is touched: asserting it and then
  // refusing would chop the operator's transmission.
  SensorReading r = a.sensor.latest();
  if (r.valid && r.powerW > c.maxTunePowerW) {
    a.status.printf("PWR HIGH >%.0fW", c.maxTunePowerW);
    return false;
  }

  RelayState hint;
  bool hasHint = false;
  if (a.currentFreqHz > 0) {
    MemLookup m = a.memory.lookup(a.currentFreqHz, c.antenna);
    if (m.hit != MemHit::Miss) { hint = m.state; hasHint = true; }
  }

  RelayState start = a.state;
  start.bypass = false;
  applyRelayStateNow(start);

  a.txReleasePending = false;
  a.tx.request(true);
  a.led.on();
  a.status.set("TUNING");
  a.display.noteActivity();

  if (!a.tuner.start(a.currentFreqHz, force, hasHint ? &hint : nullptr)) {
    a.tx.request(false);
    return false;
  }
  return true;
}

// TUNE button short press: CAT tune when enabled and the radio is not already
// transmitting, else a normal tune.
bool startTuneFromButton() {
  App& a = gApp;
  if (a.cfg().catTuneEnabled && a.cat.canKey()) {
    if (a.protectionLatched || a.thermal.inhibitsTx()) return beginTune(false);
    return a.catTune.start();
  }
  return beginTune(false);
}

void abortActivity(const char* status) {
  App& a = gApp;
  if (a.catTune.active()) a.catTune.abort(status);
  if (a.tuner.running()) a.tuner.abort();
  if (a.sweep.isRunning()) a.sweep.stop();
  if (status) a.status.set(status);
}

const char* displayKindName() { return gApp.display.kindName(); }

namespace {

// -----------------------------------------------------------------------------
// Loop tasks
// -----------------------------------------------------------------------------

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());

    if (ch == '\n' || ch == '\r') {
      Serial.println();
      if (gSerialIndex > 0) {
        gSerialLine[gSerialIndex] = '\0';
        handleCommand(gSerialLine, Serial);
        gSerialIndex = 0;
      }
      Serial.print("> ");
      continue;
    }
    if (ch == 8 || ch == 127) {
      if (gSerialIndex > 0) { --gSerialIndex; Serial.print("\b \b"); }
      continue;
    }
    if (gSerialIndex < sizeof(gSerialLine) - 1 && isPrintable(ch)) {
      gSerialLine[gSerialIndex++] = ch;
      Serial.print(ch);   // echo
    }
  }
}

void handleButtons() {
  App& a = gApp;
  Settings& c = a.cfg();

  a.tuneButton.loop();
  a.bypassButton.loop();
  a.autoButton.loop();

  bool any = a.tuneButton.anyPress() || a.bypassButton.anyPress() || a.autoButton.anyPress();
  if (any) a.display.noteActivity();

  // Any TUNE press while something is running stops it.
  if (a.tuneButton.anyPress() &&
      (a.tuner.running() || a.sweep.isRunning() || a.catTune.active())) {
    abortActivity("ABORTING");
  } else if (a.tuneButton.longPress()) {
    beginTune(true);
  } else if (a.tuneButton.shortPress()) {
    startTuneFromButton();
  }

  if (a.bypassButton.shortPress()) {
    RelayState s = desiredRelayState();
    s.bypass = !s.bypass;
    if (applyRelayState(s)) {
      saveRuntimeState();
      a.status.set(s.bypass ? "BYPASS" : "ACTIVE");
    }
  }
  // Long-press bypass cycles antennas, when the selector is configured.
  if (a.bypassButton.longPress() && c.antennaCount > 1) {
    selectAntenna(static_cast<uint8_t>((c.antenna + 1) % c.antennaCount));
  }

  if (a.autoButton.shortPress()) {
    c.autoTune = !c.autoTune;
    a.settingsStore.markDirty();
    a.status.set(c.autoTune ? "AUTO ON" : "AUTO OFF");
  }
  if (a.autoButton.longPress()) {
    clearProtection();
    a.status.set("PROT RESET");
  }
}

void handleCat() {
  App& a = gApp;
  Settings& c = a.cfg();
  if (!c.catEnabled) return;

  a.cat.loop();

  uint32_t hz = 0;
  if (!a.cat.takeUpdate(hz)) return;
  if (hz == a.currentFreqHz) return;

  a.currentFreqHz = hz;
  a.memTriedBin = UINT32_MAX;
  a.display.noteActivity();

  // Never re-arrange the network from under a running tune or sweep.
  if (a.tuner.running() || a.sweep.isRunning()) return;

  MemLookup m = a.memory.lookup(hz, c.antenna);
  if (m.hit != MemHit::Miss) {
    // A QSY while keyed must not hot-switch: applyRelayState holds the change
    // until RF drops below pwrmax.
    applyRelayState(m.state);
    a.memory.noteUse(m.bin, c.antenna);
    a.memTriedBin = m.bin;
    if (!a.relayPending) a.status.set(memHitName(m.hit));
  } else {
    a.status.set("MEM MISS");
  }
}

void updateSwrAlarm(const SensorReading& r) {
  App& a = gApp;
  Settings& c = a.cfg();
  bool high = r.valid && r.powerW >= c.minTunePowerW && r.swr >= c.maxSWR;
  if (high) {
    a.swrLowSinceMs = 0;
    if (a.swrHighSinceMs == 0) a.swrHighSinceMs = millis() | 1;
    if (!a.swrAlarm && elapsed(a.swrHighSinceMs) >= kSwrAlarmOnMs) {
      a.swrAlarm = true;
      Serial.printf("High SWR alarm: %.1f at %.0f W\n", r.swr, r.powerW);
    }
  } else {
    a.swrHighSinceMs = 0;
    if (a.swrLowSinceMs == 0) a.swrLowSinceMs = millis() | 1;
    if (a.swrAlarm && elapsed(a.swrLowSinceMs) >= kSwrAlarmOffMs) a.swrAlarm = false;
  }
}

void periodicSensor() {
  App& a = gApp;
  Settings& c = a.cfg();

  // The sensor task latches TX inhibit itself; this just runs the rest of the
  // protection sequence, and is checked every loop, not at the display rate.
  if (a.sensor.takeTrip() && a.protectState == ProtectState::Clear) {
    engageProtection("POWER OVERLOAD");
  }

  if (elapsed(a.lastSensorMs) < c.sensorUpdateMs) return;
  a.lastSensorMs = millis();

  SensorReading live = a.sensor.latest();
  // While tuning, show the tuner's own settled measurements rather than
  // readings taken mid relay transition.
  a.reading = a.tuner.running() ? a.tuner.lastReading() : live;

  if (live.valid && live.powerW >= c.minTunePowerW) a.display.noteActivity();

  a.powerProt.update(live.powerW);
  if (a.powerProt.takeTrigger() && a.protectState == ProtectState::Clear) {
    engageProtection("POWER OVERLOAD");
    return;
  }

  updateSwrAlarm(live);

  a.thermal.loop();
  if (a.thermal.demandsBypass() && a.protectState == ProtectState::Clear) {
    engageProtection("OVER TEMPERATURE");
    return;
  }
  if (a.thermal.inhibitsTx()) {
    if (a.tuner.running()) a.tuner.emergencyStop(TuneResult::OverTemp);
    if (a.catTune.active()) a.catTune.emergencyUnkey();
  }
}

void serviceProtection() {
  App& a = gApp;
  if (a.protectState != ProtectState::WaitRfDrop) return;

  if (rfSafeToSwitch()) {
    RelayState s = a.state;
    s.bypass = true;
    applyRelayStateNow(s);
    a.protectState = ProtectState::Bypassed;
    Serial.printf("Protection: RF below %.0f W, bypass engaged\n", a.cfg().maxTunePowerW);
    return;
  }
  if (!a.protectHoldAnnounced && elapsed(a.protectSinceMs) >= a.cfg().protectWaitMs) {
    a.protectHoldAnnounced = true;
    a.status.set("PROTECT - RF ON");
    Serial.printf("Protection: RF still present after %lu ms - holding the current match "
                  "to avoid hot-switching; bypass engages when RF drops\n",
                  static_cast<unsigned long>(a.cfg().protectWaitMs));
  }
}

void serviceDeferredRelays() {
  App& a = gApp;
  if (!a.relayPending || a.tuner.running() || a.protectState != ProtectState::Clear) return;
  if (!rfSafeToSwitch()) return;
  applyRelayStateNow(a.pendingState);
  saveRuntimeState();
  a.status.set("RELAYS UPDATED");
  Serial.println("Held relay change applied");
}

void serviceTxRelease() {
  App& a = gApp;
  if (!a.txReleasePending || elapsed(a.txReleaseAtMs) < a.cfg().txRequestTrailMs) return;
  a.txReleasePending = false;
  a.tx.request(false);   // a protection latch keeps the line asserted
}

void handleTuner() {
  App& a = gApp;
  Settings& c = a.cfg();

  a.tuner.loop();

  TuneResult r;
  RelayState st;
  float swr;
  if (!a.tuner.takeResult(r, st, swr)) return;

  // Release TX request after the configured trail time without blocking.
  a.txReleasePending = true;
  a.txReleaseAtMs = millis();
  a.led.off();
  a.lastTuneMs = millis();

  if (r == TuneResult::PowerOverload) {
    // The relays were left on whatever the search was probing; record that
    // before protection takes over.
    a.state = a.relays.current();
    saveRuntimeState();
    if (a.protectState == ProtectState::Clear) engageProtection("POWER OVERLOAD");
    return;
  }
  if (r == TuneResult::OverTemp) {
    a.state = a.relays.current();
    // Thermal foldback without protection: put back the setting the tune
    // started from rather than leaving a random probe in circuit.
    if (a.protectState == ProtectState::Clear) applyRelayState(a.tuner.entryState());
    saveRuntimeState();
    a.status.set("OVER TEMP");
    return;
  }

  // Emergency stops leave the relays in place; everything else goes through
  // the RF-safe path.
  applyRelayState(st);

  if ((r == TuneResult::Success || r == TuneResult::GoodEnough) && a.currentFreqHz > 0) {
    a.memory.upsert(a.currentFreqHz, st, swr, c.antenna);
    a.memTriedBin = a.memory.binFor(a.currentFreqHz);
  }
  saveRuntimeState();

  if (!a.relayPending) a.status.set(tuneResultName(r));

  Serial.printf("%s  SWR %.2f  L=0x%02X C=0x%02X %s  (%u steps, %lu ms",
                tuneResultName(r), swr, st.lMask, st.cMask, st.topology ? "Hi-Z" : "Lo-Z",
                a.tuner.lastSteps(), static_cast<unsigned long>(a.tuner.lastDurationMs()));
  if (a.currentFreqHz == 0 && a.tuner.lastModelFrequency()) {
    Serial.printf(", frequency estimated ~%.1f MHz", a.tuner.lastModelFrequency() / 1e6);
  }
  Serial.println(")");
}

void handleSweep() {
  App& a = gApp;
  a.sweep.loop();

  if (a.sweep.isRunning()) {
    a.status.printf("SWEEP %u%%", a.sweep.percent());
    return;
  }

  if (!a.sweep.takeComplete()) return;

  const std::vector<SweepPoint>& pts = a.sweep.points();
  Serial.println("Sweep complete.");
  Serial.println("Freq(MHz)\tSWR\tPower(W)");
  for (size_t i = 0; i < pts.size(); ++i) {
    Serial.printf("%.3f\t%.2f\t%.1f\n", pts[i].freqHz / 1e6, pts[i].swr, pts[i].powerW);
  }

  uint32_t minHz;
  float minSwr;
  if (a.sweep.findMinSwr(minHz, minSwr)) {
    Serial.printf("Minimum SWR %.2f at %.3f MHz\n", minSwr, minHz / 1e6);
    a.status.printf("MIN %.2f", minSwr);
  } else {
    a.status.set("SWEEP - NO RF");
  }
  if (a.sweep.restoreFrequency()) {
    Serial.printf("VFO restored to %.3f MHz\n", a.sweep.restoreFrequency() / 1e6);
  }

  // Hold the graph on screen without blocking.
  if (!pts.empty()) {
    a.display->sweepGraph(pts);
    a.showingSweep = true;
    a.sweepResultShownMs = millis();
    a.display.noteActivity();
  }
}

void maybeAutoTune() {
  App& a = gApp;
  Settings& c = a.cfg();

  if (!c.autoTune || a.tuner.running() || a.sweep.isRunning() || a.catTune.active()) return;
  if (a.state.bypass || a.protectionLatched || a.powerProt.overload()) return;
  if (a.relayPending) return;
  if (a.thermal.inhibitsTx()) return;
  if (elapsed(a.lastTuneMs) < c.retuneHoldoffMs) return;
  if (!a.reading.valid || a.reading.powerW < c.minAutoRetunePowerW) return;

  if (a.reading.swr < c.autoRetuneSWR) { a.lastSwrHighMs = 0; return; }

  // Require the SWR to stay high briefly, so a transient does not trigger a
  // tune mid-transmission.
  if (a.lastSwrHighMs == 0) { a.lastSwrHighMs = millis() | 1; return; }
  if (elapsed(a.lastSwrHighMs) < 500) return;
  a.lastSwrHighMs = 0;

  // Above pwrmax neither a memory recall nor a tune may move the relays, and a
  // tune would only assert TX request and then refuse. Say so instead.
  if (a.reading.powerW > c.maxTunePowerW) {
    a.status.printf("RETUNE: <%.0fW", c.maxTunePowerW);
    a.lastTuneMs = millis();
    return;
  }

  // Try memory first, but only once per frequency, then escalate to a tune.
  if (a.currentFreqHz > 0) {
    uint32_t bin = a.memory.binFor(a.currentFreqHz);
    if (a.memTriedBin != bin) {
      MemLookup m = a.memory.lookup(a.currentFreqHz, c.antenna);
      if (m.hit != MemHit::Miss) {
        applyRelayState(m.state);
        a.memory.noteUse(m.bin, c.antenna);
        a.memTriedBin = bin;
        a.status.set("MEM APPLY");
        a.lastTuneMs = millis();
        return;
      }
      a.memTriedBin = bin;
    }
  }

  if (a.cat.connected() && !c.autoTuneUnknownCatFrequency && a.currentFreqHz > 0) {
    a.status.set("CAT NO MEM");
    return;
  }

  beginTune(false);
}

void periodicDisplay() {
  App& a = gApp;
  Settings& c = a.cfg();
  if (elapsed(a.lastDisplayMs) < c.displayUpdateMs) return;
  a.lastDisplayMs = millis();

  a.display.loop();
  if (a.display.blanked()) return;

  // Hold the protection warning, then a finished sweep graph, for long enough
  // to read before the live view takes the screen back.
  if (a.showingProtection) {
    if (elapsed(a.protectionShownMs) < 3000) return;
    a.showingProtection = false;
  }
  if (a.showingSweep) {
    if (elapsed(a.sweepResultShownMs) < 6000) return;
    a.showingSweep = false;
  }

  UiModel m;
  m.freqHz = a.currentFreqHz;
  m.bandName = bandName(a.currentFreqHz);
  m.catConnected = a.cat.connected();
  m.autoTune = c.autoTune;
  m.state = a.state;
  m.reading = a.reading;
  m.totalL = a.relays.totalL();
  m.totalC = a.relays.totalC();
  m.status = a.status.c_str();
  m.powerWarning = a.powerProt.warning();
  m.powerOverload = a.powerProt.overload() || a.protectionLatched;
  m.protectHold = a.protectState == ProtectState::WaitRfDrop && a.protectHoldAnnounced;
  m.swrAlarm = a.swrAlarm;
  m.tuning = a.tuner.running();
  m.tunePercent = a.tuner.percent();
  m.tempValid = a.thermal.available();
  m.tempC = a.thermal.tempC();
  m.tempLevel = a.thermal.level();
  m.antenna = c.antenna;
  m.antennaSupported = RelayController::antennaSupported() || c.antennaCount > 1;
  m.wifiUp = gWeb.up();
  m.powerLimitW = c.powerLimitW;

  if (m.tuning) a.display->tuning(m);
  else a.display->render(m);
}

void updateLed() {
  App& a = gApp;
  if (a.protectionLatched || a.powerProt.overload()) a.led.blink(50, 50);
  else if (a.tuner.running() || a.sweep.isRunning() || a.catTune.active()) a.led.blink(100, 100);
  else if (a.state.bypass) a.led.blink(500, 500);
  else if (a.swrAlarm || a.powerProt.warning() || a.thermal.inhibitsTx()) a.led.blink(200, 200);
  else a.led.off();
}

}  // namespace
}  // namespace atu

// -----------------------------------------------------------------------------

using namespace atu;

void setup() {
  Serial.begin(115200);
  delay(100);

  gApp.bootMs = millis();

  Serial.println();
  Serial.println("=======================================");
  Serial.printf("ATU-1000 ESP32-S3 Controller v%s\n", kFirmwareVersion);
  Serial.println("LCD/OLED auto-detect - CAT - protection");
  Serial.println("=======================================");

  Serial.println("Settings...");
  gApp.settingsStore.begin();
  Serial.printf("  %s\n", gApp.settingsStore.migrated()        ? "migrated from v2.0"
                          : gApp.settingsStore.loadedFromNvs() ? "loaded from NVS"
                                                               : "defaults (first boot)");
  Settings& c = gApp.cfg();

  // Restore the relay state before the relay bank is initialised, so a
  // restored tune comes straight back instead of dropping every relay first.
  gApp.statePrefs.begin("atu_state", false);
  loadRuntimeState();
  if (c.bypassOnBoot) gApp.state.bypass = true;

  Serial.println("Relays...");
  gApp.relays.begin(&c, gApp.state);
  gApp.relays.setAntenna(c.antenna);

  gApp.tx.begin(&c);

  Serial.println("Sensor...");
  gApp.sensor.begin(&c, &gApp.tx);

  Serial.println("Memory...");
  gApp.memory.begin(&c);
  Serial.printf("  %u stored tunes\n", static_cast<unsigned>(gApp.memory.size()));

  Serial.println("Display...");
  gApp.display.begin(&c, Serial);

  Serial.println("Thermal...");
  gApp.thermal.begin(&c);
  Serial.printf("  source: %s%s\n", gApp.thermal.sourceName(),
                gApp.thermal.protects() ? "" : " (information only)");

  gApp.led.begin();
  gApp.powerProt.begin(&c);
  gApp.tuner.begin(&c, &gApp.relays, &gApp.sensor);
  gApp.sweep.begin(&c, &gApp.cat, &gApp.sensor);
  gApp.catTune.begin(&c, &gApp.cat, &gApp.tuner, &gApp.sensor, &beginTune, &tuneAllowed,
                     &setStatus);
  gWeb.begin(&c);

  gApp.display->splash(kFirmwareVersion);

  gApp.tuneButton.begin(kPins.tuneButton, &c);
  gApp.bypassButton.begin(kPins.bypassButton, &c);
  gApp.autoButton.begin(kPins.autoButton, &c);

  if (c.catEnabled) {
    gApp.cat.begin(&c);
    Serial.printf("CAT on GPIO%d/%d at %lu baud, protocol %s, poll %lu ms%s\n", kPins.catRx,
                  kPins.catTx, static_cast<unsigned long>(c.catBaud),
                  gApp.cat.protocolName(), static_cast<unsigned long>(c.catPollMs),
                  c.catUsbPassthrough ? ", USB passthrough" : "");
  }

  if (c.wifiEnabled) {
    Serial.println("Wi-Fi...");
    gWeb.start(Serial);   // connects in the background
  }

  delay(400);   // let the splash be readable
  gApp.reading = gApp.sensor.latest();
  gApp.status.set("READY");
  gApp.display.noteActivity();

  Serial.printf("Display: %s   Protection limit: %.0f W\n", gApp.display.kindName(),
                c.powerLimitW);
  Serial.println("Ready. Type 'help' for commands.");
  Serial.print("> ");

  // From here on a hung loop resets the board (5 s task watchdog).
  enableLoopWDT();
}

void loop() {
  trackRfQuiet();
  pollSerial();
  handleButtons();
  handleCat();
  periodicSensor();
  serviceProtection();
  handleTuner();
  gApp.catTune.loop();
  handleSweep();
  maybeAutoTune();
  serviceDeferredRelays();
  serviceTxRelease();

  gApp.relays.loop();
  gApp.memory.loop();
  gApp.settingsStore.loop();
  gWeb.loop();

  periodicDisplay();
  updateLed();
}
