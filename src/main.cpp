// =============================================================================
// ATU-1000 ESP32-S3 Controller
//
// Single firmware image for the LCD and OLED hardware variants; the display is
// probed on the I2C bus at boot and the matching driver is instantiated.
//
// Everything in loop() is non-blocking, so buttons, protection, the console and
// the web UI stay responsive during a tune or a sweep.
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
bool gTuneWasRunning = false;

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

void applyRelayState(const RelayState& s) {
  gApp.state = s;
  gApp.relays.apply(gApp.state);
}

void setTxRequest(bool active) {
  bool level = gApp.cfg().requestTxActiveHigh ? active : !active;
  if (kPins.txReq >= 0) digitalWrite(kPins.txReq, level ? HIGH : LOW);
  if (kPins.txReqInv >= 0) digitalWrite(kPins.txReqInv, level ? LOW : HIGH);
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

void engageProtection(const char* reason) {
  strncpy(gApp.protectionReason, reason, sizeof(gApp.protectionReason) - 1);
  gApp.protectionReason[sizeof(gApp.protectionReason) - 1] = '\0';
  gApp.protectionLatched = true;

  // Ask the radio to stop transmitting *before* opening the relays. Switching a
  // relay bank under a kilowatt arcs the contacts; asserting TX inhibit first at
  // least gives a radio that honours it a chance to unkey.
  setTxRequest(true);

  gApp.tuner.emergencyStop(TuneResult::PowerOverload);
  gApp.sweep.stop();

  gApp.state.bypass = true;
  applyRelayState(gApp.state);
  gApp.status.set(reason);
  gApp.display.noteActivity();
  gApp.display->overload(reason);
  // Hold the full-screen warning long enough to actually be read. The old
  // firmware painted it and let the next 200 ms refresh wipe it immediately.
  gApp.showingProtection = true;
  gApp.showingSweep = false;
  gApp.protectionShownMs = millis();
  Serial.printf("*** PROTECTION: %s - bypass engaged ***\n", reason);
}

void clearProtection() {
  gApp.powerProt.reset();
  gApp.protectionLatched = false;
  gApp.protectionReason[0] = '\0';
  gApp.state.bypass = false;
  applyRelayState(gApp.state);
  setTxRequest(false);
  gApp.status.set("READY");
  gApp.showingProtection = false;
  gApp.display.noteActivity();
}

bool beginTune(bool force) {
  App& a = gApp;
  Settings& c = a.cfg();

  if (a.tuner.running() || a.sweep.isRunning()) return false;
  if (a.protectionLatched || a.powerProt.overload()) {
    a.status.set("PROTECT - RESET");
    return false;
  }
  if (a.thermal.inhibitsTx()) {
    a.status.set("TOO HOT");
    return false;
  }
  if (!force && elapsed(a.lastTuneMs) < c.retuneHoldoffMs) return false;

  RelayState hint;
  bool hasHint = false;
  if (a.currentFreqHz > 0) {
    MemLookup m = a.memory.lookup(a.currentFreqHz, c.antenna);
    if (m.hit != MemHit::Miss) { hint = m.state; hasHint = true; }
  }

  a.state.bypass = false;
  a.relays.apply(a.state);

  setTxRequest(true);
  a.led.on();
  a.status.set("TUNING");
  a.display.noteActivity();

  if (!a.tuner.start(a.currentFreqHz, force, hasHint ? &hint : nullptr)) {
    setTxRequest(false);
    return false;
  }
  gTuneWasRunning = true;
  return true;
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
    // Backspace support; the old console was blind typing.
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

  // Any TUNE press while something is running stops it. This is the abort path
  // the original firmware had no way to offer, because a tune held the CPU for
  // its whole duration.
  if (a.tuneButton.anyPress() && (a.tuner.running() || a.sweep.isRunning())) {
    a.tuner.abort();
    a.sweep.stop();
    a.status.set("ABORTING");
  } else if (a.tuneButton.longPress()) {
    beginTune(true);
  } else if (a.tuneButton.shortPress()) {
    beginTune(false);
  }

  if (a.bypassButton.shortPress()) {
    a.state.bypass = !a.state.bypass;
    applyRelayState(a.state);
    saveRuntimeState();
    a.status.set(a.state.bypass ? "BYPASS" : "ACTIVE");
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
    applyRelayState(m.state);
    a.memory.noteUse(m.bin, c.antenna);
    a.memTriedBin = m.bin;
    a.status.set(m.hit == MemHit::Band ? "MEM BAND" : "MEM HIT");
  } else {
    a.status.set("MEM MISS");
  }
}

void periodicSensor() {
  App& a = gApp;
  Settings& c = a.cfg();
  if (elapsed(a.lastSensorMs) < c.sensorUpdateMs) return;
  a.lastSensorMs = millis();

  // While tuning, the engine is already sampling; reuse its reading so the two
  // are never fighting over the ADC.
  a.reading = a.tuner.running() ? a.tuner.lastReading() : a.sensor.readDefault();

  if (a.reading.valid && a.reading.powerW >= c.minTunePowerW) a.display.noteActivity();

  a.powerProt.update(a.reading.powerW);
  if (a.powerProt.takeTrigger()) {
    engageProtection("POWER OVERLOAD");
    return;
  }

  a.thermal.loop();
  if (a.thermal.demandsBypass() && !a.protectionLatched) {
    engageProtection("OVER TEMPERATURE");
    return;
  }
  if (a.thermal.inhibitsTx() && a.tuner.running()) {
    a.tuner.emergencyStop(TuneResult::OverTemp);
  }
}

void handleTuner() {
  App& a = gApp;
  Settings& c = a.cfg();

  a.tuner.loop();

  TuneResult r;
  RelayState st;
  float swr;
  if (!a.tuner.takeResult(r, st, swr)) return;

  // Release TX inhibit after the configured trail time - unless protection is
  // latched, in which case the inhibit was asserted deliberately and must stay
  // asserted until 'power reset'.
  delay(c.txRequestTrailMs);
  if (!a.protectionLatched) setTxRequest(false);
  a.led.off();
  a.lastTuneMs = millis();
  gTuneWasRunning = false;

  a.state = st;

  if (r == TuneResult::PowerOverload) {
    // Only engage if this is news; the sensor task may already have latched.
    if (!a.protectionLatched) engageProtection("POWER OVERLOAD");
    return;
  }
  if (r == TuneResult::OverTemp) {
    a.status.set("OVER TEMP");
    return;
  }

  if ((r == TuneResult::Success || r == TuneResult::GoodEnough) && a.currentFreqHz > 0) {
    a.memory.upsert(a.currentFreqHz, st, swr, c.antenna);
    a.memTriedBin = a.memory.binFor(a.currentFreqHz);
  }
  saveRuntimeState();

  a.status.set(tuneResultName(r));
  a.reading = a.sensor.readDefault();

  Serial.printf("%s  SWR %.2f  L=0x%02X C=0x%02X %s\n", tuneResultName(r), swr,
                st.lMask, st.cMask, st.topology ? "Hi-Z" : "Lo-Z");
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

  // Hold the graph on screen without blocking the whole firmware for 5 s the
  // way the old delay(5000) did.
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

  if (!c.autoTune || a.tuner.running() || a.sweep.isRunning()) return;
  if (a.state.bypass || a.protectionLatched || a.powerProt.overload()) return;
  if (a.thermal.inhibitsTx()) return;
  if (elapsed(a.lastTuneMs) < c.retuneHoldoffMs) return;
  if (!a.reading.valid || a.reading.powerW < c.minAutoRetunePowerW) return;

  if (a.reading.swr < c.autoRetuneSWR) { a.lastSwrHighMs = 0; return; }

  // Require the SWR to stay high briefly, so a transient does not trigger a
  // tune mid-transmission.
  if (a.lastSwrHighMs == 0) { a.lastSwrHighMs = millis() | 1; return; }
  if (elapsed(a.lastSwrHighMs) < 500) return;
  a.lastSwrHighMs = 0;

  // Try memory first, but only once per frequency. The original re-applied the
  // same stored entry every couple of seconds forever when that entry was
  // stale, and never escalated to an actual tune.
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
  else if (a.tuner.running() || a.sweep.isRunning()) a.led.blink(100, 100);
  else if (a.state.bypass) a.led.blink(500, 500);
  else if (a.powerProt.warning() || a.thermal.inhibitsTx()) a.led.blink(200, 200);
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
  Serial.printf("  %s\n", gApp.settingsStore.loadedFromNvs()
                              ? "loaded from NVS"
                              : "defaults (first boot or version change)");
  Settings& c = gApp.cfg();

  gApp.statePrefs.begin("atu_state", false);

  Serial.println("Relays...");
  gApp.relays.begin(&c);

  Serial.println("Sensor...");
  gApp.sensor.begin(&c);

  Serial.println("Memory...");
  gApp.memory.begin(&c);
  Serial.printf("  %u stored tunes\n", static_cast<unsigned>(gApp.memory.size()));

  Serial.println("Display...");
  gApp.display.begin(&c, Serial);

  Serial.println("Thermal...");
  gApp.thermal.begin(&c);
  Serial.printf("  source: %s\n", gApp.thermal.sourceName());

  gApp.led.begin();
  gApp.powerProt.begin(&c);
  gApp.tuner.begin(&c, &gApp.relays, &gApp.sensor);
  gApp.sweep.begin(&c, &gApp.cat, &gApp.sensor);
  gWeb.begin(&c);

  gApp.display->splash(kFirmwareVersion);

  gApp.tuneButton.begin(kPins.tuneButton, &c);
  gApp.bypassButton.begin(kPins.bypassButton, &c);
  gApp.autoButton.begin(kPins.autoButton, &c);

  if (kPins.txReq >= 0) pinMode(kPins.txReq, OUTPUT);
  if (kPins.txReqInv >= 0) pinMode(kPins.txReqInv, OUTPUT);
  setTxRequest(false);

  loadRuntimeState();
  if (c.bypassOnBoot) gApp.state.bypass = true;
  applyRelayState(gApp.state);
  gApp.relays.setAntenna(c.antenna);

  if (c.catEnabled) {
    gApp.cat.begin(&c);
    Serial.printf("CAT on GPIO%d/%d at %lu baud, protocol %s\n", kPins.catRx,
                  kPins.catTx, static_cast<unsigned long>(c.catBaud),
                  gApp.cat.protocolName());
  }

  if (c.wifiEnabled) {
    Serial.println("Wi-Fi...");
    if (gWeb.start(Serial)) Serial.printf("Web UI: http://%s/\n", gWeb.ip().c_str());
  }

  delay(400);   // let the splash be readable
  gApp.reading = gApp.sensor.readAverage(4, 4);
  gApp.status.set("READY");
  gApp.display.noteActivity();

  Serial.printf("Display: %s   Protection limit: %.0f W\n", gApp.display.kindName(),
                c.powerLimitW);
  Serial.println("Ready. Type 'help' for commands.");
  Serial.print("> ");
}

void loop() {
  pollSerial();
  handleButtons();
  handleCat();
  periodicSensor();
  handleTuner();
  handleSweep();
  maybeAutoTune();

  gApp.relays.loop();
  gApp.memory.loop();
  gApp.settingsStore.loop();
  gWeb.loop();

  periodicDisplay();
  updateLed();
}
