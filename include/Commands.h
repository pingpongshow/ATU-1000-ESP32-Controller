#pragma once
//
// Console command interface.
//
// Every command writes to a Print&, so the exact same parser backs both the
// serial console and the web UI's command box.
//

#include <Arduino.h>
#include <Wire.h>

#include <algorithm>
#include <cmath>

#include "App.h"
#include "Config.h"
#include "Network.h"
#include "Solver.h"
#include "Types.h"

namespace atu {

// Captures command output into a String for the web API.
class StringPrint : public Print {
 public:
  size_t write(uint8_t c) override { s_ += static_cast<char>(c); return 1; }
  size_t write(const uint8_t* buf, size_t size) override {
    for (size_t i = 0; i < size; ++i) s_ += static_cast<char>(buf[i]);
    return size;
  }
  const String& str() const { return s_; }
  void reset() { s_ = ""; }

 private:
  String s_;
};

void handleCommand(const char* line, Print& out);

// -----------------------------------------------------------------------------

inline void printStatus(Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();

  out.printf("Freq    : %.3f MHz %s%s\n", a.currentFreqHz / 1e6,
             bandName(a.currentFreqHz),
             a.cat.connected() ? " (CAT)" : "");
  if (a.reading.valid) {
    out.printf("SWR     : %.2f\n", a.reading.swr);
    out.printf("Power   : %.1f W\n", a.reading.powerW);
  } else {
    out.println("SWR     : -- (no RF)");
    out.println("Power   : 0.0 W");
  }
  out.printf("Network : L=0x%02X (%.2f uH)  C=0x%02X (%u pF)  %s%s\n",
             a.state.lMask, a.relays.totalL(), a.state.cMask,
             static_cast<unsigned>(a.relays.totalC()),
             a.state.topology ? "Hi-Z" : "Lo-Z",
             a.state.bypass ? "  BYPASS" : "");
  if (a.relayPending) {
    out.printf("Pending : L=0x%02X C=0x%02X %s%s - held until RF <= %.0f W\n",
               a.pendingState.lMask, a.pendingState.cMask,
               a.pendingState.topology ? "Hi-Z" : "Lo-Z",
               a.pendingState.bypass ? " BYPASS" : "", c.maxTunePowerW);
  }
  out.printf("Mode    : %s   Status: %s\n", c.autoTune ? "AUTO" : "MANUAL",
             a.status.c_str());
  out.printf("CAT     : %s @ %lu baud, poll %lu ms%s%s%s\n", a.cat.protocolName(),
             static_cast<unsigned long>(c.catBaud),
             static_cast<unsigned long>(c.catPollMs),
             a.cat.enabled() ? "" : " (disabled)",
             a.cat.usbPassthrough() ? (a.cat.pcActive() ? ", USB PC active" : ", USB passthrough") : "",
             c.catTuneEnabled ? ", CAT tune on" : "");
  out.printf("Memory  : %u entries\n", static_cast<unsigned>(a.memory.size()));
  out.printf("Display : %s\n", a.display.kindName());
  if (a.thermal.available()) {
    out.printf("Temp    : %.1f C (%s, %s)\n", a.thermal.tempC(),
               a.thermal.sourceName(), Thermal::levelName(a.thermal.level()));
  }
  if (RelayController::antennaSupported()) {
    out.printf("Antenna : %u of %u\n", a.relays.antenna() + 1, c.antennaCount);
  }
  out.printf("Protect : limit %.0f W, warn %.0f W, %s\n", c.powerLimitW,
             c.powerWarningW, c.powerProtEnabled ? "enabled" : "DISABLED");
  if (a.powerProt.overload() || a.protectionLatched) {
    out.printf("*** PROTECTION ACTIVE: %s (%s) ***\n", a.protectionReason,
               a.protectState == ProtectState::WaitRfDrop ? "TX inhibit, waiting for RF to drop"
                                                          : "TX inhibit, bypass engaged");
  }
  if (a.swrAlarm) out.printf("*** HIGH SWR ALARM (above %.1f) ***\n", c.maxSWR);
  out.printf("Relays  : %lu total operations\n",
             static_cast<unsigned long>(a.relays.totalCycles()));
  out.printf("Uptime  : %lu s   Heap: %lu\n",
             static_cast<unsigned long>(elapsed(a.bootMs) / 1000UL),
             static_cast<unsigned long>(ESP.getFreeHeap()));
}

inline void printHelp(Print& out) {
  out.println("ATU-1000 commands");
  out.println("  status | help | reboot");
  out.println("Tuning");
  out.println("  tune              start a tune (needs RF)");
  out.println("  tune force        tune without the minimum-power check");
  out.println("  tune cat          key the radio over CAT, tune, restore");
  out.println("  abort             stop a tune, CAT tune or sweep");
  out.println("  bypass on|off     bypass toggle");
  out.println("  auto on|off       auto-tune toggle");
  out.println("  freq <hz>         set the working frequency");
  out.println("Relays");
  out.println("  l <hex> | c <hex> set the L / C relay mask");
  out.println("  topo hi|lo        set the topology relay");
  out.println("  ant <1-8>         select antenna");
  out.println("  cycles [reset]    relay operation counters");
  out.println("Memory");
  out.println("  mem size|clear|list");
  out.println("  mem export        dump as CSV");
  out.println("  mem import        read CSV until a line containing 'end'");
  out.println("  mem del <hz>      delete one entry");
  out.println("Diagnostics");
  out.println("  raw | cal | i2cscan | temp");
  out.println("  cal zero          zero both detector offsets (no RF)");
  out.println("  cal fwd <watts>   solve powerScale at a known power");
  out.println("  cal rev <swr>     solve revScale into a known mismatch");
  out.println("  cal rev           check bridge balance into a 50 ohm load");
  out.println("  settle test       measure relay settle time (needs RF)");
  out.println("  solve <ohms>      show the ideal L/C for a load at this freq");
  out.println("Protection");
  out.println("  power reset|on|off");
  out.println("Sweep");
  out.println("  sweep <startMHz> <endMHz> [stepkHz] [dwellMs]");
  out.println("  sweep stop");
  out.println("CAT");
  out.println("  cat auto|kenwood|icom|yaesu|yaesun|flex|off");
  out.println("  (see catpoll, catusb, civaddr, cattune, catpwr, catmode)");
  out.println("Configuration");
  out.println("  config            list every setting");
  out.println("  get <key>");
  out.println("  set <key> <value>");
  out.println("  config save|reset");
  out.println("Network");
  out.println("  wifi status|on|off");
  out.println("  wifi <ssid> <passphrase>");
}

inline void printConfig(Print& out, const char* filter) {
  App& a = gApp;
  char val[72];
  for (size_t i = 0; i < kSettingDefCount; ++i) {
    const SettingDef& d = kSettingDefs[i];
    if (filter && *filter && !strcasestr(d.key, filter)) continue;
    a.settingsStore.format(d, val, sizeof(val));
    out.printf("  %-12s = %-16s  %s\n", d.key, val, d.help);
  }
}

// -----------------------------------------------------------------------------

inline bool cmdMem(const char* args, Print& out);
inline bool cmdCal(const char* args, Print& out);
inline bool cmdSettleTest(Print& out);
inline bool measurementBusy(Print& out);
inline bool cmdSweep(const char* args, Print& out);
inline bool cmdWifi(const char* args, Print& out);

// CSV import mode: while active, whole lines are fed to the memory store.
inline bool& memImportActive() {
  static bool active = false;
  return active;
}

inline void handleCommand(const char* line, Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();

  if (!line) return;
  while (*line == ' ' || *line == '\t') ++line;
  if (!*line) return;

  // ---- CSV import mode ----
  if (memImportActive()) {
    if (strcasecmp(line, "end") == 0 || strcasecmp(line, ".") == 0) {
      memImportActive() = false;
      a.memory.flush();
      out.printf("Import finished, %u entries stored\n",
                 static_cast<unsigned>(a.memory.size()));
      return;
    }
    if (!a.memory.importCsvLine(line)) out.printf("Skipped: %s\n", line);
    return;
  }

  if (!strcasecmp(line, "help") || !strcmp(line, "?")) { printHelp(out); return; }
  if (!strcasecmp(line, "status")) { printStatus(out); return; }

  if (!strcasecmp(line, "reboot") || !strcasecmp(line, "restart")) {
    out.println("Rebooting...");
    out.flush();
    a.memory.flush();
    a.settingsStore.save();
    a.relays.saveCycles();
    delay(200);
    ESP.restart();
    return;
  }

  // ---- Tuning ----
  if (!strcasecmp(line, "tune")) {
    if (!beginTune(false)) out.println("Busy or inhibited");
    else out.println("Tuning...");
    return;
  }
  if (!strcasecmp(line, "tune force")) {
    if (!beginTune(true)) out.printf("Not started: %s\n", a.status.c_str());
    else out.println("Force tuning...");
    return;
  }
  if (!strcasecmp(line, "tune cat")) {
    if (a.catTune.start()) {
      out.printf("CAT tune: keying the radio at %u W %s, limit %lu ms\n", c.catTunePowerW,
                 c.catTuneMode == 1 ? "AM" : c.catTuneMode == 2 ? "CW" : "FM",
                 static_cast<unsigned long>(c.catTuneMaxMs));
    } else {
      out.printf("Not started: %s\n", a.status.c_str());
    }
    return;
  }
  if (!strcasecmp(line, "abort") || !strcasecmp(line, "stop")) {
    bool did = a.tuner.running() || a.sweep.isRunning() || a.catTune.active();
    abortActivity(did ? "ABORTED" : nullptr);
    out.println(did ? "Aborting" : "Nothing running");
    return;
  }

  if (!strncasecmp(line, "bypass ", 7)) {
    const char* v = line + 7;
    RelayState s = desiredRelayState();
    if (!strcasecmp(v, "on")) s.bypass = true;
    else if (!strcasecmp(v, "off")) s.bypass = false;
    else { out.println("Usage: bypass on|off"); return; }
    if (applyRelayState(s)) {
      a.status.set(s.bypass ? "BYPASS" : "ACTIVE");
      saveRuntimeState();
    } else {
      out.printf("RF is above %.0f W; the change will apply when it drops.\n", c.maxTunePowerW);
    }
    printStatus(out);
    return;
  }

  if (!strncasecmp(line, "auto ", 5)) {
    const char* v = line + 5;
    if (!strcasecmp(v, "on")) c.autoTune = true;
    else if (!strcasecmp(v, "off")) c.autoTune = false;
    else { out.println("Usage: auto on|off"); return; }
    a.status.set(c.autoTune ? "AUTO ON" : "AUTO OFF");
    a.settingsStore.markDirty();
    out.printf("Auto-tune %s\n", c.autoTune ? "enabled" : "disabled");
    return;
  }

  if (!strncasecmp(line, "freq ", 5)) {
    uint32_t hz = strtoul(line + 5, nullptr, 10);
    if (hz < 1000000UL || hz > 60000000UL) {
      out.println("Frequency must be 1-60 MHz, in Hz");
      return;
    }
    a.currentFreqHz = hz;
    a.memTriedBin = UINT32_MAX;
    MemLookup m = a.memory.lookup(hz, c.antenna);
    if (m.hit != MemHit::Miss) {
      if (applyRelayState(m.state)) a.status.set(memHitName(m.hit));
      a.memory.noteUse(m.bin, c.antenna);
    } else {
      a.status.set("MEM MISS");
    }
    printStatus(out);
    return;
  }

  // ---- Relays ----
  if (!strncasecmp(line, "l ", 2) || !strncasecmp(line, "c ", 2)) {
    bool isL = (line[0] == 'l' || line[0] == 'L');
    char* end = nullptr;
    unsigned long v = strtoul(line + 2, &end, 16);
    if (end == line + 2) { out.println("Usage: l|c <hex mask 00-7F>"); return; }
    RelayState s = desiredRelayState();
    if (isL) s.lMask = static_cast<uint8_t>(v) & 0x7F;
    else s.cMask = static_cast<uint8_t>(v) & 0x7F;
    if (!applyRelayState(s)) {
      out.printf("RF is above %.0f W; the change will apply when it drops.\n", c.maxTunePowerW);
      return;
    }
    saveRuntimeState();
    out.printf("L=0x%02X (%.2f uH)  C=0x%02X (%u pF)\n", a.state.lMask,
               a.relays.totalL(), a.state.cMask,
               static_cast<unsigned>(a.relays.totalC()));
    return;
  }

  if (!strncasecmp(line, "topo ", 5)) {
    const char* v = line + 5;
    RelayState s = desiredRelayState();
    if (!strcasecmp(v, "hi")) s.topology = true;
    else if (!strcasecmp(v, "lo")) s.topology = false;
    else { out.println("Usage: topo hi|lo"); return; }
    if (!applyRelayState(s)) {
      out.printf("RF is above %.0f W; the change will apply when it drops.\n", c.maxTunePowerW);
      return;
    }
    saveRuntimeState();
    out.printf("Topology: %s\n", a.state.topology ? "Hi-Z" : "Lo-Z");
    return;
  }

  if (!strncasecmp(line, "ant ", 4)) {
    unsigned n = strtoul(line + 4, nullptr, 10);
    if (n < 1 || n > c.antennaCount) {
      out.printf("Antenna must be 1..%u\n", c.antennaCount);
      return;
    }
    selectAntenna(static_cast<uint8_t>(n - 1));
    out.printf("Antenna %u selected%s\n", n,
               RelayController::antennaSupported() ? "" : " (no output pins configured)");
    return;
  }

  if (!strncasecmp(line, "cycles", 6)) {
    if (strcasestr(line, "reset")) {
      a.relays.resetCycles();
      out.println("Relay cycle counters cleared");
      return;
    }
    out.println("Relay operation counts:");
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      out.printf("  L%u (%.2fuH) %lu\n", i, kLValuesUh[i],
                 static_cast<unsigned long>(a.relays.cycles(i)));
    }
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      out.printf("  C%u (%upF)  %lu\n", i, static_cast<unsigned>(kCValuesPf[i]),
                 static_cast<unsigned long>(a.relays.cycles(kRelayCount + i)));
    }
    out.printf("  Topology   %lu\n",
               static_cast<unsigned long>(a.relays.cycles(kRelayCount * 2)));
    out.printf("  TOTAL      %lu\n",
               static_cast<unsigned long>(a.relays.totalCycles()));
    return;
  }

  // ---- Memory ----
  if (!strncasecmp(line, "mem", 3)) { cmdMem(line + 3, out); return; }

  // ---- Diagnostics ----
  if (!strcasecmp(line, "raw")) {
    if (measurementBusy(out)) return;
    SensorReading r = a.sensor.measureBlocking(32);
    out.printf("FWD: %6.0f raw  %7.1f mV\n", r.fwdRaw, r.fwdMv);
    out.printf("REV: %6.0f raw  %7.1f mV\n", r.revRaw, r.revMv);
    if (r.valid) {
      out.printf("SWR: %.2f   |Gamma| %.3f +/- %.3f   Power: %.1f W   (%u pairs)\n", r.swr,
                 r.gamma, r.gammaNoise, r.powerW, r.pairs);
    } else {
      out.println("SWR: --     Power: 0.0 W  (below swrminfwd)");
    }
    return;
  }

  if (!strcasecmp(line, "settle test") || !strcasecmp(line, "settletest")) {
    cmdSettleTest(out);
    return;
  }

  if (!strncasecmp(line, "cal", 3)) { cmdCal(line + 3, out); return; }

  if (!strcasecmp(line, "temp")) {
    if (!a.thermal.available()) {
      out.printf("No temperature source (%s)\n", a.thermal.sourceName());
      return;
    }
    out.printf("Temperature: %.1f C via %s", a.thermal.tempC(), a.thermal.sourceName());
    if (a.thermal.address()) out.printf(" @ 0x%02X", a.thermal.address());
    out.printf("\nLevel: %s (warn %.0f, foldback %.0f, limit %.0f)\n",
               Thermal::levelName(a.thermal.level()), c.tempWarnC, c.tempFoldbackC,
               c.tempLimitC);
    return;
  }

  if (!strncasecmp(line, "solve ", 6)) {
    float r = atof(line + 6);
    if (r <= 0 || a.currentFreqHz == 0) {
      out.println("Usage: solve <ohms>   (needs a frequency set)");
      return;
    }
    float lUh = 0, cPf = 0;
    if (!designLNetwork(a.currentFreqHz, r, lUh, cPf)) {
      out.println("No L-network solution");
      return;
    }
    uint8_t lm = nearestLMask(clampf(lUh, 0, kLTotalUh));
    uint8_t cm = nearestCMask(clampf(cPf, 0, kCTotalPf));
    out.printf("At %.3f MHz into %.0f ohms:\n", a.currentFreqHz / 1e6, r);
    out.printf("  ideal    L=%.3f uH  C=%.1f pF\n", lUh, cPf);
    out.printf("  nearest  L=0x%02X (%.2f uH)  C=0x%02X (%u pF)\n",
               lm, maskToUh(lm), cm, static_cast<unsigned>(maskToPf(cm)));
    return;
  }

  if (!strcasecmp(line, "i2cscan")) {
    out.println("Scanning I2C bus...");
    out.printf("SDA=GPIO%d SCL=GPIO%d\n", kPins.i2cSda, kPins.i2cScl);
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
      feedLoopWDT();   // a stuck bus can make each probe time out
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        const char* guess = "";
        if (addr == 0x3C || addr == 0x3D) guess = " (SSD1306/SSD1309 OLED)";
        else if (addr >= 0x20 && addr <= 0x27) guess = " (PCF8574 LCD backpack)";
        else if (addr >= 0x38 && addr <= 0x3F) guess = " (PCF8574A LCD backpack)";
        else if (addr >= 0x48 && addr <= 0x4F) guess = " (LM75/TMP102 temperature)";
        out.printf("  0x%02X%s\n", addr, guess);
        ++found;
      }
      yield();
    }
    if (found == 0) {
      out.println("  Nothing responded.");
      out.println("  Check wiring and 4.7k pull-ups on SDA/SCL.");
    } else {
      out.printf("  %u device(s)\n", found);
    }
    return;
  }

  // ---- Protection ----
  if (!strcasecmp(line, "power reset")) {
    clearProtection();
    out.println("Protection reset, bypass released");
    return;
  }
  if (!strcasecmp(line, "power off")) {
    c.powerProtEnabled = false;
    a.settingsStore.markDirty();
    clearProtection();
    out.println("Power protection DISABLED (testing only)");
    return;
  }
  if (!strcasecmp(line, "power on")) {
    c.powerProtEnabled = true;
    a.settingsStore.markDirty();
    out.println("Power protection enabled");
    return;
  }

  // ---- Sweep ----
  if (!strncasecmp(line, "sweep", 5)) { cmdSweep(line + 5, out); return; }

  // ---- CAT ----
  if (!strncasecmp(line, "cat ", 4)) {
    CatProtocol p;
    if (!parseCatProtocol(line + 4, p)) {
      out.println("Usage: cat auto|kenwood|icom|yaesu|yaesun|off");
      return;
    }
    if (p == CatProtocol::None) {
      c.catEnabled = false;
      a.cat.end();
      out.println("CAT disabled");
    } else {
      c.catEnabled = true;
      if (!a.cat.enabled()) a.cat.begin(&c);
      a.cat.setProtocol(p);
      out.printf("CAT protocol: %s", catProtocolName(p));
      if (c.catPollMs) out.printf(", polling every %lu ms", static_cast<unsigned long>(c.catPollMs));
      out.println();
    }
    a.settingsStore.markDirty();
    return;
  }

  // ---- Settings ----
  if (!strcasecmp(line, "config")) {
    out.println("Settings (change with 'set <key> <value>'):");
    printConfig(out, nullptr);
    return;
  }
  if (!strcasecmp(line, "config save")) {
    a.settingsStore.save();
    out.println("Settings saved");
    return;
  }
  if (!strcasecmp(line, "config reset")) {
    a.settingsStore.resetDefaults();
    out.println("Settings restored to defaults. Reboot to re-detect hardware.");
    return;
  }
  if (!strncasecmp(line, "config ", 7)) {
    printConfig(out, line + 7);
    return;
  }

  if (!strncasecmp(line, "get ", 4)) {
    const SettingDef* d = SettingsStore::find(line + 4);
    if (!d) { out.printf("Unknown key '%s'\n", line + 4); return; }
    char val[72];
    a.settingsStore.format(*d, val, sizeof(val));
    out.printf("%s = %s   (%s)\n", d->key, val, d->help);
    return;
  }

  if (!strncasecmp(line, "set ", 4)) {
    char key[24];
    const char* p = line + 4;
    size_t n = 0;
    while (*p && *p != ' ' && n < sizeof(key) - 1) key[n++] = *p++;
    key[n] = '\0';
    while (*p == ' ') ++p;
    if (!*p) { out.println("Usage: set <key> <value>"); return; }

    const SettingDef* d = SettingsStore::find(key);
    if (!d) { out.printf("Unknown key '%s'. Try 'config'.\n", key); return; }

    char err[64] = "";
    if (!a.settingsStore.set(*d, p, err, sizeof(err))) {
      out.printf("Cannot set %s: %s\n", d->key, err);
      return;
    }
    char val[72];
    a.settingsStore.format(*d, val, sizeof(val));
    out.printf("%s = %s\n", d->key, val);

    // A few keys need something re-initialised to take effect now.
    if (!strcasecmp(d->key, "catbaud") || !strcasecmp(d->key, "caten") ||
        !strcasecmp(d->key, "civaddr")) {
      a.cat.restart();
    } else if (!strcasecmp(d->key, "catusb")) {
      out.println(c.catUsbPassthrough ? "Reboot to start the USB CAT port."
                                      : "USB passthrough stops now; reboot to release the port.");
    }
    if (!strcasecmp(d->key, "catproto")) a.cat.setProtocol(static_cast<CatProtocol>(c.catProtocol));
    else if (!strcasecmp(d->key, "ant")) selectAntenna(c.antenna);
    else if (!strcasecmp(d->key, "disptype") || !strcasecmp(d->key, "lcdaddr") ||
             !strcasecmp(d->key, "lcdcols") || !strcasecmp(d->key, "lcdrows") ||
             !strcasecmp(d->key, "oledaddr") || !strcasecmp(d->key, "oleddrv")) {
      out.println("Reboot for the display change to take effect.");
    } else if (!strncasecmp(d->key, "wifi", 4)) {
      out.println("Run 'wifi on' (or reboot) to apply.");
    }
    return;
  }

  // ---- Wi-Fi ----
  if (!strncasecmp(line, "wifi", 4)) { cmdWifi(line + 4, out); return; }

  out.printf("Unknown command '%s'. Type 'help'.\n", line);
}

// -----------------------------------------------------------------------------

inline bool cmdMem(const char* args, Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();
  while (*args == ' ') ++args;

  if (!*args || !strcasecmp(args, "size")) {
    out.printf("Memory: %u / %u entries\n", static_cast<unsigned>(a.memory.size()),
               c.maxMemoryEntries);
    return true;
  }
  if (!strcasecmp(args, "clear")) {
    a.memory.clear();
    a.memTriedBin = UINT32_MAX;
    out.println("Memory cleared");
    return true;
  }
  if (!strcasecmp(args, "list")) {
    const std::vector<MemoryEntry>& e = a.memory.entries();
    if (e.empty()) { out.println("Memory is empty"); return true; }
    out.println("  Freq(MHz)  Band  L    C    Topo  Ant  SWR   Uses");
    for (size_t i = 0; i < e.size(); ++i) {
      uint32_t hz = a.memory.hzForBin(e[i].bin);
      out.printf("  %9.3f  %-4s  0x%02X 0x%02X %-4s  %u    %.2f  %u\n",
                 hz / 1e6, bandName(hz), e[i].lMask, e[i].cMask,
                 (e[i].flags & FLAG_TOPOLOGY) ? "Hi-Z" : "Lo-Z",
                 static_cast<unsigned>(e[i].antenna + 1),
                 e[i].swrX100 / 100.0f, static_cast<unsigned>(e[i].useCount));
    }
    return true;
  }
  if (!strcasecmp(args, "export")) {
    a.memory.exportCsv(out);
    return true;
  }
  if (!strcasecmp(args, "import")) {
    memImportActive() = true;
    out.println("Paste CSV rows, then a line containing 'end'.");
    out.println("Format: freqHz,lMask,cMask,topology,bypass,antenna,swr,useCount");
    return true;
  }
  if (!strncasecmp(args, "del ", 4)) {
    uint32_t hz = strtoul(args + 4, nullptr, 10);
    out.println(a.memory.remove(hz, c.antenna) ? "Deleted" : "No entry for that frequency");
    return true;
  }
  out.println("Usage: mem size|clear|list|export|import|del <hz>");
  return true;
}

// The tuner and the sweep own the sensor's measurement requests while they run.
inline bool measurementBusy(Print& out) {
  if (gApp.tuner.running() || gApp.sweep.isRunning()) {
    out.println("A tune or sweep is using the sensor - 'abort' first.");
    return true;
  }
  return false;
}

inline bool cmdCal(const char* args, Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();
  while (*args == ' ') ++args;

  if (*args && measurementBusy(out)) return true;

  if (!*args) {
    out.printf("FWD offset %.1f mV, scale %.4f\n", c.fwdOffsetMv, c.fwdScale);
    out.printf("REV offset %.1f mV, scale %.4f\n", c.revOffsetMv, c.revScale);
    out.printf("Power scale %.6f   (P = (Vfwd/1000)^2 / powerScale)\n", c.powerScale);
    out.printf("SWR valid above %.0f mV forward\n", c.swrMinForward);
    out.printf("PIC reference constants: FWD=%u REV=%u\n", kPicCalForward, kPicCalReverse);
    out.println("Wizard: 'cal zero' (no RF), 'cal fwd <W>' (dummy load),");
    out.println("        'cal rev <swr>' (known mismatch, e.g. 100 ohm = 2.0)");
    return true;
  }

  // The detectors' DC offset is what they read with no RF at all. It is not
  // what the reverse port reads into a matched load: that residual is bridge
  // imbalance and diode leakage, which scale with power, so zeroing it at one
  // power level makes every other power level wrong.
  if (!strcasecmp(args, "zero")) {
    SensorReading r = a.sensor.measureBlocking(64);
    if (r.fwdMv > 150.0f) {
      out.printf("FWD reads %.1f mV - unkey the transmitter first.\n", r.fwdMv);
      return true;
    }
    out.printf("No-RF offsets: FWD %.1f mV (was %.1f), REV %.1f mV (was %.1f)\n", r.fwdMv,
               c.fwdOffsetMv, r.revMv, c.revOffsetMv);
    c.fwdOffsetMv = clampf(r.fwdMv, 0.0f, 2000.0f);
    c.revOffsetMv = clampf(r.revMv, 0.0f, 2000.0f);
    a.settingsStore.save();
    return true;
  }

  if (!strncasecmp(args, "fwd ", 4)) {
    float watts = atof(args + 4);
    if (watts <= 0.0f) { out.println("Usage: cal fwd <watts>"); return true; }
    SensorReading r = a.sensor.measureBlocking(64);
    float scale = 0.0f;
    if (!a.sensor.solvePowerScale(r, watts, scale)) {
      out.printf("Not enough forward voltage (%.1f mV). Key the transmitter first.\n",
                 r.fwdMv);
      return true;
    }
    out.printf("FWD %.1f mV at %.1f W -> powerScale %.6f (was %.6f)\n",
               r.fwdMv, watts, scale, c.powerScale);
    c.powerScale = clampf(scale, 1e-6f, 1000.0f);
    a.settingsStore.save();
    SensorReading check = a.sensor.measureBlocking(32);
    out.printf("Now reading %.1f W, SWR %.2f\n", check.powerW,
               check.valid ? check.swr : 0.0f);
    return true;
  }

  if (!strncasecmp(args, "rev", 3)) {
    const char* v = args + 3;
    while (*v == ' ') ++v;
    SensorReading r = a.sensor.measureBlocking(64);
    float vf = a.sensor.forwardCorrectedMv(r.fwdMv);
    if (vf < c.swrMinForward) {
      out.println("Key a steady carrier first.");
      return true;
    }

    if (!*v) {
      // Check only. Into 50 ohms the reverse port should read (almost) zero;
      // whatever is left is the bridge's directivity limit.
      float g = a.sensor.reverseCorrectedMv(r.revMv) / vf;
      out.printf("Into a 50 ohm load: REV %.1f mV, FWD %.1f mV -> SWR %.2f (|Gamma| %.3f)\n",
                 r.revMv, r.fwdMv, gammaToSwr(g), g);
      out.println(g < 0.05f ? "Bridge balance is good."
                            : "Residual reflection: check the bridge balance trimmer.");
      out.println("To calibrate REV scale, key into a known mismatch: 'cal rev <swr>'.");
      return true;
    }

    float swr = atof(v);
    if (swr < 1.5f || swr > 10.0f) {
      out.println("Use a known mismatch between SWR 1.5 and 10, e.g. 100 ohm = 2.0");
      return true;
    }
    float rawRev = std::max(0.0f, r.revMv - c.revOffsetMv);
    if (rawRev < 5.0f) {
      out.printf("REV reads only %.1f mV above its offset - is the mismatch connected?\n", rawRev);
      return true;
    }
    float wantGamma = swrToGamma(swr);
    float scale = (wantGamma * vf) / rawRev;
    out.printf("SWR %.2f load: FWD %.1f mV, REV %.1f mV -> revScale %.4f (was %.4f)\n", swr,
               r.fwdMv, r.revMv, scale, c.revScale);
    c.revScale = clampf(scale, 0.001f, 1000.0f);
    a.settingsStore.save();
    SensorReading check = a.sensor.measureBlocking(32);
    out.printf("Now reading SWR %.2f\n", check.valid ? check.swr : 0.0f);
    return true;
  }

  out.println("Usage: cal | cal zero | cal fwd <watts> | cal rev [<swr>]");
  return true;
}

// Measures how long a relay change takes to show up fully at the detectors
// (relay operate + bounce + the detector RC filter), in both directions, and
// recommends a `settle` value.
inline bool cmdSettleTest(Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();
  if (a.tuner.running() || a.sweep.isRunning() || a.catTune.active()) {
    out.println("Busy - 'abort' first.");
    return true;
  }
  if (a.protectionLatched) {
    out.println("Protection is latched - 'power reset' first.");
    return true;
  }
  SensorReading r = a.sensor.latest();
  if (!r.valid || r.powerW < c.minTunePowerW || r.powerW > c.maxTunePowerW) {
    out.printf("Key a steady carrier between %.0f and %.0f W first.\n", c.minTunePowerW,
               c.maxTunePowerW);
    return true;
  }

  if (a.relayPending || a.protectState != ProtectState::Clear) {
    out.println("Relay changes are held or protection is active - try again shortly.");
    return true;
  }
  const RelayState home = a.state;
  RelayState other = home;
  other.bypass = false;
  // A large capacitance step changes the reflection clearly on any band.
  uint8_t ci = cLadder().indexOf[other.cMask];
  other.cMask = cLadder().maskAt[ci >= 64 ? ci - 48 : ci + 48];

  static constexpr uint16_t kPairs = 600;
  static TracePoint trace[kPairs];
  uint32_t worstUs[2] = {0, 0};

  for (int dir = 0; dir < 2; ++dir) {
    const RelayState& to = (dir == 0) ? other : home;
    a.sensor.startTrace(trace, kPairs);
    delay(15);                          // pre-switch baseline
    uint32_t switchUs = micros();
    a.relays.apply(to);
    a.state = to;
    uint32_t t0 = millis();
    while (!a.sensor.traceDone() && elapsed(t0) < 1500) delay(2);
    if (!a.sensor.traceDone()) {
      out.println("Trace timed out");
      break;
    }

    // Final value: mean |Gamma| over the last 20% of the trace.
    auto gammaAt = [&](const TracePoint& t) {
      float vf = a.sensor.forwardCorrectedMv(t.fwdMv);
      return vf >= c.swrMinForward ? a.sensor.reverseCorrectedMv(t.revMv) / vf : -1.0f;
    };
    float finalG = 0, startG = 0;
    int nf = 0, ns = 0;
    for (uint16_t i = kPairs * 4 / 5; i < kPairs; ++i) {
      float g = gammaAt(trace[i]);
      if (g >= 0) { finalG += g; ++nf; }
    }
    for (uint16_t i = 0; i < kPairs && trace[i].us < switchUs; ++i) {
      float g = gammaAt(trace[i]);
      if (g >= 0) { startG += g; ++ns; }
    }
    if (nf == 0 || ns == 0) {
      out.println("RF dropped during the test");
      break;
    }
    finalG /= nf;
    startG /= ns;
    float band = std::max(0.02f, 0.1f * std::fabs(finalG - startG));

    uint32_t settledUs = 0;
    for (uint16_t i = 0; i < kPairs; ++i) {
      if (trace[i].us < switchUs) continue;
      float g = gammaAt(trace[i]);
      if (g < 0 || std::fabs(g - finalG) > band) settledUs = trace[i].us - switchUs;
    }
    worstUs[dir] = settledUs;
    float spanMs = (trace[kPairs - 1].us - trace[0].us) / 1000.0f;
    out.printf("%s: |Gamma| %.3f -> %.3f, settled in %.1f ms (trace %.0f ms, %.2f ms/pair)\n",
               dir == 0 ? "Away " : "Back ", startG, finalG, settledUs / 1000.0f, spanMs,
               spanMs / kPairs);
  }

  applyRelayState(home);
  uint32_t worstMs = (std::max(worstUs[0], worstUs[1]) + 999) / 1000;
  uint32_t recommend = worstMs + worstMs / 2 + 2;
  out.printf("Current settle = %lu ms. Recommended: set settle %lu\n",
             static_cast<unsigned long>(c.relaySettleMs), static_cast<unsigned long>(recommend));
  return true;
}

inline bool cmdSweep(const char* args, Print& out) {
  App& a = gApp;
  while (*args == ' ') ++args;

  if (!strcasecmp(args, "stop")) {
    a.sweep.stop();
    out.println("Sweep stopped");
    return true;
  }
  if (!*args) {
    if (a.sweep.isRunning()) out.printf("Sweep running, %u%%\n", a.sweep.percent());
    else out.println("Usage: sweep <startMHz> <endMHz> [stepkHz] [dwellMs] | sweep stop");
    return true;
  }

  if (a.tuner.running() || a.catTune.active()) {
    out.println("A tune is in progress. 'abort' first.");
    return true;
  }
  if (a.protectionLatched || a.thermal.inhibitsTx()) {
    out.println("Inhibited by protection. Clear it with 'power reset'.");
    return true;
  }

  float startMHz = 0, endMHz = 0;
  unsigned long stepKHz = 25, dwellMs = 200;
  int n = sscanf(args, "%f %f %lu %lu", &startMHz, &endMHz, &stepKHz, &dwellMs);
  if (n < 2) {
    out.println("Usage: sweep <startMHz> <endMHz> [stepkHz] [dwellMs]");
    return true;
  }
  if (endMHz <= startMHz || startMHz < 1.0f || endMHz > 60.0f) {
    out.println("Range must be increasing and within 1-60 MHz");
    return true;
  }
  if (stepKHz == 0) stepKHz = 25;

  uint32_t startHz = static_cast<uint32_t>(startMHz * 1e6f);
  uint32_t endHz = static_cast<uint32_t>(endMHz * 1e6f);

  if (!a.sweep.start(startHz, endHz, stepKHz * 1000UL, static_cast<uint16_t>(dwellMs))) {
    out.println("Cannot start: sweep already running, or CAT is not available.");
    return true;
  }
  out.printf("Sweeping %.3f-%.3f MHz, %lu kHz steps, %lu ms dwell\n",
             startMHz, endMHz, stepKHz, dwellMs);
  out.println("Key a steady carrier (AM/FM/RTTY) for the whole sweep.");
  out.println("'sweep stop' or the TUNE button aborts.");
  return true;
}

}  // namespace atu
