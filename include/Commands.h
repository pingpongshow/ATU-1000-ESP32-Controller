#pragma once
//
// Console command interface.
//
// Every command writes to a Print&, so the exact same parser backs both the
// serial console and the web UI's command box.
//

#include <Arduino.h>
#include <Wire.h>

#include "App.h"
#include "Config.h"
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
  out.printf("Mode    : %s   Status: %s\n", c.autoTune ? "AUTO" : "MANUAL",
             a.status.c_str());
  out.printf("CAT     : %s @ %lu baud %s\n", a.cat.protocolName(),
             static_cast<unsigned long>(c.catBaud),
             a.cat.enabled() ? "" : "(disabled)");
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
    out.printf("*** PROTECTION ACTIVE: %s ***\n", a.protectionReason);
  }
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
  out.println("  tune force        tune without the RF power check");
  out.println("  abort             stop a tune or sweep in progress");
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
  out.println("  cal fwd <watts>   solve powerScale at a known power");
  out.println("  cal rev           zero the reverse channel into a matched load");
  out.println("  solve <ohms>      show the ideal L/C for a load at this freq");
  out.println("Protection");
  out.println("  power reset|on|off");
  out.println("Sweep");
  out.println("  sweep <startMHz> <endMHz> [stepkHz] [dwellMs]");
  out.println("  sweep stop");
  out.println("CAT");
  out.println("  cat auto|kenwood|icom|yaesu|yaesun|off");
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
    if (!beginTune(true)) out.println("Busy or inhibited");
    else out.println("Force tuning...");
    return;
  }
  if (!strcasecmp(line, "abort") || !strcasecmp(line, "stop")) {
    bool did = false;
    if (a.tuner.running()) { a.tuner.abort(); did = true; }
    if (a.sweep.isRunning()) { a.sweep.stop(); did = true; }
    out.println(did ? "Aborting" : "Nothing running");
    return;
  }

  if (!strncasecmp(line, "bypass ", 7)) {
    const char* v = line + 7;
    if (!strcasecmp(v, "on")) { a.state.bypass = true; a.status.set("BYPASS"); }
    else if (!strcasecmp(v, "off")) { a.state.bypass = false; a.status.set("ACTIVE"); }
    else { out.println("Usage: bypass on|off"); return; }
    applyRelayState(a.state);
    saveRuntimeState();
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
      applyRelayState(m.state);
      a.memory.noteUse(m.bin, c.antenna);
      a.status.set(m.hit == MemHit::Band ? "MEM BAND" : "MEM HIT");
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
    if (isL) a.state.lMask = static_cast<uint8_t>(v) & 0x7F;
    else a.state.cMask = static_cast<uint8_t>(v) & 0x7F;
    applyRelayState(a.state);
    saveRuntimeState();
    out.printf("L=0x%02X (%.2f uH)  C=0x%02X (%u pF)\n", a.state.lMask,
               a.relays.totalL(), a.state.cMask,
               static_cast<unsigned>(a.relays.totalC()));
    return;
  }

  if (!strncasecmp(line, "topo ", 5)) {
    const char* v = line + 5;
    if (!strcasecmp(v, "hi")) a.state.topology = true;
    else if (!strcasecmp(v, "lo")) a.state.topology = false;
    else { out.println("Usage: topo hi|lo"); return; }
    applyRelayState(a.state);
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
    SensorReading r = a.sensor.readAverage(16, 2);
    out.printf("FWD: %6.0f raw  %7.1f mV\n", r.fwdRaw, r.fwdMv);
    out.printf("REV: %6.0f raw  %7.1f mV\n", r.revRaw, r.revMv);
    if (r.valid) out.printf("SWR: %.2f   Power: %.1f W\n", r.swr, r.powerW);
    else out.println("SWR: --     Power: 0.0 W  (below swrminfwd)");
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
      out.printf("CAT protocol: %s\n", catProtocolName(p));
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
    if (!strcasecmp(d->key, "catbaud") || !strcasecmp(d->key, "caten")) a.cat.restart();
    else if (!strcasecmp(d->key, "catproto")) a.cat.setProtocol(static_cast<CatProtocol>(c.catProtocol));
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

inline bool cmdCal(const char* args, Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();
  while (*args == ' ') ++args;

  if (!*args) {
    out.printf("FWD offset %.1f mV, scale %.4f\n", c.fwdOffsetMv, c.fwdScale);
    out.printf("REV offset %.1f mV, scale %.4f\n", c.revOffsetMv, c.revScale);
    out.printf("Power scale %.6f   (P = (Vfwd/1000)^2 / powerScale)\n", c.powerScale);
    out.printf("SWR valid above %.0f mV forward\n", c.swrMinForward);
    out.printf("PIC reference constants: FWD=%u REV=%u\n", kPicCalForward, kPicCalReverse);
    out.println("Wizard: key a carrier into a dummy load, then 'cal fwd <watts>'");
    return true;
  }

  if (!strncasecmp(args, "fwd ", 4)) {
    float watts = atof(args + 4);
    if (watts <= 0.0f) { out.println("Usage: cal fwd <watts>"); return true; }
    SensorReading r = a.sensor.readAverage(32, 2);
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
    SensorReading check = a.sensor.readAverage(16, 2);
    out.printf("Now reading %.1f W, SWR %.2f\n", check.powerW,
               check.valid ? check.swr : 0.0f);
    return true;
  }

  if (!strcasecmp(args, "rev")) {
    SensorReading r = a.sensor.readAverage(32, 2);
    if (r.fwdMv < c.swrMinForward) {
      out.println("Key a carrier into a 50 ohm dummy load first.");
      return true;
    }
    // Into a matched load the reverse port should read zero, so whatever it
    // does read is the detector's offset.
    float newOffset = clampf(r.revMv, 0.0f, 2000.0f);
    out.printf("REV reads %.1f mV into a matched load -> revoffset %.1f (was %.1f)\n",
               r.revMv, newOffset, c.revOffsetMv);
    c.revOffsetMv = newOffset;
    a.settingsStore.save();
    SensorReading check = a.sensor.readAverage(16, 2);
    out.printf("Now reading SWR %.2f\n", check.valid ? check.swr : 0.0f);
    return true;
  }

  out.println("Usage: cal | cal fwd <watts> | cal rev");
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

  if (a.tuner.running()) {
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
