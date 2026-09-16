#pragma once
//
// Runtime settings, persisted in NVS and editable at runtime.
//
// Everything here used to be a `constexpr` in Config.h, which meant every
// calibration tweak needed a recompile and a reflash. The struct is a POD so
// it can be described by a table of {name, type, offset} and driven generically
// by `set` / `get` / `config` from both the serial console and the web UI.
//

#include <Arduino.h>
#include <Preferences.h>

#include <cstddef>

#include "Config.h"
#include "Types.h"

namespace atu {

constexpr uint8_t kSettingsVersion = 3;

struct Settings {
  uint8_t version;

  // --- Serial / CAT ---
  uint32_t catBaud;
  uint32_t catTimeoutMs;
  uint8_t catProtocol;        // CatProtocol
  bool catEnabled;
  uint32_t catPollMs;         // 0 = never poll, only listen
  bool catUsbPassthrough;     // share the radio's CAT port with a PC over USB
  uint8_t icomAddress;        // Icom radio CI-V address, 0 = learn from traffic
  bool catTuneEnabled;        // TUNE button keys the radio over CAT
  uint8_t catTunePowerW;
  uint8_t catTuneMode;        // CarrierMode
  uint32_t catTuneMaxMs;      // hard key-down limit for a CAT tune

  // --- Timing ---
  uint32_t sensorUpdateMs;
  uint32_t displayUpdateMs;
  uint32_t relaySettleMs;
  uint32_t retuneHoldoffMs;
  uint32_t txRequestLeadMs;
  uint32_t txRequestTrailMs;

  // --- Power thresholds (W) ---
  float minTunePowerW;
  float minAutoRetunePowerW;
  float maxTunePowerW;        // also the most the relays are ever switched under
  float powerLimitW;
  float powerWarningW;
  uint32_t powerOverloadHoldoffMs;
  bool powerProtEnabled;
  uint32_t protectWaitMs;     // how long to wait for RF to drop before holding

  // --- SWR thresholds ---
  float targetSWR;
  float goodSWR;
  float autoRetuneSWR;
  float maxSWR;

  // --- Tuning algorithm ---
  uint8_t measureSamples;     // FWD/REV pairs per quick measurement
  uint8_t preciseSamples;     // pairs per precise measurement near the match
  uint8_t refinePasses;
  uint16_t maxTuneSteps;      // hard ceiling on measurements per tune
  bool tuneModelFit;

  // --- Frequency memory ---
  uint32_t memoryBinHz;
  uint16_t maxMemoryEntries;
  bool memoryBandFallback;
  uint32_t memoryInterpHz;    // 0 = no interpolation

  // --- Display ---
  uint8_t displayType;        // DisplayKind
  uint8_t lcdAddress;
  uint8_t lcdCols;
  uint8_t lcdRows;
  uint8_t oledAddress;
  uint8_t oledDriver;         // 0 = SSD1309 NONAME2, 1 = SSD1306 NONAME, 2 = SSD1309 NONAME0
  uint8_t oledContrast;
  uint16_t dimAfterSec;       // 0 = never dim
  uint16_t blankAfterSec;     // 0 = never blank
  uint16_t burnInShiftSec;    // 0 = no pixel shift

  // --- Hardware ---
  bool relayActiveHigh;
  bool requestTxActiveHigh;
  bool buttonsActiveLow;

  // --- Feature enables ---
  bool autoTune;
  bool autoTuneUnknownCatFrequency;
  bool bypassOnBoot;

  // --- SWR bridge calibration ---
  float fwdOffsetMv;
  float revOffsetMv;
  float fwdScale;
  float revScale;
  float powerScale;
  float swrMinForward;

  // --- Thermal ---
  bool thermalEnabled;
  uint8_t tempAddress;        // 0 = auto-probe LM75/TMP102 family
  float tempWarnC;
  float tempFoldbackC;
  float tempLimitC;
  float ntcBeta;
  float ntcNominalOhms;
  float ntcSeriesOhms;

  // --- Antenna selector ---
  uint8_t antenna;            // currently selected antenna index
  uint8_t antennaCount;

  // --- Wi-Fi / web UI ---
  bool wifiEnabled;
  bool wifiApFallback;
  char wifiSsid[33];
  char wifiPass[65];
  char hostname[25];

  uint32_t checksum;
};

// Layout of the v2.0 settings blob, kept only so an upgrade does not throw
// away a calibrated tuner's configuration.
struct SettingsV2 {
  uint8_t version;

  // --- Serial / CAT ---
  uint32_t catBaud;
  uint32_t catTimeoutMs;
  uint8_t catProtocol;        // CatProtocol
  bool catEnabled;

  // --- Timing ---
  uint32_t sensorUpdateMs;
  uint32_t displayUpdateMs;
  uint32_t relaySettleMs;
  uint32_t retuneHoldoffMs;
  uint32_t txRequestLeadMs;
  uint32_t txRequestTrailMs;

  // --- Power thresholds (W) ---
  float minTunePowerW;
  float minAutoRetunePowerW;
  float maxTunePowerW;
  float powerLimitW;
  float powerWarningW;
  uint32_t powerOverloadHoldoffMs;
  bool powerProtEnabled;

  // --- SWR thresholds ---
  float targetSWR;
  float goodSWR;
  float autoRetuneSWR;
  float maxSWR;

  // --- Tuning algorithm ---
  uint8_t measureSamples;
  uint16_t measureSampleSpacingMs;
  uint8_t refinePasses;
  uint16_t maxTuneSteps;      // hard ceiling on measurements per tune

  // --- Frequency memory ---
  uint32_t memoryBinHz;
  uint16_t maxMemoryEntries;
  bool memoryBandFallback;

  // --- Display ---
  uint8_t displayType;        // DisplayKind
  uint8_t lcdAddress;
  uint8_t lcdCols;
  uint8_t lcdRows;
  uint8_t oledAddress;
  uint8_t oledDriver;         // 0 = SSD1309 NONAME2, 1 = SSD1306 NONAME, 2 = SSD1309 NONAME0
  uint8_t oledContrast;
  uint16_t dimAfterSec;       // 0 = never dim
  uint16_t blankAfterSec;     // 0 = never blank
  uint16_t burnInShiftSec;    // 0 = no pixel shift

  // --- Hardware ---
  bool relayActiveHigh;
  bool requestTxActiveHigh;
  bool buttonsActiveLow;
  uint8_t relayMode;          // RelayMode
  uint16_t latchPulseMs;

  // --- Feature enables ---
  bool autoTune;
  bool autoTuneUnknownCatFrequency;
  bool bypassOnBoot;

  // --- SWR bridge calibration ---
  float fwdOffsetMv;
  float revOffsetMv;
  float fwdScale;
  float revScale;
  float powerScale;
  float swrMinForward;

  // --- Thermal ---
  bool thermalEnabled;
  uint8_t tempAddress;        // 0 = auto-probe LM75/TMP102 family
  float tempWarnC;
  float tempFoldbackC;
  float tempLimitC;
  float ntcBeta;
  float ntcNominalOhms;
  float ntcSeriesOhms;

  // --- Antenna selector ---
  uint8_t antenna;            // currently selected antenna index
  uint8_t antennaCount;

  // --- Wi-Fi / web UI ---
  bool wifiEnabled;
  bool wifiApFallback;
  char wifiSsid[33];
  char wifiPass[65];
  char hostname[25];

  uint32_t checksum;
};

inline void settingsDefaults(Settings& s) {
  memset(&s, 0, sizeof(s));
  s.version = kSettingsVersion;

  s.catBaud = 9600;
  s.catTimeoutMs = 5000;
  s.catProtocol = static_cast<uint8_t>(CatProtocol::Auto);
  s.catEnabled = true;
  s.catPollMs = 1000;
  s.catUsbPassthrough = false;
  s.icomAddress = 0;
  s.catTuneEnabled = false;
  s.catTunePowerW = 10;
  s.catTuneMode = static_cast<uint8_t>(CarrierMode::Fm);
  s.catTuneMaxMs = 15000;

  s.sensorUpdateMs = 100;
  s.displayUpdateMs = 200;
  s.relaySettleMs = 15;
  s.retuneHoldoffMs = 2000;
  s.txRequestLeadMs = 50;
  s.txRequestTrailMs = 50;

  s.minTunePowerW = 1.0f;
  s.minAutoRetunePowerW = 3.0f;
  s.maxTunePowerW = 50.0f;
  s.powerLimitW = 1000.0f;
  s.powerWarningW = 800.0f;
  s.powerOverloadHoldoffMs = 5000;
  s.powerProtEnabled = true;
  s.protectWaitMs = 300;

  s.targetSWR = 1.2f;
  s.goodSWR = 1.5f;
  s.autoRetuneSWR = 2.5f;
  s.maxSWR = 10.0f;

  s.measureSamples = 8;
  s.preciseSamples = 24;
  s.refinePasses = 3;
  s.maxTuneSteps = 160;
  s.tuneModelFit = true;

  s.memoryBinHz = 25000;
  s.maxMemoryEntries = 256;
  s.memoryBandFallback = true;
  s.memoryInterpHz = 300000;

  s.displayType = static_cast<uint8_t>(DisplayKind::Auto);
  s.lcdAddress = 0x27;
  s.lcdCols = 20;
  s.lcdRows = 4;
  s.oledAddress = 0x3C;
  s.oledDriver = 0;
  s.oledContrast = 255;
  s.dimAfterSec = 300;
  s.blankAfterSec = 0;
  s.burnInShiftSec = 120;

  s.relayActiveHigh = true;
  s.requestTxActiveHigh = true;
  s.buttonsActiveLow = true;

  s.autoTune = true;
  s.autoTuneUnknownCatFrequency = true;
  s.bypassOnBoot = false;

  s.fwdOffsetMv = 15.0f;
  s.revOffsetMv = 15.0f;
  s.fwdScale = 1.0f;
  s.revScale = 1.0f;
  s.powerScale = 0.03f;
  s.swrMinForward = 30.0f;

  s.thermalEnabled = true;
  s.tempAddress = 0;
  s.tempWarnC = 55.0f;
  s.tempFoldbackC = 70.0f;
  s.tempLimitC = 85.0f;
  s.ntcBeta = 3950.0f;
  s.ntcNominalOhms = 10000.0f;
  s.ntcSeriesOhms = 10000.0f;

  s.antenna = 0;
  s.antennaCount = 1;

  s.wifiEnabled = false;
  s.wifiApFallback = true;
  s.wifiSsid[0] = '\0';
  s.wifiPass[0] = '\0';
  strncpy(s.hostname, "atu1000", sizeof(s.hostname) - 1);
}


// The loader tells the two layouts apart by blob size.
static_assert(sizeof(Settings) != sizeof(SettingsV2), "settings layouts must differ in size");

// Carries every setting that still exists across from a v2.0 blob.
inline void migrateFromV2(const SettingsV2& o, Settings& s) {
  settingsDefaults(s);
#define ATU_COPY(f) s.f = o.f
  ATU_COPY(catBaud); ATU_COPY(catTimeoutMs); ATU_COPY(catProtocol); ATU_COPY(catEnabled);
  ATU_COPY(sensorUpdateMs); ATU_COPY(displayUpdateMs); ATU_COPY(relaySettleMs);
  ATU_COPY(retuneHoldoffMs); ATU_COPY(txRequestLeadMs); ATU_COPY(txRequestTrailMs);
  ATU_COPY(minTunePowerW); ATU_COPY(minAutoRetunePowerW); ATU_COPY(maxTunePowerW);
  ATU_COPY(powerLimitW); ATU_COPY(powerWarningW); ATU_COPY(powerOverloadHoldoffMs);
  ATU_COPY(powerProtEnabled);
  ATU_COPY(targetSWR); ATU_COPY(goodSWR); ATU_COPY(autoRetuneSWR); ATU_COPY(maxSWR);
  ATU_COPY(refinePasses); ATU_COPY(maxTuneSteps);
  ATU_COPY(memoryBinHz); ATU_COPY(maxMemoryEntries); ATU_COPY(memoryBandFallback);
  ATU_COPY(displayType); ATU_COPY(lcdAddress); ATU_COPY(lcdCols); ATU_COPY(lcdRows);
  ATU_COPY(oledAddress); ATU_COPY(oledDriver); ATU_COPY(oledContrast);
  ATU_COPY(dimAfterSec); ATU_COPY(blankAfterSec); ATU_COPY(burnInShiftSec);
  ATU_COPY(relayActiveHigh); ATU_COPY(requestTxActiveHigh); ATU_COPY(buttonsActiveLow);
  ATU_COPY(autoTune); ATU_COPY(autoTuneUnknownCatFrequency); ATU_COPY(bypassOnBoot);
  ATU_COPY(fwdOffsetMv); ATU_COPY(revOffsetMv); ATU_COPY(fwdScale); ATU_COPY(revScale);
  ATU_COPY(powerScale); ATU_COPY(swrMinForward);
  ATU_COPY(thermalEnabled); ATU_COPY(tempAddress); ATU_COPY(tempWarnC);
  ATU_COPY(tempFoldbackC); ATU_COPY(tempLimitC); ATU_COPY(ntcBeta);
  ATU_COPY(ntcNominalOhms); ATU_COPY(ntcSeriesOhms);
  ATU_COPY(antenna); ATU_COPY(antennaCount);
  ATU_COPY(wifiEnabled); ATU_COPY(wifiApFallback);
#undef ATU_COPY
  memcpy(s.wifiSsid, o.wifiSsid, sizeof(s.wifiSsid));
  memcpy(s.wifiPass, o.wifiPass, sizeof(s.wifiPass));
  memcpy(s.hostname, o.hostname, sizeof(s.hostname));
  // v2 sampled with 3 ms spacing; the sampling task now takes one pair per
  // millisecond, so the old default of 4 samples would be noisier than before.
  if (o.measureSamples > s.measureSamples) s.measureSamples = o.measureSamples;
}

// -----------------------------------------------------------------------------
// Generic key/value description of the struct
// -----------------------------------------------------------------------------

enum class SettingType : uint8_t { F32, U32, U16, U8, Bool, Str };

struct SettingDef {
  const char* key;
  SettingType type;
  uint16_t offset;
  uint16_t len;    // strings only
  float lo;
  float hi;
  const char* help;
};

#define ATU_SET(field) static_cast<uint16_t>(offsetof(Settings, field))

// Ranges are enforced on write so a typo cannot, for example, push the power
// limit past the absolute ceiling or set a zero sample count.
constexpr SettingDef kSettingDefs[] = {
    // Calibration
    {"fwdoffset",  SettingType::F32,  ATU_SET(fwdOffsetMv), 0, 0, 2000, "FWD detector offset (mV)"},
    {"revoffset",  SettingType::F32,  ATU_SET(revOffsetMv), 0, 0, 2000, "REV detector offset (mV)"},
    {"fwdscale",   SettingType::F32,  ATU_SET(fwdScale), 0, 0.001f, 1000, "FWD voltage scale"},
    {"revscale",   SettingType::F32,  ATU_SET(revScale), 0, 0.001f, 1000, "REV voltage scale"},
    {"pwrscale",   SettingType::F32,  ATU_SET(powerScale), 0, 1e-6f, 1000, "V^2 -> W divisor"},
    {"swrminfwd",  SettingType::F32,  ATU_SET(swrMinForward), 0, 1, 2000, "Min FWD mV for a valid SWR"},

    // SWR thresholds
    {"swrtarget",  SettingType::F32,  ATU_SET(targetSWR), 0, 1.0f, 5.0f, "Target SWR (tune success)"},
    {"swrgood",    SettingType::F32,  ATU_SET(goodSWR), 0, 1.0f, 5.0f, "Acceptable SWR"},
    {"swrretune",  SettingType::F32,  ATU_SET(autoRetuneSWR), 0, 1.0f, 20.0f, "Auto-retune SWR threshold"},
    {"swrmax",     SettingType::F32,  ATU_SET(maxSWR), 0, 1.1f, 99.0f, "High-SWR alarm threshold"},

    // Power
    {"pwrmin",     SettingType::F32,  ATU_SET(minTunePowerW), 0, 0, 500, "Min power to tune (W)"},
    {"pwrmax",     SettingType::F32,  ATU_SET(maxTunePowerW), 0, 1, 500, "Max power to tune or switch relays (W)"},
    {"pwrautomin", SettingType::F32,  ATU_SET(minAutoRetunePowerW), 0, 0, 500, "Min power for auto-retune (W)"},
    {"pwrlimit",   SettingType::F32,  ATU_SET(powerLimitW), 0, 10, kAbsMaxPowerLimitW, "Overload limit (W)"},
    {"pwrwarn",    SettingType::F32,  ATU_SET(powerWarningW), 0, 10, kAbsMaxPowerLimitW, "Warning threshold (W)"},
    {"pwrholdoff", SettingType::U32,  ATU_SET(powerOverloadHoldoffMs), 0, 100, 60000, "Overload cooldown (ms)"},
    {"pwrprot",    SettingType::Bool, ATU_SET(powerProtEnabled), 0, 0, 1, "Power protection on/off"},
    {"protwait",   SettingType::U32,  ATU_SET(protectWaitMs), 0, 50, 5000, "Wait for RF to drop before holding (ms)"},

    // Timing
    {"settle",     SettingType::U32,  ATU_SET(relaySettleMs), 0, 1, 500, "Relay settle time (ms)"},
    {"holdoff",    SettingType::U32,  ATU_SET(retuneHoldoffMs), 0, 0, 60000, "Retune holdoff (ms)"},
    {"txlead",     SettingType::U32,  ATU_SET(txRequestLeadMs), 0, 0, 2000, "TX request lead (ms)"},
    {"txtrail",    SettingType::U32,  ATU_SET(txRequestTrailMs), 0, 0, 2000, "TX request trail (ms)"},
    {"sensorms",   SettingType::U32,  ATU_SET(sensorUpdateMs), 0, 10, 5000, "Sensor update period (ms)"},
    {"dispms",     SettingType::U32,  ATU_SET(displayUpdateMs), 0, 20, 5000, "Display update period (ms)"},

    // Tuning
    {"samples",    SettingType::U8,   ATU_SET(measureSamples), 0, 1, 64, "FWD/REV pairs per quick measurement"},
    {"psamples",   SettingType::U8,   ATU_SET(preciseSamples), 0, 1, 64, "FWD/REV pairs per precise measurement"},
    {"refine",     SettingType::U8,   ATU_SET(refinePasses), 0, 0, 10, "Search re-expansions after converging"},
    {"maxsteps",   SettingType::U16,  ATU_SET(maxTuneSteps), 0, 10, 2000, "Max measurements per tune"},
    {"tunemodel",  SettingType::Bool, ATU_SET(tuneModelFit), 0, 0, 1, "Fit a load model during tune"},

    // Memory
    {"membin",     SettingType::U32,  ATU_SET(memoryBinHz), 0, 1000, 1000000, "Memory bin width (Hz)"},
    {"memmax",     SettingType::U16,  ATU_SET(maxMemoryEntries), 0, 8, 1024, "Max memory entries"},
    {"memband",    SettingType::Bool, ATU_SET(memoryBandFallback), 0, 0, 1, "Per-band memory fallback"},
    {"meminterp",  SettingType::U32,  ATU_SET(memoryInterpHz), 0, 0, 2000000, "Blend memories this far apart (Hz, 0=off)"},

    // CAT
    {"catbaud",    SettingType::U32,  ATU_SET(catBaud), 0, 300, 115200, "CAT baud rate"},
    {"cattimeout", SettingType::U32,  ATU_SET(catTimeoutMs), 0, 500, 120000, "CAT link timeout (ms)"},
    {"catproto",   SettingType::U8,   ATU_SET(catProtocol), 0, 0, 6, "0=auto 1=kenwood 2=icom 3=yaesu 4=yaesuN 5=off 6=flex"},
    {"caten",      SettingType::Bool, ATU_SET(catEnabled), 0, 0, 1, "CAT enabled"},
    {"catpoll",    SettingType::U32,  ATU_SET(catPollMs), 0, 0, 60000, "Poll radio frequency (ms, 0=listen only)"},
    {"catusb",     SettingType::Bool, ATU_SET(catUsbPassthrough), 0, 0, 1, "Pass CAT through to the native USB port"},
    {"civaddr",    SettingType::U8,   ATU_SET(icomAddress), 0, 0, 255, "Icom CI-V radio address (0=learn)"},
    {"cattune",    SettingType::Bool, ATU_SET(catTuneEnabled), 0, 0, 1, "TUNE button keys the radio over CAT"},
    {"catpwr",     SettingType::U8,   ATU_SET(catTunePowerW), 0, 1, 100, "CAT tune carrier power (W)"},
    {"catmode",    SettingType::U8,   ATU_SET(catTuneMode), 0, 0, 2, "CAT tune carrier 0=FM 1=AM 2=CW"},
    {"catmaxms",   SettingType::U32,  ATU_SET(catTuneMaxMs), 0, 2000, 60000, "CAT tune key-down limit (ms)"},

    // Display
    {"disptype",   SettingType::U8,   ATU_SET(displayType), 0, 0, 3, "0=auto 1=none 2=lcd 3=oled"},
    {"lcdaddr",    SettingType::U8,   ATU_SET(lcdAddress), 0, 1, 127, "LCD I2C address"},
    {"lcdcols",    SettingType::U8,   ATU_SET(lcdCols), 0, 8, 40, "LCD columns"},
    {"lcdrows",    SettingType::U8,   ATU_SET(lcdRows), 0, 2, 4, "LCD rows"},
    {"oledaddr",   SettingType::U8,   ATU_SET(oledAddress), 0, 1, 127, "OLED I2C address"},
    {"oleddrv",    SettingType::U8,   ATU_SET(oledDriver), 0, 0, 2, "0=SSD1309/N2 1=SSD1306 2=SSD1309/N0"},
    {"contrast",   SettingType::U8,   ATU_SET(oledContrast), 0, 1, 255, "OLED contrast"},
    {"dimsec",     SettingType::U16,  ATU_SET(dimAfterSec), 0, 0, 36000, "Dim after idle (s, 0=never)"},
    {"blanksec",   SettingType::U16,  ATU_SET(blankAfterSec), 0, 0, 36000, "Blank after idle (s, 0=never)"},
    {"burnsec",    SettingType::U16,  ATU_SET(burnInShiftSec), 0, 0, 36000, "Pixel shift period (s, 0=off)"},

    // Hardware
    {"relayhigh",  SettingType::Bool, ATU_SET(relayActiveHigh), 0, 0, 1, "Relay drive active high"},
    {"txhigh",     SettingType::Bool, ATU_SET(requestTxActiveHigh), 0, 0, 1, "TX request active high"},
    {"btnlow",     SettingType::Bool, ATU_SET(buttonsActiveLow), 0, 0, 1, "Buttons active low"},

    // Features
    {"autotune",   SettingType::Bool, ATU_SET(autoTune), 0, 0, 1, "Auto-tune enabled"},
    {"autounknown",SettingType::Bool, ATU_SET(autoTuneUnknownCatFrequency), 0, 0, 1, "Auto-tune unknown CAT freq"},
    {"bypassboot", SettingType::Bool, ATU_SET(bypassOnBoot), 0, 0, 1, "Start in bypass"},

    // Thermal
    {"tempen",     SettingType::Bool, ATU_SET(thermalEnabled), 0, 0, 1, "Thermal monitoring on/off"},
    {"tempaddr",   SettingType::U8,   ATU_SET(tempAddress), 0, 0, 127, "Temp sensor addr (0=auto)"},
    {"tempwarn",   SettingType::F32,  ATU_SET(tempWarnC), 0, 0, kAbsMaxTempLimitC, "Temp warning (C)"},
    {"tempfold",   SettingType::F32,  ATU_SET(tempFoldbackC), 0, 0, kAbsMaxTempLimitC, "Temp foldback - inhibit TX (C)"},
    {"templimit",  SettingType::F32,  ATU_SET(tempLimitC), 0, 0, kAbsMaxTempLimitC, "Temp shutdown - bypass (C)"},
    {"ntcbeta",    SettingType::F32,  ATU_SET(ntcBeta), 0, 100, 10000, "NTC beta constant"},
    {"ntcnominal", SettingType::F32,  ATU_SET(ntcNominalOhms), 0, 100, 1e6f, "NTC resistance at 25C"},
    {"ntcseries",  SettingType::F32,  ATU_SET(ntcSeriesOhms), 0, 100, 1e6f, "NTC divider series R"},

    // Antenna
    {"ant",        SettingType::U8,   ATU_SET(antenna), 0, 0, 7, "Selected antenna"},
    {"antcount",   SettingType::U8,   ATU_SET(antennaCount), 0, 1, 8, "Number of antennas"},

    // Wi-Fi
    {"wifien",     SettingType::Bool, ATU_SET(wifiEnabled), 0, 0, 1, "Wi-Fi enabled"},
    {"wifiap",     SettingType::Bool, ATU_SET(wifiApFallback), 0, 0, 1, "Fall back to AP mode"},
    {"wifissid",   SettingType::Str,  ATU_SET(wifiSsid), sizeof(Settings::wifiSsid), 0, 0, "Wi-Fi SSID"},
    {"wifipass",   SettingType::Str,  ATU_SET(wifiPass), sizeof(Settings::wifiPass), 0, 0, "Wi-Fi passphrase"},
    {"hostname",   SettingType::Str,  ATU_SET(hostname), sizeof(Settings::hostname), 0, 0, "mDNS / AP hostname"},
};

#undef ATU_SET

constexpr size_t kSettingDefCount = sizeof(kSettingDefs) / sizeof(kSettingDefs[0]);

// -----------------------------------------------------------------------------
// Store
// -----------------------------------------------------------------------------

class SettingsStore {
 public:
  void begin() {
    prefs_.begin("atu_cfg", false);
    settingsDefaults(s_);
    size_t len = prefs_.getBytesLength("cfg");
    if (len == sizeof(Settings)) {
      Settings loaded;
      prefs_.getBytes("cfg", &loaded, sizeof(loaded));
      if (loaded.version == kSettingsVersion && checksumOf(loaded) == loaded.checksum) {
        s_ = loaded;
        loadedOk_ = true;
      }
    } else if (len == sizeof(SettingsV2)) {
      SettingsV2 old;
      prefs_.getBytes("cfg", &old, sizeof(old));
      uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&old),
                           sizeof(SettingsV2) - sizeof(old.checksum));
      if (old.version == 2 && crc == old.checksum) {
        migrateFromV2(old, s_);
        loadedOk_ = true;
        migrated_ = true;
      }
    }
    clampAll();
    if (migrated_) save();
  }

  Settings& get() { return s_; }
  const Settings& get() const { return s_; }
  bool loadedFromNvs() const { return loadedOk_; }
  bool migrated() const { return migrated_; }

  void save() {
    clampAll();
    s_.version = kSettingsVersion;
    s_.checksum = checksumOf(s_);
    prefs_.putBytes("cfg", &s_, sizeof(s_));
    dirty_ = false;
  }

  void markDirty() { dirty_ = true; dirtyMs_ = millis(); }

  // Coalesce rapid edits into a single flash write.
  void loop() {
    if (dirty_ && elapsed(dirtyMs_) > 1500) save();
  }

  void resetDefaults() {
    settingsDefaults(s_);
    save();
  }

  static const SettingDef* find(const char* key) {
    for (size_t i = 0; i < kSettingDefCount; ++i) {
      if (strcasecmp(kSettingDefs[i].key, key) == 0) return &kSettingDefs[i];
    }
    return nullptr;
  }

  void format(const SettingDef& d, char* out, size_t outLen) const {
    const uint8_t* base = reinterpret_cast<const uint8_t*>(&s_) + d.offset;
    switch (d.type) {
      case SettingType::F32: {
        float v;
        memcpy(&v, base, sizeof(v));
        snprintf(out, outLen, "%.4g", v);
        break;
      }
      case SettingType::U32: {
        uint32_t v;
        memcpy(&v, base, sizeof(v));
        snprintf(out, outLen, "%lu", static_cast<unsigned long>(v));
        break;
      }
      case SettingType::U16: {
        uint16_t v;
        memcpy(&v, base, sizeof(v));
        snprintf(out, outLen, "%u", static_cast<unsigned>(v));
        break;
      }
      case SettingType::U8: {
        uint8_t v = *base;
        snprintf(out, outLen, "%u", static_cast<unsigned>(v));
        break;
      }
      case SettingType::Bool:
        snprintf(out, outLen, "%s", (*base) ? "on" : "off");
        break;
      case SettingType::Str:
        // Never echo the Wi-Fi passphrase back over the console or the web API.
        if (d.offset == offsetof(Settings, wifiPass)) {
          snprintf(out, outLen, "%s", (*base) ? "<set>" : "<empty>");
        } else {
          snprintf(out, outLen, "%s", reinterpret_cast<const char*>(base));
        }
        break;
    }
  }

  bool set(const SettingDef& d, const char* value, char* err, size_t errLen) {
    uint8_t* base = reinterpret_cast<uint8_t*>(&s_) + d.offset;

    if (d.type == SettingType::Str) {
      size_t n = strlen(value);
      if (n >= d.len) {
        snprintf(err, errLen, "too long (max %u)", static_cast<unsigned>(d.len - 1));
        return false;
      }
      memcpy(base, value, n);
      base[n] = '\0';
      markDirty();
      return true;
    }

    if (d.type == SettingType::Bool) {
      bool v;
      if (!parseBool(value, v)) {
        snprintf(err, errLen, "expected on/off");
        return false;
      }
      *base = v ? 1 : 0;
      markDirty();
      return true;
    }

    char* end = nullptr;
    double v = strtod(value, &end);
    if (end == value) {
      snprintf(err, errLen, "not a number");
      return false;
    }
    if (v < d.lo || v > d.hi) {
      snprintf(err, errLen, "out of range %.4g..%.4g", d.lo, d.hi);
      return false;
    }

    switch (d.type) {
      case SettingType::F32: { float f = static_cast<float>(v); memcpy(base, &f, sizeof(f)); break; }
      case SettingType::U32: { uint32_t u = static_cast<uint32_t>(v); memcpy(base, &u, sizeof(u)); break; }
      case SettingType::U16: { uint16_t u = static_cast<uint16_t>(v); memcpy(base, &u, sizeof(u)); break; }
      case SettingType::U8:  { *base = static_cast<uint8_t>(v); break; }
      default: break;
    }
    markDirty();
    return true;
  }

 private:
  Preferences prefs_;
  Settings s_{};
  bool dirty_ = false;
  bool loadedOk_ = false;
  bool migrated_ = false;
  uint32_t dirtyMs_ = 0;

  static uint32_t checksumOf(const Settings& s) {
    return crc32(reinterpret_cast<const uint8_t*>(&s),
                 sizeof(Settings) - sizeof(s.checksum));
  }

  static bool parseBool(const char* v, bool& out) {
    if (!strcasecmp(v, "on") || !strcasecmp(v, "1") || !strcasecmp(v, "true") ||
        !strcasecmp(v, "yes") || !strcasecmp(v, "enable")) { out = true; return true; }
    if (!strcasecmp(v, "off") || !strcasecmp(v, "0") || !strcasecmp(v, "false") ||
        !strcasecmp(v, "no") || !strcasecmp(v, "disable")) { out = false; return true; }
    return false;
  }

  // Re-apply every declared range plus a few cross-field invariants. Called on
  // load (so an old or corrupted blob can never produce a dangerous config) and
  // on save.
  void clampAll() {
    for (size_t i = 0; i < kSettingDefCount; ++i) {
      const SettingDef& d = kSettingDefs[i];
      uint8_t* base = reinterpret_cast<uint8_t*>(&s_) + d.offset;
      switch (d.type) {
        case SettingType::F32: {
          float v;
          memcpy(&v, base, sizeof(v));
          if (!isfinite(v)) v = d.lo;
          v = clampf(v, d.lo, d.hi);
          memcpy(base, &v, sizeof(v));
          break;
        }
        case SettingType::U32: {
          uint32_t v;
          memcpy(&v, base, sizeof(v));
          if (v < static_cast<uint32_t>(d.lo)) v = static_cast<uint32_t>(d.lo);
          if (v > static_cast<uint32_t>(d.hi)) v = static_cast<uint32_t>(d.hi);
          memcpy(base, &v, sizeof(v));
          break;
        }
        case SettingType::U16: {
          uint16_t v;
          memcpy(&v, base, sizeof(v));
          if (v < static_cast<uint16_t>(d.lo)) v = static_cast<uint16_t>(d.lo);
          if (v > static_cast<uint16_t>(d.hi)) v = static_cast<uint16_t>(d.hi);
          memcpy(base, &v, sizeof(v));
          break;
        }
        case SettingType::U8: {
          if (*base < static_cast<uint8_t>(d.lo)) *base = static_cast<uint8_t>(d.lo);
          if (*base > static_cast<uint8_t>(d.hi)) *base = static_cast<uint8_t>(d.hi);
          break;
        }
        case SettingType::Bool:
          *base = (*base) ? 1 : 0;
          break;
        case SettingType::Str:
          base[d.len - 1] = '\0';
          break;
      }
    }

    // Cross-field invariants.
    if (s_.powerWarningW >= s_.powerLimitW) s_.powerWarningW = s_.powerLimitW * 0.8f;
    if (s_.goodSWR < s_.targetSWR) s_.goodSWR = s_.targetSWR;
    if (s_.autoRetuneSWR < s_.goodSWR) s_.autoRetuneSWR = s_.goodSWR;
    if (s_.maxTunePowerW < s_.minTunePowerW) s_.maxTunePowerW = s_.minTunePowerW;
    if (s_.preciseSamples < s_.measureSamples) s_.preciseSamples = s_.measureSamples;
    if (s_.catProtocol > static_cast<uint8_t>(CatProtocol::Flex)) s_.catProtocol = 0;
    if (s_.tempFoldbackC > s_.tempLimitC) s_.tempFoldbackC = s_.tempLimitC;
    if (s_.tempWarnC > s_.tempFoldbackC) s_.tempWarnC = s_.tempFoldbackC;
    if (s_.antenna >= s_.antennaCount) s_.antenna = 0;
    s_.wifiSsid[sizeof(s_.wifiSsid) - 1] = '\0';
    s_.wifiPass[sizeof(s_.wifiPass) - 1] = '\0';
    s_.hostname[sizeof(s_.hostname) - 1] = '\0';
    if (s_.hostname[0] == '\0') strncpy(s_.hostname, "atu1000", sizeof(s_.hostname) - 1);
  }
};

}  // namespace atu
