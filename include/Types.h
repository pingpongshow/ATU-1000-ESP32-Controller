#pragma once
//
// Shared value types and small helpers used across the whole firmware.
//

#include <Arduino.h>

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "Config.h"
#include "RelayState.h"

namespace atu {

// -----------------------------------------------------------------------------
// Sensor
// -----------------------------------------------------------------------------

// swr and gamma are only meaningful when `valid` is true. When there is no
// detectable forward power `valid` is false and swr/powerW are zeroed rather
// than being left at a sentinel that the UI would happily draw as a full-scale
// bar.
struct SensorReading {
  float fwdMv = 0.0f;
  float revMv = 0.0f;
  float fwdRaw = 0.0f;
  float revRaw = 0.0f;
  float swr = 1.0f;
  float gamma = 0.0f;       // |reflection coefficient|, median of per-pair ratios
  float gammaNoise = 0.0f;  // standard error of `gamma`
  float powerW = 0.0f;
  float peakPowerW = 0.0f;  // highest single-pair power in the reading
  uint8_t pairs = 0;        // FWD/REV sample pairs behind this reading
  bool valid = false;
};

struct SweepPoint {
  uint32_t freqHz;
  float swr;
  float powerW;
};

// -----------------------------------------------------------------------------
// Results
// -----------------------------------------------------------------------------

enum class CarrierMode : uint8_t { Fm = 0, Am = 1, Cw = 2 };

enum class TuneResult : uint8_t {
  None,
  Success,
  GoodEnough,
  NoPower,
  PowerHigh,
  PowerOverload,
  OverTemp,
  Failed,
  Aborted,
  Busy,
};

inline const char* tuneResultName(TuneResult r) {
  switch (r) {
    case TuneResult::Success: return "TUNE OK";
    case TuneResult::GoodEnough: return "TUNE GOOD";
    case TuneResult::NoPower: return "NO RF";
    case TuneResult::PowerHigh: return "PWR HIGH";
    case TuneResult::PowerOverload: return "OVERLOAD";
    case TuneResult::OverTemp: return "OVER TEMP";
    case TuneResult::Failed: return "TUNE FAIL";
    case TuneResult::Aborted: return "ABORTED";
    case TuneResult::Busy: return "BUSY";
    default: return "";
  }
}

enum class CatProtocol : uint8_t {
  Auto = 0,
  Kenwood = 1,  // Kenwood / Elecraft / FlexRadio ASCII
  Icom = 2,     // Icom CI-V binary
  YaesuOld = 3, // Yaesu 5-byte binary (FT-817/857/897)
  YaesuNew = 4, // Yaesu newer ASCII (FTDX101 etc)
  None = 5,
  Flex = 6,     // FlexRadio SmartSDR CAT (Kenwood-style plus ZZ extensions)
};

inline const char* catProtocolName(CatProtocol p) {
  switch (p) {
    case CatProtocol::Kenwood: return "Kenwood";
    case CatProtocol::Icom: return "Icom";
    case CatProtocol::YaesuOld: return "Yaesu";
    case CatProtocol::YaesuNew: return "YaesuN";
    case CatProtocol::Flex: return "Flex";
    case CatProtocol::Auto: return "Auto";
    default: return "None";
  }
}

inline bool parseCatProtocol(const char* s, CatProtocol& out) {
  if (strcasecmp(s, "auto") == 0) { out = CatProtocol::Auto; return true; }
  if (strcasecmp(s, "kenwood") == 0 || strcasecmp(s, "elecraft") == 0) { out = CatProtocol::Kenwood; return true; }
  if (strcasecmp(s, "icom") == 0 || strcasecmp(s, "civ") == 0) { out = CatProtocol::Icom; return true; }
  if (strcasecmp(s, "yaesu") == 0 || strcasecmp(s, "yaesuold") == 0) { out = CatProtocol::YaesuOld; return true; }
  if (strcasecmp(s, "yaesun") == 0 || strcasecmp(s, "yaesunew") == 0) { out = CatProtocol::YaesuNew; return true; }
  if (strcasecmp(s, "flex") == 0 || strcasecmp(s, "flexradio") == 0 || strcasecmp(s, "smartsdr") == 0) { out = CatProtocol::Flex; return true; }
  if (strcasecmp(s, "off") == 0 || strcasecmp(s, "none") == 0) { out = CatProtocol::None; return true; }
  return false;
}

enum class DisplayKind : uint8_t {
  Auto = 0,
  None = 1,
  Lcd = 2,
  Oled = 3,
};

// -----------------------------------------------------------------------------
// Small utilities
// -----------------------------------------------------------------------------

inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

inline uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return ~crc;
}

// millis() difference that is correct across the 49.7 day rollover.
inline uint32_t elapsed(uint32_t sinceMs) {
  return static_cast<uint32_t>(millis() - sinceMs);
}

// Fixed-capacity status text. Replaces the old Arduino String global, which
// re-allocated on the heap several times a second for the lifetime of the box.
struct StatusText {
  char buf[24];

  StatusText() { buf[0] = '\0'; }
  void set(const char* s) {
    strncpy(buf, s ? s : "", sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
  }
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
  }
  const char* c_str() const { return buf; }
};

}  // namespace atu
