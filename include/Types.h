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

namespace atu {

// -----------------------------------------------------------------------------
// Relay / tuner state
// -----------------------------------------------------------------------------

struct RelayState {
  uint8_t lMask = 0;
  uint8_t cMask = 0;
  bool topology = false;  // false = Lo-Z, true = Hi-Z
  bool bypass = false;

  bool operator==(const RelayState& o) const {
    return lMask == o.lMask && cMask == o.cMask && topology == o.topology &&
           bypass == o.bypass;
  }
  bool operator!=(const RelayState& o) const { return !(*this == o); }
};

enum RelayFlag : uint8_t {
  FLAG_TOPOLOGY = 0x01,
  FLAG_BYPASS = 0x02,
};

inline uint8_t packFlags(const RelayState& s) {
  return static_cast<uint8_t>((s.topology ? FLAG_TOPOLOGY : 0) |
                              (s.bypass ? FLAG_BYPASS : 0));
}

inline void unpackFlags(uint8_t flags, RelayState& s) {
  s.topology = (flags & FLAG_TOPOLOGY) != 0;
  s.bypass = (flags & FLAG_BYPASS) != 0;
}

// -----------------------------------------------------------------------------
// Sensor
// -----------------------------------------------------------------------------

// swr is only meaningful when `valid` is true. When there is no detectable
// forward power `valid` is false and swr/powerW are zeroed rather than being
// left at a sentinel that the UI would happily draw as a full-scale bar.
struct SensorReading {
  float fwdMv = 0.0f;
  float revMv = 0.0f;
  float fwdRaw = 0.0f;
  float revRaw = 0.0f;
  float swr = 1.0f;
  float powerW = 0.0f;
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
};

inline const char* catProtocolName(CatProtocol p) {
  switch (p) {
    case CatProtocol::Kenwood: return "Kenwood";
    case CatProtocol::Icom: return "Icom";
    case CatProtocol::YaesuOld: return "Yaesu";
    case CatProtocol::YaesuNew: return "YaesuN";
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
  if (strcasecmp(s, "off") == 0 || strcasecmp(s, "none") == 0) { out = CatProtocol::None; return true; }
  return false;
}

enum class DisplayKind : uint8_t {
  Auto = 0,
  None = 1,
  Lcd = 2,
  Oled = 3,
};

enum class RelayMode : uint8_t {
  Continuous = 0,  // coil energised for as long as the relay is selected
  Pulsed = 1,      // coil driven for latchPulseMs then released (impulse relays)
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
