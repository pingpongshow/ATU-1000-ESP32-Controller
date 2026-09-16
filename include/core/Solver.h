#pragma once
//
// L-network seeding.
//
// A single-directional SWR bridge measures |Gamma| only - there is no phase
// information - so the load impedance cannot be solved for from one reading.
// These analytic seeds (ideal L-match into a ladder of plausible resistive
// loads) give the search well-spread starting points; LoadFit.h then uses the
// seed measurements to estimate the complex load properly.
//

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "Components.h"
#include "Network.h"
#include "RelayState.h"

namespace atu {

// Closest achievable total inductance, as a relay mask. 128 combinations is
// nothing to brute-force and it handles the non-ideal real component values.
inline uint8_t nearestLMask(float targetUh) {
  uint8_t best = 0;
  float bestErr = 1e9f;
  for (uint16_t mask = 0; mask < 128; ++mask) {
    float err = std::fabs(maskToUh(static_cast<uint8_t>(mask)) - targetUh);
    if (err < bestErr) { bestErr = err; best = static_cast<uint8_t>(mask); }
  }
  return best;
}

inline uint8_t nearestCMask(float targetPf) {
  uint8_t best = 0;
  float bestErr = 1e9f;
  for (uint16_t mask = 0; mask < 128; ++mask) {
    float err = std::fabs(static_cast<float>(maskToPf(static_cast<uint8_t>(mask))) - targetPf);
    if (err < bestErr) { bestErr = err; best = static_cast<uint8_t>(mask); }
  }
  return best;
}

// Lowpass L-match between 50 ohms and a purely resistive rLoad.
//
//   rLoad < 50:  Q = sqrt(50/rLoad - 1), Xshunt = 50/Q, Xseries = Q*rLoad
//   rLoad > 50:  Q = sqrt(rLoad/50 - 1), Xshunt = rLoad/Q, Xseries = Q*50
//
// The series arm is the inductor and the shunt arm the capacitor; which side
// the capacitor sits on is exactly what the K2 topology relay selects, so both
// topologies are seeded and the measurement decides.
inline bool designLNetwork(uint32_t freqHz, float rLoad, float& lUh, float& cPf) {
  if (freqHz < 100000UL || rLoad <= 0.0f) return false;
  const float rs = kZ0;
  const float w = kTwoPi * static_cast<float>(freqHz);

  float q, xSeries, xShunt;
  if (rLoad < rs) {
    q = std::sqrt(rs / rLoad - 1.0f);
    xShunt = rs / q;
    xSeries = q * rLoad;
  } else if (rLoad > rs) {
    q = std::sqrt(rLoad / rs - 1.0f);
    xShunt = rLoad / q;
    xSeries = q * rs;
  } else {
    lUh = 0.0f;
    cPf = 0.0f;
    return true;   // already matched: straight through
  }
  if (!std::isfinite(q) || q <= 0.0f) return false;

  lUh = (xSeries / w) * 1e6f;              // henries -> microhenries
  cPf = (1.0f / (w * xShunt)) * 1e12f;     // farads  -> picofarads
  return std::isfinite(lUh) && std::isfinite(cPf);
}

// A spread of load resistances that covers most of what a real antenna
// presents at the tuner, from a badly-fed low-Z load to a high-Z end-fed.
constexpr float kSeedResistances[] = {10.0f, 20.0f, 100.0f, 200.0f, 450.0f, 800.0f};
constexpr size_t kSeedResistanceCount =
    sizeof(kSeedResistances) / sizeof(kSeedResistances[0]);

// Fills `out` with up to `maxOut` analytically derived seeds for one topology.
// Returns how many were written.
inline size_t buildAnalyticSeeds(uint32_t freqHz, bool topology, RelayState* out,
                                 size_t maxOut) {
  size_t n = 0;
  for (size_t i = 0; i < kSeedResistanceCount && n < maxOut; ++i) {
    float lUh = 0.0f, cPf = 0.0f;
    if (!designLNetwork(freqHz, kSeedResistances[i], lUh, cPf)) continue;
    if (lUh > kLTotalUh * 1.5f || cPf > kCTotalPf * 1.5f) continue;

    RelayState s;
    s.lMask = nearestLMask(lUh > kLTotalUh ? kLTotalUh : lUh);
    s.cMask = nearestCMask(cPf > kCTotalPf ? static_cast<float>(kCTotalPf) : cPf);
    s.topology = topology;
    s.bypass = false;

    // Skip duplicates - several nearby resistances often snap to the same
    // achievable combination.
    bool dup = false;
    for (size_t j = 0; j < n; ++j) {
      if (out[j] == s) { dup = true; break; }
    }
    if (!dup) out[n++] = s;
  }
  return n;
}

}  // namespace atu
