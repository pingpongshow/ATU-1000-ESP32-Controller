#pragma once
//
// L-network circuit model.
//
// Used by the tuner to predict |Gamma| for any relay combination once the load
// has been estimated (LoadFit.h), and by the desktop test harness as the
// simulated "real" tuner, with parasitics switched on.
//
// Topology convention: RelayState::topology == true (Hi-Z) puts the capacitor
// bank on the load side of the series inductor, which is the physically
// correct arrangement for a load above 50 ohms. Whether K2 actually does that
// on a given board is not assumed: the fitter tries both mappings and lets the
// measurements decide.
//

#include <cmath>
#include <complex>
#include <cstdint>

#include "Components.h"
#include "RelayState.h"

namespace atu {

using Cplx = std::complex<float>;

constexpr float kZ0 = 50.0f;
constexpr float kTwoPi = 6.2831853f;

// |Gamma| is capped here, which is SWR 99 - the same ceiling the UI has always
// displayed.
constexpr float kGammaCeiling = 0.98f;

inline float swrToGamma(float swr) {
  if (!(swr > 1.0f)) return 0.0f;
  return (swr - 1.0f) / (swr + 1.0f);
}

inline float gammaToSwr(float gamma) {
  if (!(gamma > 0.0f)) return 1.0f;
  if (gamma > kGammaCeiling) gamma = kGammaCeiling;
  return (1.0f + gamma) / (1.0f - gamma);
}

inline float maskToUh(uint8_t mask) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < kRelayCount; ++i) {
    if ((mask >> i) & 1) sum += kLValuesUh[i];
  }
  return sum;
}

inline uint16_t maskToPf(uint8_t mask) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < kRelayCount; ++i) {
    if ((mask >> i) & 1) sum += kCValuesPf[i];
  }
  return sum;
}

// -----------------------------------------------------------------------------
// Value-ordered relay combinations
// -----------------------------------------------------------------------------
//
// Adjacent achievable values are often many bit flips apart (0x3F -> 0x40 is
// seven relays), so the search walks an index into the combinations sorted by
// real value instead of flipping bits.

constexpr uint8_t kComboCount = 128;

struct Ladder {
  uint8_t maskAt[kComboCount];    // index -> relay mask
  uint8_t indexOf[kComboCount];   // relay mask -> index
  float valueAt[kComboCount];     // index -> uH or pF
};

inline Ladder buildLadder(bool inductors) {
  Ladder l;
  float v[kComboCount];
  uint8_t order[kComboCount];
  for (int m = 0; m < kComboCount; ++m) {
    v[m] = inductors ? maskToUh(static_cast<uint8_t>(m))
                     : static_cast<float>(maskToPf(static_cast<uint8_t>(m)));
    order[m] = static_cast<uint8_t>(m);
  }
  for (int i = 1; i < kComboCount; ++i) {
    uint8_t k = order[i];
    int j = i - 1;
    while (j >= 0 && v[order[j]] > v[k]) {
      order[j + 1] = order[j];
      --j;
    }
    order[j + 1] = k;
  }
  for (int i = 0; i < kComboCount; ++i) {
    l.maskAt[i] = order[i];
    l.indexOf[order[i]] = static_cast<uint8_t>(i);
    l.valueAt[i] = v[order[i]];
  }
  return l;
}

inline const Ladder& lLadder() {
  static const Ladder l = buildLadder(true);
  return l;
}

inline const Ladder& cLadder() {
  static const Ladder l = buildLadder(false);
  return l;
}

// -----------------------------------------------------------------------------
// Circuit
// -----------------------------------------------------------------------------

// Non-idealities. All zero / unity for the tuner's own model; the test harness
// turns them on so the search is exercised against a model it does not match.
struct Parasitics {
  float lScale = 1.0f;       // component tolerance
  float cScale = 1.0f;
  float seriesLUh = 0.0f;    // wiring inductance in series with the L bank
  float strayInPf = 0.0f;    // shunt stray at the transmitter port
  float strayOutPf = 0.0f;   // shunt stray at the antenna port
  float inductorQ = 0.0f;    // 0 = lossless
};

inline Cplx shuntC(Cplx z, float w, float farads) {
  if (farads <= 0.0f) return z;
  Cplx y = Cplx(1.0f, 0.0f) / z + Cplx(0.0f, w * farads);
  return Cplx(1.0f, 0.0f) / y;
}

inline Cplx networkInputZ(float freqHz, float lUh, float cPf, bool capsAtLoad,
                          Cplx zLoad, const Parasitics& p) {
  const float w = kTwoPi * freqHz;
  const float henries = (lUh * p.lScale + p.seriesLUh) * 1e-6f;
  const float farads = cPf * p.cScale * 1e-12f;

  Cplx zSeries(0.0f, w * henries);
  if (p.inductorQ > 0.0f && henries > 0.0f) zSeries += w * henries / p.inductorQ;

  Cplx z = shuntC(zLoad, w, p.strayOutPf * 1e-12f);
  if (capsAtLoad) {
    z = shuntC(z, w, farads);
    z += zSeries;
  } else {
    z += zSeries;
    z = shuntC(z, w, farads);
  }
  return shuntC(z, w, p.strayInPf * 1e-12f);
}

inline bool capsAtLoadFor(bool topology, bool invertedMapping) {
  return topology != invertedMapping;
}

inline float gammaOf(Cplx zin) {
  Cplx g = (zin - kZ0) / (zin + kZ0);
  float m = std::abs(g);
  if (!std::isfinite(m)) return 1.0f;
  return m > 1.0f ? 1.0f : m;
}

inline float modelGamma(uint32_t freqHz, const RelayState& s, bool invertedMapping,
                        Cplx zLoad, const Parasitics& p = Parasitics()) {
  float lUh = s.bypass ? 0.0f : maskToUh(s.lMask);
  float cPf = s.bypass ? 0.0f : static_cast<float>(maskToPf(s.cMask));
  return gammaOf(networkInputZ(static_cast<float>(freqHz), lUh, cPf,
                               capsAtLoadFor(s.topology, invertedMapping), zLoad, p));
}

}  // namespace atu
