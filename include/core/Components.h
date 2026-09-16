#pragma once
//
// Physical component values and the amateur band table.
//
// Free of Arduino headers so the tuning maths can be built and tested on a
// desktop machine (see test/).
//

#include <cstddef>
#include <cstdint>

namespace atu {

constexpr uint8_t kRelayCount = 7;

// =============================================================================
// Component values from schematic - binary weighted LC network
// =============================================================================

// L values in microhenries (uH) - from ATU-1000 schematic
// Binary weighted inductors, matched to relay K designators
constexpr float kLValuesUh[kRelayCount] = {
    0.05f,   // L[0]: K1  (Q13) - 0.05 uH
    0.10f,   // L[1]: K4  (Q15) - 0.1 uH
    0.22f,   // L[2]: K6  (Q14) - 0.22 uH
    0.45f,   // L[3]: K8  (Q12) - 0.45 uH
    1.00f,   // L[4]: K10 (Q10) - 1.0 uH
    2.20f,   // L[5]: K12 (Q11) - 2.2 uH
    4.70f    // L[6]: K14 (Q9)  - 4.7 uH
};

// C values in picofarads (pF) - silver mica 5kV
// Binary weighted capacitors, matched to relay K designators
constexpr uint16_t kCValuesPf[kRelayCount] = {
    10,      // C[0]: K3  (Q6) - 10 pF
    22,      // C[1]: K5  (Q8) - 22 pF
    47,      // C[2]: K7  (Q7) - 47 pF
    100,     // C[3]: K9  (Q5) - 100 pF
    220,     // C[4]: K11 (Q3) - 220 pF
    470,     // C[5]: K13 (Q4) - 470 pF
    1000     // C[6]: K15 (Q2) - 1000 pF (1nF)
};

// Total L range: 0.05 to 8.72 uH (all on)
// Total C range: 10 to 1869 pF (all on)
constexpr float kLTotalUh = 8.72f;
constexpr uint16_t kCTotalPf = 1869;

// =============================================================================
// Band definitions for frequency-based tuning hints
// =============================================================================

struct BandInfo {
  uint32_t lowHz;
  uint32_t highHz;
  const char* name;
  uint8_t typicalLMask;
  uint8_t typicalCMask;
  bool typicalTopology;  // true = Hi-Z, false = Lo-Z
};

constexpr BandInfo kBands[] = {
    {1800000,   2000000,  "160m", 0x7F, 0x7F, false},
    {3500000,   4000000,  "80m",  0x3F, 0x3F, false},
    {5330000,   5410000,  "60m",  0x1F, 0x1F, false},
    {7000000,   7300000,  "40m",  0x1F, 0x1F, false},
    {10100000,  10150000, "30m",  0x0F, 0x0F, true},
    {14000000,  14350000, "20m",  0x0F, 0x0F, true},
    {18068000,  18168000, "17m",  0x07, 0x07, true},
    {21000000,  21450000, "15m",  0x07, 0x07, true},
    {24890000,  24990000, "12m",  0x03, 0x03, true},
    {28000000,  29700000, "10m",  0x03, 0x03, true},
    {50000000,  54000000, "6m",   0x01, 0x01, true},
};
constexpr size_t kBandCount = sizeof(kBands) / sizeof(kBands[0]);

inline const BandInfo* findBand(uint32_t freqHz) {
  for (size_t i = 0; i < kBandCount; ++i) {
    if (freqHz >= kBands[i].lowHz && freqHz <= kBands[i].highHz) {
      return &kBands[i];
    }
  }
  return nullptr;
}

// Index of the band containing freqHz, or -1. Used for the per-band memory
// fallback so a 20m entry is never matched against a 40m frequency.
inline int findBandIndex(uint32_t freqHz) {
  for (size_t i = 0; i < kBandCount; ++i) {
    if (freqHz >= kBands[i].lowHz && freqHz <= kBands[i].highHz) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

inline const char* bandName(uint32_t freqHz) {
  const BandInfo* b = findBand(freqHz);
  return b ? b->name : "";
}

}  // namespace atu
