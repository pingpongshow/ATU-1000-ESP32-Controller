#pragma once
//
// Compile-time configuration: pin map, component values, band table and the
// factory defaults for the NVS-backed Settings block.
//
// Anything a user might reasonably want to change in the field lives in
// Settings (see Settings.h) and is editable at runtime with `set <key> <val>`.
// Only things that are physically fixed by the board wiring live here.
//

#include <Arduino.h>

namespace atu {

constexpr uint8_t kRelayCount = 7;
constexpr uint8_t kAntennaPinCount = 3;

constexpr const char* kFirmwareVersion = "2.0";

struct Pins {
  int lRelays[kRelayCount];
  int cRelays[kRelayCount];
  int topologyRelay;
  int fwdAdc;
  int revAdc;
  int ntcAdc;                          // optional NTC thermistor, -1 = disabled
  int tuneButton;
  int bypassButton;
  int autoButton;
  int catRx;
  int catTx;
  int txReq;
  int txReqInv;
  int i2cSda;
  int i2cScl;
  int ledStatus;
  int antenna[kAntennaPinCount];       // binary-coded antenna select, -1 = unused
};

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

// =============================================================================
// ESP32-S3 Pin Configuration - Directly wired to PIC16F1938 socket
// =============================================================================
//
// The ESP32 connects to the PIC socket pins via the MJD122 transistor drivers.
// PIC is removed; ESP32 GPIO directly drives the transistor base resistors.
//
// ESP32 GPIO to PIC Socket Pin Mapping (relay outputs):
// -------------------------------------------------------
// ESP32 GPIO | PIC Pin | Transistor | Relay | Function
// -----------|---------|------------|-------|------------------
//     5      |   24    |    Q13     |  K1   | 0.05 uH inductor
//     6      |    4    |    Q15     |  K4   | 0.1 uH inductor
//     7      |   25    |    Q14     |  K6   | 0.22 uH inductor
//     8      |    5    |    Q12     |  K8   | 0.45 uH inductor
//     9      |   26    |    Q10     |  K10  | 1.0 uH inductor
//    10      |    7    |    Q11     |  K12  | 2.2 uH inductor
//    11      |    6    |    Q9      |  K14  | 4.7 uH inductor
//    12      |   18    |    Q6      |  K3   | 10 pF capacitor
//    13      |   14    |    Q8      |  K5   | 22 pF capacitor
//    14      |   17    |    Q7      |  K7   | 47 pF capacitor
//    15      |   13    |    Q5      |  K9   | 100 pF capacitor
//    16      |   16    |    Q3      |  K11  | 220 pF capacitor
//    17      |   12    |    Q4      |  K13  | 470 pF capacitor
//    18      |   15    |    Q2      |  K15  | 1 nF capacitor
//    48      |   11    |    Q1      |  K2   | I/O topology relay
//
// ADC inputs (directly to PIC socket analog pins). Both MUST be ADC1 channels
// (GPIO 1-10 on the S3) because ADC2 is unavailable whenever Wi-Fi is active.
// -------------------------------------------------------
// ESP32 GPIO | PIC Pin | Function
// -----------|---------|---------------------------
//     2      |    3    | Forward power (ADC1_CH1)
//     1      |    2    | Reverse power (ADC1_CH0)
//
// Additional ESP32 pins (directly to GPIO, no PIC equivalent):
// -------------------------------------------------------
//     0      | -       | Tune button   (BOOT strapping pin - see hardware.md)
//     3      | -       | Bypass button (JTAG source strapping pin)
//     4      | -       | Auto button
//    39      | -       | CAT RX  (JTAG MTCK - free unless a JTAG probe is used)
//    40      | -       | CAT TX  (JTAG MTDO)
//    41      | -       | TX Request output     (JTAG MTDI)
//    42      | -       | TX Request inverted   (JTAG MTMS)
//    47      | -       | I2C SDA (display + optional temperature sensor)
//    21      | -       | I2C SCL
//
// Antenna selector outputs are disabled by default. GPIO 35/36/37 are the
// usual candidates, but they are consumed by octal PSRAM on -N8R8/-N16R8
// modules; only enable them on a module without octal PSRAM.
//
// Set any unused pin to -1 to disable that function.

constexpr Pins kPins = {
    // L relays - drive MJD122 transistor bases
    // Sorted by inductance: smallest (0.05uH) to largest (4.7uH)
    .lRelays = {
        5,   // GPIO5  -> Q13 -> K1  -> 0.05uH
        6,   // GPIO6  -> Q15 -> K4  -> 0.1uH
        7,   // GPIO7  -> Q14 -> K6  -> 0.22uH
        8,   // GPIO8  -> Q12 -> K8  -> 0.45uH
        9,   // GPIO9  -> Q10 -> K10 -> 1.0uH
        10,  // GPIO10 -> Q11 -> K12 -> 2.2uH
        11   // GPIO11 -> Q9  -> K14 -> 4.7uH
    },

    // C relays - drive MJD122 transistor bases
    // Sorted by capacitance: smallest (10pF) to largest (1nF)
    .cRelays = {
        12,  // GPIO12 -> Q6  -> K3  -> 10pF
        13,  // GPIO13 -> Q8  -> K5  -> 22pF
        14,  // GPIO14 -> Q7  -> K7  -> 47pF
        15,  // GPIO15 -> Q5  -> K9  -> 100pF
        16,  // GPIO16 -> Q3  -> K11 -> 220pF
        17,  // GPIO17 -> Q4  -> K13 -> 470pF
        18   // GPIO18 -> Q2  -> K15 -> 1nF
    },

    // Topology relay (Hi-Z / Lo-Z) - labeled "I/O" on schematic
    // GPIO48 -> Q1 -> K2 -> I/O
    //
    // K2 is the L-network TOPOLOGY relay, NOT a bypass relay.
    //   - OFF (Lo-Z mode): Capacitors on OUTPUT side - for low impedance loads
    //   - ON  (Hi-Z mode): Capacitors on INPUT side  - for high impedance loads
    //
    // Bypass is implemented in SOFTWARE by turning all L and C relays OFF.
    //
    // NOTE: GPIO48 is the on-board addressable RGB LED on ESP32-S3-DevKitC-1
    // *v1.0*. On v1.1 and later the LED moved to GPIO38 and GPIO48 is free.
    // On a v1.0 board either cut the LED trace or move this relay.
    .topologyRelay = 48,

    // SWR bridge ADC inputs - ADC1 only (GPIO 1-10)
    .fwdAdc = 2,   // Forward power
    .revAdc = 1,   // Reverse power

    // Optional NTC thermistor input. Must also be an ADC1 pin, so it is only
    // usable if you free one up. -1 = use the I2C temperature sensor instead.
    .ntcAdc = -1,

    // User buttons (internal pull-up / pull-down selected from settings)
    .tuneButton   = 0,   // Short = tune, long = force tune, press during tune = abort
    .bypassButton = 3,   // Bypass toggle
    .autoButton   = 4,   // Auto-tune toggle

    // CAT interface UART1 (optional)
    .catRx = 39,  // RX from radio CAT TX
    .catTx = 40,  // TX to radio CAT RX (optional)

    // TX request outputs (active during tune cycle)
    .txReq    = 41,  // Connect to radio's TX inhibit if available
    .txReqInv = 42,  // Inverted output (-1 to disable)

    // I2C - shared by the display and the optional temperature sensor
    .i2cSda = 47,
    .i2cScl = 21,

    // Status LED - disabled by default
    .ledStatus = -1,

    // Antenna selector outputs (binary coded, LSB first). Disabled by default.
    .antenna = {-1, -1, -1},
};

// =============================================================================
// Factory defaults for the runtime Settings block
// =============================================================================
//
// SWR bridge calibration notes
// ----------------------------
// The original PIC firmware used calibration constants 0x03B6 (950) forward
// and 0x0557 (1367) reverse with a 10-bit ADC and FVR auto-ranging. The ESP32
// uses a 12-bit ADC with 11dB attenuation (0-2.6V linear range), so those
// constants are reference only.
//
// Power formula:  P = ((Vfwd_mV - fwdOffsetMv) * fwdScale / 1000)^2 / powerScale
//
// Calibrate with the built-in wizard rather than by hand:
//   1. 50 ohm dummy load, key a steady carrier at a known power
//   2. `cal fwd <watts>`   - solves powerScale for you
//   3. `cal rev`           - zeroes the reverse channel into a matched load
//
constexpr uint16_t kPicCalForward = 950;    // 0x03B6, reference only
constexpr uint16_t kPicCalReverse = 1367;   // 0x0557, reference only

// Hard ceilings the runtime settings are clamped to, so a bad `set` cannot
// disable protection entirely.
constexpr float kAbsMaxPowerLimitW = 2000.0f;
constexpr float kAbsMaxTempLimitC = 120.0f;

}  // namespace atu
