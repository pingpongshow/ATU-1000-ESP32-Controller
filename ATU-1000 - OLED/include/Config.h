#pragma once

#include <Arduino.h>

namespace atu {

constexpr uint8_t kRelayCount = 7;

struct Pins {
  int lRelays[kRelayCount];
  int cRelays[kRelayCount];
  int topologyRelay;
  int fwdAdc;
  int revAdc;
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
};

struct TuningConfig {
  // Serial/CAT
  uint32_t catBaud;
  uint32_t catTimeoutMs;

  // Timing
  uint32_t sensorUpdateMs;
  uint32_t displayUpdateMs;
  uint32_t relaySettleMs;
  uint32_t retuneHoldoffMs;
  uint32_t txRequestLeadMs;
  uint32_t txRequestTrailMs;

  // Power thresholds
  float minTunePowerW;
  float minAutoRetunePowerW;
  float maxTunePowerW;

  // SWR thresholds
  float targetSWR;
  float goodSWR;
  float autoRetuneSWR;
  float maxSWR;

  // Tuning algorithm
  uint8_t measureSamples;
  uint16_t measureSampleSpacingMs;
  uint8_t coarseSearchPasses;
  uint8_t fineSearchPasses;
  uint16_t fullSearchMaxSteps;

  // Memory
  uint32_t memoryBinHz;
  uint16_t maxMemoryEntries;

  // OLED Display (128x64 SSD1309)
  uint8_t oledAddress;  // 7-bit I2C address (0x3D for 8-bit 0x7A)
  uint8_t oledWidth;
  uint8_t oledHeight;

  // Hardware config
  bool relayActiveHigh;
  bool requestTxActiveHigh;
  bool buttonsActiveLow;

  // Feature enables
  bool enableCatByDefault;
  bool autoTuneUnknownCatFrequency;
  bool enableAutoTuneByDefault;
  bool enableBypassOnBoot;

  // SWR bridge calibration (from PIC firmware function_021)
  // PIC uses 10-bit ADC with FVR auto-ranging (1.024V, 2.048V, 4.096V)
  // ESP32 uses 12-bit ADC with 11dB attenuation (~3.1V max)
  // These values scale the raw ADC to calibrated mV
  float fwdOffsetMv;
  float revOffsetMv;
  float fwdScale;
  float revScale;
  float powerScale;
  float swrMinForward;

  // Original PIC calibration constants (for reference)
  // From firmware: 0x03B6 = 950, 0x0557 = 1367
  uint16_t picCalForward;   // 950 - scaling factor for forward power
  uint16_t picCalReverse;   // 1367 - scaling factor for reverse power
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

// Total L range: 0.05 to 8.72 uH (all on: 0.05+0.1+0.22+0.45+1.0+2.2+4.7)
// Total C range: 10 to 1869 pF (all on: 10+22+47+100+220+470+1000)

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
// ADC inputs (directly to PIC socket analog pins):
// -------------------------------------------------------
// ESP32 GPIO | PIC Pin | Function
// -----------|---------|---------------------------
//     1      |    2    | Forward power (AN0)
//     2      |    3    | Reverse power (AN1)
//
// Additional ESP32 pins (directly to GPIO, no PIC equivalent):
// -------------------------------------------------------
//     0      | -       | Tune button (strapping pin, safe for input)
//     3      | -       | Bypass button (strapping pin, safe for input)
//     4      | -       | Auto button
//    39      | -       | CAT RX (USB OTG pin, safe after boot)
//    40      | -       | CAT TX (USB OTG pin, safe after boot)
//    41      | -       | TX Request output
//    42      | -       | TX Request inverted output
//    47      | -       | I2C SDA for LCD
//    21      | -       | I2C SCL for LCD
//
// Set any unused pin to -1 to disable that function.

constexpr Pins kPins = {
    // L relays - drive MJD122 transistor bases
    // Sorted by inductance: smallest (0.05uH) to largest (4.7uH)
    // GPIO -> Transistor -> Relay -> Value
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
    // GPIO -> Transistor -> Relay -> Value
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
    // It switches the position of the capacitor bank in the L-network:
    //   - OFF (Lo-Z mode): Capacitors on OUTPUT side - for low impedance loads
    //   - ON  (Hi-Z mode): Capacitors on INPUT side  - for high impedance loads
    //
    // Bypass mode is implemented in SOFTWARE by turning all L and C relays OFF,
    // which creates a direct path through the tuner (no physical bypass relay).
    .topologyRelay = 48,

    // SWR bridge ADC inputs - must be ADC1 channels on ESP32-S3
    // GPIO 1 = ADC1_CH0, GPIO 2 = ADC1_CH1
    // NOTE: Swapped from original PIC pinout to match actual bridge wiring
    .fwdAdc = 2,   // Forward power (was REV on PIC)
    .revAdc = 1,   // Reverse power (was FWD on PIC)

    // User buttons (directly to GPIO, internal pull-up)
    // Using strapping pins - safe for inputs, buttons open at boot
    .tuneButton   = 0,   // Strapping pin - Tune (short=normal, long=force)
    .bypassButton = 3,   // Strapping pin - Bypass toggle
    .autoButton   = 4,   // Weak pullup on reset - Auto-tune toggle

    // CAT interface UART1 (optional)
    // Using USB OTG pins (safe after boot)
    .catRx = 39,  // RX from radio CAT TX
    .catTx = 40,  // TX to radio CAT RX (optional)

    // TX request outputs (active during tune cycle)
    // Using USB OTG pins (safe after boot)
    .txReq    = 41,  // Connect to radio's TX inhibit if available
    .txReqInv = 42,  // Inverted output (-1 to disable)

    // I2C for OLED display
    .i2cSda = 47,
    .i2cScl = 21,

    // Status LED - disabled (use RGB LED on GPIO 38 separately if needed)
    .ledStatus = -1,
};

// =============================================================================
// Tuning Configuration
// =============================================================================
//
// SWR Bridge Calibration Notes:
// -----------------------------
// The PIC firmware uses calibration constants:
//   Forward: 0x03B6 = 950
//   Reverse: 0x0557 = 1367
//
// The PIC ADC is 10-bit with FVR auto-ranging (1.024V, 2.048V, 4.096V).
// The ESP32 ADC is 12-bit with 11dB attenuation (0-2.6V linear range).
//
// SWR bridge typically uses:
//   - Tandem match or Stockton bridge
//   - Schottky diode detectors (BAT43, 1N5711)
//   - ~100mV per watt typical sensitivity
//
// Calibration procedure:
//   1. Connect 50 ohm dummy load
//   2. Transmit at known power (e.g., 5W, 10W)
//   3. Use 'raw' command to see ADC values
//   4. Adjust fwdScale until power reads correctly
//   5. Verify SWR reads ~1.0 into 50 ohms
//
// Power formula: P = (Vfwd * fwdScale)^2 * powerScale

constexpr TuningConfig kCfg = {
    // Serial/CAT
    .catBaud = 9600,
    .catTimeoutMs = 5000,

    // Timing
    .sensorUpdateMs = 100,
    .displayUpdateMs = 200,
    .relaySettleMs = 15,
    .retuneHoldoffMs = 2000,
    .txRequestLeadMs = 50,
    .txRequestTrailMs = 50,

    // Power thresholds (Watts)
    .minTunePowerW = 1.0f,
    .minAutoRetunePowerW = 3.0f,
    .maxTunePowerW = 50.0f,

    // SWR thresholds
    .targetSWR = 1.2f,
    .goodSWR = 1.5f,
    .autoRetuneSWR = 2.5f,
    .maxSWR = 10.0f,

    // Tuning algorithm
    .measureSamples = 4,
    .measureSampleSpacingMs = 3,
    .coarseSearchPasses = 2,
    .fineSearchPasses = 3,
    .fullSearchMaxSteps = 512,

    // Frequency memory
    .memoryBinHz = 25000,       // 25 kHz bins
    .maxMemoryEntries = 256,

    // OLED configuration (2.42" 128x64 SSD1309)
    .oledAddress = 0x3C,        // 7-bit address (confirmed via i2cscan)
    .oledWidth = 128,
    .oledHeight = 64,

    // Hardware configuration
    .relayActiveHigh = true,    // HIGH turns relay ON
    .requestTxActiveHigh = true,
    .buttonsActiveLow = true,   // Buttons connect to GND when pressed

    // Feature enables
    .enableCatByDefault = true,
    .autoTuneUnknownCatFrequency = true,
    .enableAutoTuneByDefault = true,
    .enableBypassOnBoot = false,

    // SWR bridge calibration
    // Derived from PIC firmware constants and typical bridge behavior
    // Adjust these based on your actual bridge circuit
    .fwdOffsetMv = 15.0f,       // Diode forward voltage drop
    .revOffsetMv = 15.0f,
    .fwdScale = 1.0f,           // Voltage scaling (adjust for your bridge)
    .revScale = 1.0f,
    .powerScale = 0.03f,        // V^2 to Watts - calibrated at 25W for this SWR bridge
    .swrMinForward = 30.0f,     // Min mV for valid SWR (reject noise)

    // Original PIC calibration (reference)
    .picCalForward = 950,       // 0x03B6 from firmware function_021
    .picCalReverse = 1367,      // 0x0557 from firmware function_021
};

// =============================================================================
// Helper Functions
// =============================================================================

inline const BandInfo* findBand(uint32_t freqHz) {
  for (size_t i = 0; i < kBandCount; ++i) {
    if (freqHz >= kBands[i].lowHz && freqHz <= kBands[i].highHz) {
      return &kBands[i];
    }
  }
  return nullptr;
}

}  // namespace atu
