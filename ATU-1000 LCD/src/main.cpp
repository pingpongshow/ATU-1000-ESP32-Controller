// =============================================================================
// ESP32-S3 ATU Controller Firmware
// Complete replacement for PIC16F1938-based ATU board
// =============================================================================

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <Wire.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "Config.h"

namespace {

using atu::kBandCount;
using atu::kBands;
using atu::kCfg;
using atu::kCValuesPf;
using atu::kLValuesUh;
using atu::kPins;
using atu::kRelayCount;

// =============================================================================
// Data Structures
// =============================================================================

struct RelayState {
  uint8_t lMask = 0;
  uint8_t cMask = 0;
  bool topology = false;  // false = Lo-Z, true = Hi-Z
  bool bypass = false;
};

struct SensorReading {
  float fwdMv = 0.0f;
  float revMv = 0.0f;
  float fwdRaw = 0.0f;
  float revRaw = 0.0f;
  float swr = 99.0f;
  float powerW = 0.0f;
  bool valid = false;
};

struct MemoryEntry {
  uint32_t bin = 0;
  uint8_t lMask = 0;
  uint8_t cMask = 0;
  uint8_t flags = 0;
  uint16_t swrX100 = 0;
  uint16_t useCount = 0;
  uint32_t lastSeenSec = 0;
} __attribute__((packed));

struct RuntimeState {
  uint8_t version = 1;
  uint8_t lMask = 0;
  uint8_t cMask = 0;
  uint8_t flags = 0;
  uint8_t autoTune = 1;
  uint32_t checksum = 0;
} __attribute__((packed));

// SWR sweep data point
struct SweepPoint {
  uint32_t freqHz;
  float swr;
  float powerW;
};

enum RelayFlag : uint8_t {
  FLAG_TOPOLOGY = 0x01,
  FLAG_BYPASS = 0x02,
};

enum class TuneResult : uint8_t {
  Success,
  GoodEnough,
  NoPower,
  PowerHigh,
  PowerOverload,
  Failed,
  Aborted,
  InProgress,
};

enum class CatProtocol : uint8_t {
  None,
  Kenwood,    // Kenwood/Elecraft/FlexRadio ASCII
  Icom,       // Icom CI-V binary
  YaesuOld,   // Yaesu older 5-byte binary (FT-817, etc)
  YaesuNew,   // Yaesu newer ASCII (FTDX101, etc)
  Auto,       // Auto-detect
};

// =============================================================================
// Constants
// =============================================================================

// Power limit protection
constexpr float kPowerLimitW = 1000.0f;       // Maximum power before protection
constexpr float kPowerWarningW = 800.0f;      // Warning threshold
constexpr uint32_t kPowerOverloadHoldoffMs = 5000;  // Cooldown after overload

// Icom CI-V constants
constexpr uint8_t kIcomPreamble = 0xFE;
constexpr uint8_t kIcomEOM = 0xFD;
constexpr uint8_t kIcomDefaultAddr = 0x00;    // Broadcast address
constexpr uint8_t kIcomControllerAddr = 0xE0; // Our address

// Yaesu constants
constexpr uint8_t kYaesuBlockSize = 5;

// =============================================================================
// Utility Functions
// =============================================================================

float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return ~crc;
}

uint8_t popcount8(uint8_t v) {
  uint8_t c = 0;
  while (v) { c += v & 1; v >>= 1; }
  return c;
}

// BCD conversion for Icom CI-V
uint32_t bcdToHz(const uint8_t* bcd, uint8_t len) {
  uint32_t hz = 0;
  uint32_t mult = 1;
  for (int i = 0; i < len; ++i) {
    hz += (bcd[i] & 0x0F) * mult;
    mult *= 10;
    hz += ((bcd[i] >> 4) & 0x0F) * mult;
    mult *= 10;
  }
  return hz;
}

void hzToBcd(uint32_t hz, uint8_t* bcd, uint8_t len) {
  for (int i = 0; i < len; ++i) {
    bcd[i] = (hz % 10);
    hz /= 10;
    bcd[i] |= (hz % 10) << 4;
    hz /= 10;
  }
}

// =============================================================================
// Relay Controller
// =============================================================================

class RelayController {
 public:
  void begin() {
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      setupOutput(kPins.lRelays[i]);
      setupOutput(kPins.cRelays[i]);
    }
    setupOutput(kPins.topologyRelay);
    apply(RelayState{});
  }

  void apply(const RelayState& state) {
    current_ = state;
    uint8_t lMask = state.bypass ? 0 : state.lMask;
    uint8_t cMask = state.bypass ? 0 : state.cMask;

    for (uint8_t i = 0; i < kRelayCount; ++i) {
      writeRelay(kPins.lRelays[i], (lMask >> i) & 0x01);
      writeRelay(kPins.cRelays[i], (cMask >> i) & 0x01);
    }
    writeRelay(kPins.topologyRelay, state.topology);
  }

  void applyWithSettle(const RelayState& state) {
    apply(state);
    delay(kCfg.relaySettleMs);
  }

  RelayState current() const { return current_; }

  float totalL() const {
    float sum = 0.0f;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      if ((current_.lMask >> i) & 0x01) sum += kLValuesUh[i];
    }
    return sum;
  }

  uint16_t totalC() const {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < kRelayCount; ++i) {
      if ((current_.cMask >> i) & 0x01) sum += kCValuesPf[i];
    }
    return sum;
  }

 private:
  RelayState current_{};

  static void setupOutput(int pin) {
    if (pin < 0) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, kCfg.relayActiveHigh ? LOW : HIGH);
  }

  static void writeRelay(int pin, bool on) {
    if (pin < 0) return;
    bool level = kCfg.relayActiveHigh ? on : !on;
    digitalWrite(pin, level ? HIGH : LOW);
  }
};

// =============================================================================
// SWR Bridge Sensor
// =============================================================================

class BridgeSensor {
 public:
  void begin() {
    if (kPins.fwdAdc >= 0) analogSetPinAttenuation(kPins.fwdAdc, ADC_11db);
    if (kPins.revAdc >= 0) analogSetPinAttenuation(kPins.revAdc, ADC_11db);
    analogReadResolution(12);
  }

  SensorReading readOnce() const {
    SensorReading out;
    if (kPins.fwdAdc >= 0) {
      out.fwdRaw = static_cast<float>(analogRead(kPins.fwdAdc));
      out.fwdMv = static_cast<float>(analogReadMilliVolts(kPins.fwdAdc));
    }
    if (kPins.revAdc >= 0) {
      out.revRaw = static_cast<float>(analogRead(kPins.revAdc));
      out.revMv = static_cast<float>(analogReadMilliVolts(kPins.revAdc));
    }
    calculateSwr(out);
    return out;
  }

  SensorReading readAverage(uint8_t samples, uint16_t spacingMs) const {
    if (samples == 0) return SensorReading{};

    float fwdSum = 0, revSum = 0, fwdRawSum = 0, revRawSum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
      SensorReading now = readOnce();
      fwdSum += now.fwdMv;
      revSum += now.revMv;
      fwdRawSum += now.fwdRaw;
      revRawSum += now.revRaw;
      if (spacingMs > 0 && i < samples - 1) delay(spacingMs);
    }

    SensorReading out;
    out.fwdMv = fwdSum / samples;
    out.revMv = revSum / samples;
    out.fwdRaw = fwdRawSum / samples;
    out.revRaw = revRawSum / samples;
    calculateSwr(out);
    return out;
  }

 private:
  static void calculateSwr(SensorReading& r) {
    float vf = std::max(0.0f, (r.fwdMv - kCfg.fwdOffsetMv) * kCfg.fwdScale);
    float vr = std::max(0.0f, (r.revMv - kCfg.revOffsetMv) * kCfg.revScale);

    if (vf < kCfg.swrMinForward) {
      r.swr = 99.0f;
      r.powerW = 0.0f;
      r.valid = false;
      return;
    }

    r.valid = true;
    float gamma = clampf(vr / vf, 0.0f, 0.99f);
    r.swr = (1.0f + gamma) / (1.0f - gamma);
    r.swr = clampf(r.swr, 1.0f, 99.0f);

    float vfV = vf / 1000.0f;
    r.powerW = std::max(0.0f, vfV * vfV / kCfg.powerScale);
  }
};

// =============================================================================
// CAT Interface - Multi-Protocol Support
// =============================================================================

class CatInterface {
 public:
  void begin() {
    if (kPins.catRx < 0) {
      enabled_ = false;
      return;
    }
    serial_.begin(kCfg.catBaud, SERIAL_8N1, kPins.catRx, kPins.catTx);
    enabled_ = true;
    protocol_ = CatProtocol::Auto;
  }

  void setProtocol(CatProtocol p) { protocol_ = p; }
  CatProtocol getProtocol() const { return detectedProtocol_; }

  void loop() {
    if (!enabled_) return;

    while (serial_.available() > 0) {
      uint8_t b = serial_.read();

      // Add to buffer
      if (bufIdx_ < sizeof(buffer_) - 1) {
        buffer_[bufIdx_++] = b;
      }

      // Try to parse based on protocol
      bool parsed = false;

      if (protocol_ == CatProtocol::Auto || protocol_ == CatProtocol::Icom) {
        parsed = tryParseIcom();
      }
      if (!parsed && (protocol_ == CatProtocol::Auto || protocol_ == CatProtocol::YaesuOld)) {
        parsed = tryParseYaesuOld();
      }
      if (!parsed && (protocol_ == CatProtocol::Auto || protocol_ == CatProtocol::Kenwood ||
                      protocol_ == CatProtocol::YaesuNew)) {
        parsed = tryParseKenwood();
      }

      // Buffer management
      if (bufIdx_ >= sizeof(buffer_) - 1) {
        // Buffer full, shift out old data
        memmove(buffer_, buffer_ + 32, bufIdx_ - 32);
        bufIdx_ -= 32;
      }
    }
  }

  bool connected() const {
    return enabled_ && (millis() - lastRxMs_ <= kCfg.catTimeoutMs);
  }

  bool takeUpdate(uint32_t& freqHz) {
    if (!updated_) return false;
    updated_ = false;
    freqHz = lastFreqHz_;
    return true;
  }

  uint32_t lastFrequency() const { return lastFreqHz_; }

  // Send frequency command (for sweep mode)
  bool setFrequency(uint32_t hz) {
    if (!enabled_ || kPins.catTx < 0) return false;

    switch (detectedProtocol_) {
      case CatProtocol::Kenwood:
      case CatProtocol::YaesuNew:
        return sendKenwoodFreq(hz);
      case CatProtocol::Icom:
        return sendIcomFreq(hz);
      case CatProtocol::YaesuOld:
        return sendYaesuOldFreq(hz);
      default:
        return false;
    }
  }

  const char* protocolName() const {
    switch (detectedProtocol_) {
      case CatProtocol::Kenwood: return "Kenwood";
      case CatProtocol::Icom: return "Icom";
      case CatProtocol::YaesuOld: return "Yaesu";
      case CatProtocol::YaesuNew: return "YaesuN";
      default: return "Auto";
    }
  }

 private:
  HardwareSerial serial_{1};
  bool enabled_ = false;
  bool updated_ = false;
  uint32_t lastFreqHz_ = 0;
  uint32_t lastRxMs_ = 0;
  CatProtocol protocol_ = CatProtocol::Auto;
  CatProtocol detectedProtocol_ = CatProtocol::None;
  uint8_t buffer_[128]{};
  size_t bufIdx_ = 0;
  uint8_t icomRadioAddr_ = 0x00;

  void freqUpdated(uint32_t hz, CatProtocol proto) {
    if (hz >= 1000000 && hz <= 60000000) {
      lastFreqHz_ = hz;
      lastRxMs_ = millis();
      updated_ = true;
      detectedProtocol_ = proto;
    }
  }

  // ---- Kenwood/Elecraft ASCII Protocol ----
  bool tryParseKenwood() {
    // Look for FA or IF command ending with ;
    for (size_t i = 0; i < bufIdx_; ++i) {
      if (buffer_[i] == ';') {
        // Found end of command
        buffer_[i] = '\0';
        char* cmd = reinterpret_cast<char*>(buffer_);

        // FA command: FAxxxxxxxxxxx;
        const char* fa = strstr(cmd, "FA");
        if (fa) {
          uint32_t hz = parseAsciiFreq(fa + 2);
          if (hz > 0) {
            freqUpdated(hz, CatProtocol::Kenwood);
            bufIdx_ = 0;
            return true;
          }
        }

        // IF command: IF...freq...;
        const char* ifc = strstr(cmd, "IF");
        if (ifc && strlen(ifc) >= 13) {
          uint32_t hz = parseAsciiFreq(ifc + 2);
          if (hz > 0) {
            freqUpdated(hz, CatProtocol::Kenwood);
            bufIdx_ = 0;
            return true;
          }
        }

        // Remove parsed portion
        memmove(buffer_, buffer_ + i + 1, bufIdx_ - i - 1);
        bufIdx_ -= (i + 1);
        return false;
      }
    }
    return false;
  }

  uint32_t parseAsciiFreq(const char* p) {
    uint64_t val = 0;
    uint8_t digits = 0;
    while (isdigit(*p) && digits < 12) {
      val = val * 10ULL + (*p - '0');
      ++digits;
      ++p;
    }
    if (digits >= 7 && val >= 1000000 && val <= 60000000) {
      return static_cast<uint32_t>(val);
    }
    return 0;
  }

  bool sendKenwoodFreq(uint32_t hz) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "FA%011lu;", (unsigned long)hz);
    serial_.print(cmd);
    return true;
  }

  // ---- Icom CI-V Binary Protocol ----
  bool tryParseIcom() {
    // Look for FE FE ... FD pattern
    for (size_t i = 0; i + 10 < bufIdx_; ++i) {
      if (buffer_[i] == kIcomPreamble && buffer_[i+1] == kIcomPreamble) {
        // Found preamble, look for EOM
        for (size_t j = i + 4; j < bufIdx_; ++j) {
          if (buffer_[j] == kIcomEOM) {
            // Complete CI-V frame
            uint8_t toAddr = buffer_[i+2];
            uint8_t fromAddr = buffer_[i+3];
            uint8_t cmd = buffer_[i+4];

            // Store radio address for responses
            if (fromAddr != kIcomControllerAddr) {
              icomRadioAddr_ = fromAddr;
            }

            // Command 0x00 or 0x03 = frequency data
            // Frequency is 5 bytes BCD after command
            if ((cmd == 0x00 || cmd == 0x03) && j - i >= 10) {
              uint32_t hz = bcdToHz(&buffer_[i+5], 5);
              if (hz > 0) {
                freqUpdated(hz, CatProtocol::Icom);
              }
            }

            // Remove parsed frame
            memmove(buffer_, buffer_ + j + 1, bufIdx_ - j - 1);
            bufIdx_ -= (j + 1);
            return true;
          }
        }
      }
    }
    return false;
  }

  bool sendIcomFreq(uint32_t hz) {
    uint8_t cmd[13];
    cmd[0] = kIcomPreamble;
    cmd[1] = kIcomPreamble;
    cmd[2] = icomRadioAddr_;        // To radio
    cmd[3] = kIcomControllerAddr;   // From us
    cmd[4] = 0x05;                  // Set frequency command
    hzToBcd(hz, &cmd[5], 5);        // 5 bytes BCD
    cmd[10] = kIcomEOM;
    serial_.write(cmd, 11);
    return true;
  }

  // ---- Yaesu Older Binary Protocol (FT-817, FT-857, FT-897, etc) ----
  bool tryParseYaesuOld() {
    // Yaesu uses 5-byte blocks, frequency response after 0x03 command
    // Format: 4 bytes frequency (BCD, MSB first), 1 byte mode/status
    if (bufIdx_ >= 5) {
      // Check if looks like frequency data (reasonable values)
      uint32_t hz = 0;
      hz += ((buffer_[0] >> 4) & 0x0F) * 10000000;
      hz += (buffer_[0] & 0x0F) * 1000000;
      hz += ((buffer_[1] >> 4) & 0x0F) * 100000;
      hz += (buffer_[1] & 0x0F) * 10000;
      hz += ((buffer_[2] >> 4) & 0x0F) * 1000;
      hz += (buffer_[2] & 0x0F) * 100;
      hz += ((buffer_[3] >> 4) & 0x0F) * 10;
      hz += (buffer_[3] & 0x0F);
      hz *= 10;  // Yaesu sends in 10Hz units

      if (hz >= 1000000 && hz <= 60000000) {
        freqUpdated(hz, CatProtocol::YaesuOld);
        memmove(buffer_, buffer_ + 5, bufIdx_ - 5);
        bufIdx_ -= 5;
        return true;
      }

      // Not valid, shift buffer
      memmove(buffer_, buffer_ + 1, bufIdx_ - 1);
      bufIdx_--;
    }
    return false;
  }

  bool sendYaesuOldFreq(uint32_t hz) {
    // Yaesu set frequency: freq(4 bytes BCD) + 0x01
    uint8_t cmd[5];
    hz /= 10;  // Convert to 10Hz units
    cmd[0] = ((hz / 10000000) << 4) | ((hz / 1000000) % 10);
    cmd[1] = (((hz / 100000) % 10) << 4) | ((hz / 10000) % 10);
    cmd[2] = (((hz / 1000) % 10) << 4) | ((hz / 100) % 10);
    cmd[3] = (((hz / 10) % 10) << 4) | (hz % 10);
    cmd[4] = 0x01;  // Set frequency command
    serial_.write(cmd, 5);
    return true;
  }
};

// =============================================================================
// Frequency Memory Store
// =============================================================================

class MemoryStore {
 public:
  void begin() {
    prefs_.begin("atu_mem", false);
    load();
  }

  void loop() {
    if (dirty_ && (millis() - dirtyMs_ > 2000)) save();
  }

  bool lookup(uint32_t freqHz, RelayState& out, uint16_t* swrX100 = nullptr) {
    if (freqHz == 0 || entries_.empty()) return false;

    uint32_t bin = binFor(freqHz);
    int bestIdx = -1;
    uint32_t bestDist = UINT32_MAX;

    for (size_t i = 0; i < entries_.size(); ++i) {
      uint32_t dist = (entries_[i].bin > bin) ? (entries_[i].bin - bin) : (bin - entries_[i].bin);
      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = static_cast<int>(i);
      }
      if (dist == 0) break;
    }

    if (bestIdx < 0 || bestDist > 1) return false;

    MemoryEntry& e = entries_[bestIdx];
    out.lMask = e.lMask;
    out.cMask = e.cMask;
    out.topology = (e.flags & FLAG_TOPOLOGY) != 0;
    out.bypass = (e.flags & FLAG_BYPASS) != 0;
    if (swrX100) *swrX100 = e.swrX100;
    if (e.useCount < UINT16_MAX) ++e.useCount;
    e.lastSeenSec = millis() / 1000U;
    markDirty();
    return true;
  }

  void upsert(uint32_t freqHz, const RelayState& state, float swr) {
    if (freqHz == 0) return;

    uint32_t bin = binFor(freqHz);
    uint16_t swrX100 = static_cast<uint16_t>(clampf(swr, 1.0f, 99.99f) * 100.0f);

    int idx = findByBin(bin);
    if (idx < 0) {
      if (entries_.size() >= kCfg.maxMemoryEntries) {
        idx = pickEvictionIndex();
      } else {
        entries_.push_back(MemoryEntry{});
        idx = static_cast<int>(entries_.size() - 1);
      }
    }

    MemoryEntry& e = entries_[idx];
    e.bin = bin;
    e.lMask = state.lMask;
    e.cMask = state.cMask;
    e.flags = (state.topology ? FLAG_TOPOLOGY : 0) | (state.bypass ? FLAG_BYPASS : 0);
    e.swrX100 = swrX100;
    if (e.useCount < UINT16_MAX) ++e.useCount;
    e.lastSeenSec = millis() / 1000U;
    markDirty();
  }

  // Get SWR data for sweep display
  bool getSwrForFreq(uint32_t freqHz, float& swr) {
    uint32_t bin = binFor(freqHz);
    int idx = findByBin(bin);
    if (idx < 0) return false;
    swr = entries_[idx].swrX100 / 100.0f;
    return true;
  }

  void clear() {
    entries_.clear();
    prefs_.remove("table");
    dirty_ = false;
  }

  size_t size() const { return entries_.size(); }

 private:
  Preferences prefs_;
  std::vector<MemoryEntry> entries_;
  bool dirty_ = false;
  uint32_t dirtyMs_ = 0;

  void markDirty() { dirty_ = true; dirtyMs_ = millis(); }
  static uint32_t binFor(uint32_t freqHz) { return freqHz / kCfg.memoryBinHz; }

  int findByBin(uint32_t bin) const {
    for (size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].bin == bin) return static_cast<int>(i);
    }
    return -1;
  }

  int pickEvictionIndex() const {
    if (entries_.empty()) return -1;
    int best = 0;
    uint32_t minScore = UINT32_MAX;
    for (size_t i = 0; i < entries_.size(); ++i) {
      uint32_t score = entries_[i].useCount * 10U + (entries_[i].lastSeenSec / 60U);
      if (score < minScore) { minScore = score; best = static_cast<int>(i); }
    }
    return best;
  }

  void load() {
    entries_.clear();
    size_t len = prefs_.getBytesLength("table");
    if (len == 0 || (len % sizeof(MemoryEntry)) != 0) return;
    entries_.resize(len / sizeof(MemoryEntry));
    prefs_.getBytes("table", entries_.data(), len);
  }

  void save() {
    if (entries_.empty()) {
      prefs_.remove("table");
    } else {
      prefs_.putBytes("table", entries_.data(), entries_.size() * sizeof(MemoryEntry));
    }
    dirty_ = false;
  }
};

// =============================================================================
// Display Manager with Bargraph Support
// =============================================================================

class DisplayManager {
 public:
  DisplayManager() : lcd_(kCfg.lcdAddress, kCfg.lcdCols, kCfg.lcdRows) {}

  void begin() {
    if (kPins.i2cSda < 0 || kPins.i2cScl < 0) {
      Serial.println("  Display disabled (no I2C pins)");
      enabled_ = false;
      return;
    }
    Wire.begin(kPins.i2cSda, kPins.i2cScl);

    // Check if LCD is present before initializing (prevents hang)
    Wire.beginTransmission(kCfg.lcdAddress);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
      Serial.printf("  LCD not found at 0x%02X (error %d)\n", kCfg.lcdAddress, error);
      Serial.printf("  I2C pins: SDA=GPIO%d, SCL=GPIO%d\n", kPins.i2cSda, kPins.i2cScl);
      enabled_ = false;
      return;
    }

    Serial.printf("  LCD found at 0x%02X\n", kCfg.lcdAddress);
    lcd_.init();
    lcd_.backlight();
    enabled_ = true;
    createBarChars();
    clear();
  }

  void clear() {
    if (!enabled_) return;
    lcd_.clear();
  }

  void showSplash(const char* version) {
    if (!enabled_) return;
    lcd_.clear();
    lcd_.setCursor(0, 0);
    lcd_.print("ATU-1000 ESP32");
    lcd_.setCursor(0, 1);
    lcd_.print(version);
    lcd_.setCursor(0, 2);
    lcd_.print("Initializing...");
  }

  void render(uint32_t freqHz, bool catConnected, bool autoTuneEnabled,
              const RelayState& state, const SensorReading& reading,
              float totalL, uint16_t totalC, const char* status,
              bool powerWarning, bool powerOverload) {
    if (!enabled_) return;

    // Line 0: CAT/freq, auto mode
    writeLine(0, formatLine0(freqHz, catConnected, autoTuneEnabled));

    // Line 1: SWR and Power with bargraph
    writeLine(1, formatLine1WithBar(reading, powerWarning, powerOverload));

    // Line 2: L/C values and topology
    writeLine(2, formatLine2(state, totalL, totalC));

    // Line 3: Status
    writeLine(3, fit(status));
  }

  void showSweepGraph(const std::vector<SweepPoint>& points, uint32_t startHz, uint32_t endHz) {
    if (!enabled_ || points.empty()) return;

    lcd_.clear();

    // Line 0: Title
    char buf[20];
    snprintf(buf, sizeof(buf), "SWR %.1f-%.1fMHz", startHz/1e6, endHz/1e6);
    writeLine(0, fit(buf));

    // Lines 1-3: Bargraph of SWR (3 rows x 16 cols = 48 data points max)
    // Map SWR 1.0-3.0 to 0-8 character height (each char has 8 rows)

    float minSwr = 99.0f, maxSwr = 1.0f;
    for (const auto& p : points) {
      if (p.swr < minSwr) minSwr = p.swr;
      if (p.swr > maxSwr) maxSwr = p.swr;
    }

    // Normalize to 16 columns
    for (uint8_t col = 0; col < 16 && col < points.size(); ++col) {
      size_t idx = (points.size() * col) / 16;
      float swr = points[idx].swr;

      // Map SWR to 0-24 (3 rows * 8 pixels)
      int height = static_cast<int>((swr - 1.0f) / 2.0f * 24);
      height = constrain(height, 0, 24);

      // Draw from bottom up
      for (int row = 3; row >= 1; --row) {
        int rowHeight = height - (3 - row) * 8;
        rowHeight = constrain(rowHeight, 0, 8);
        lcd_.setCursor(col, row);
        if (rowHeight == 0) {
          lcd_.write(' ');
        } else if (rowHeight >= 8) {
          lcd_.write(0xFF);  // Full block
        } else {
          lcd_.write(rowHeight - 1);  // Custom char 0-7
        }
      }
    }
  }

  void showOverloadWarning() {
    if (!enabled_) return;
    lcd_.clear();
    lcd_.setCursor(0, 0);
    lcd_.print("!!! OVERLOAD !!!");
    lcd_.setCursor(0, 1);
    lcd_.print("Power > 1000W");
    lcd_.setCursor(0, 2);
    lcd_.print("BYPASS ENGAGED");
    lcd_.setCursor(0, 3);
    lcd_.print("Reduce power!");
  }

 private:
  LiquidCrystal_I2C lcd_;
  bool enabled_ = false;

  // Custom characters for bargraph (vertical bars of height 1-8)
  void createBarChars() {
    // Characters 0-7 represent bars of height 1-8 pixels from bottom
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t pattern[8];
      for (uint8_t row = 0; row < 8; ++row) {
        pattern[row] = (row >= (7 - i)) ? 0x1F : 0x00;
      }
      lcd_.createChar(i, pattern);
    }
  }

  String fit(const char* in) {
    String out(in);
    while (out.length() < kCfg.lcdCols) out += ' ';
    if (out.length() > kCfg.lcdCols) out = out.substring(0, kCfg.lcdCols);
    return out;
  }

  static String formatFreq(uint32_t hz) {
    char buf[16];
    uint32_t mhz = hz / 1000000UL;
    uint32_t khz = (hz / 1000UL) % 1000UL;
    snprintf(buf, sizeof(buf), "%2lu.%03lu", (unsigned long)mhz, (unsigned long)khz);
    return String(buf);
  }

  String formatLine0(uint32_t freqHz, bool catConnected, bool autoTuneEnabled) {
    String out;
    out += catConnected ? "CAT " : "--- ";
    out += (freqHz > 0) ? formatFreq(freqHz) : "--.---";
    out += autoTuneEnabled ? " A" : " M";
    return fit(out.c_str());
  }

  String formatLine1WithBar(const SensorReading& reading, bool powerWarning, bool powerOverload) {
    char buf[20];

    // SWR value
    if (reading.swr < 10.0f) {
      snprintf(buf, sizeof(buf), "S%.1f", reading.swr);
    } else {
      snprintf(buf, sizeof(buf), "S>10");
    }
    String out(buf);

    // Power bargraph (6 chars wide, max 1000W)
    out += " ";
    float pct = constrain(reading.powerW / kPowerLimitW, 0.0f, 1.0f);
    int bars = static_cast<int>(pct * 30);  // 6 chars * 5 segments each

    for (int i = 0; i < 6; ++i) {
      int segBars = bars - (i * 5);
      if (segBars <= 0) {
        out += ' ';
      } else if (segBars >= 5) {
        out += char(0xFF);  // Full block
      } else {
        // Use custom chars 1-4 (avoid char(0) which is null terminator)
        out += char(segBars);  // Partial bar (custom char 1-4)
      }
    }

    // Warning/overload indicator
    if (powerOverload) {
      out += "!OVL!";
    } else if (powerWarning) {
      out += " WARN";
    } else {
      snprintf(buf, sizeof(buf), "%4dW", (int)reading.powerW);
      out += buf;
    }

    return fit(out.c_str());
  }

  String formatLine2(const RelayState& state, float totalL, uint16_t totalC) {
    char buf[20];
    char topo = state.topology ? 'H' : 'L';
    char byp = state.bypass ? 'B' : ' ';
    snprintf(buf, sizeof(buf), "%.1fuH %4upF %c%c", totalL, totalC, topo, byp);
    return fit(buf);
  }

  void writeLine(uint8_t row, const String& text) {
    lcd_.setCursor(0, row);
    lcd_.print(text);
  }
};

// =============================================================================
// Button Handler
// =============================================================================

class DebouncedButton {
 public:
  void begin(int pin) {
    pin_ = pin;
    if (pin_ < 0) { enabled_ = false; return; }
    enabled_ = true;
    pinMode(pin_, INPUT_PULLUP);
    stable_ = readRaw();
    lastRaw_ = stable_;
  }

  void loop() {
    if (!enabled_) { shortEvent_ = longEvent_ = false; return; }
    shortEvent_ = longEvent_ = false;

    bool raw = readRaw();
    if (raw != lastRaw_) { lastRaw_ = raw; lastDebounceMs_ = millis(); }
    if ((millis() - lastDebounceMs_) < 30) return;

    if (raw != stable_) {
      stable_ = raw;
      if (stable_) { pressedMs_ = millis(); longFired_ = false; }
      else {
        uint32_t held = millis() - pressedMs_;
        if (held >= 50 && held < 800 && !longFired_) shortEvent_ = true;
      }
      return;
    }

    if (stable_ && !longFired_ && (millis() - pressedMs_ >= 800)) {
      longFired_ = true;
      longEvent_ = true;
    }
  }

  bool shortPress() const { return shortEvent_; }
  bool longPress() const { return longEvent_; }
  bool isPressed() const { return enabled_ && stable_; }

 private:
  int pin_ = -1;
  bool enabled_ = false;
  bool stable_ = false;
  bool lastRaw_ = false;
  bool shortEvent_ = false;
  bool longEvent_ = false;
  bool longFired_ = false;
  uint32_t lastDebounceMs_ = 0;
  uint32_t pressedMs_ = 0;

  bool readRaw() const {
    bool level = digitalRead(pin_) == LOW;
    return kCfg.buttonsActiveLow ? level : !level;
  }
};

// =============================================================================
// Status LED
// =============================================================================

class StatusLed {
 public:
  void begin() {
    if (kPins.ledStatus < 0) return;
    pinMode(kPins.ledStatus, OUTPUT);
    off();
  }
  void on() { if (kPins.ledStatus >= 0) digitalWrite(kPins.ledStatus, HIGH); }
  void off() { if (kPins.ledStatus >= 0) digitalWrite(kPins.ledStatus, LOW); }
  void blink(uint16_t onMs, uint16_t offMs) {
    uint32_t phase = millis() % (onMs + offMs);
    if (phase < onMs) on(); else off();
  }
};

// =============================================================================
// Power Protection
// =============================================================================

class PowerProtection {
 public:
  void check(float powerW, bool& warning, bool& overload) {
    warning = (powerW >= kPowerWarningW && powerW < kPowerLimitW);

    if (powerW >= kPowerLimitW) {
      if (!overloadActive_) {
        overloadActive_ = true;
        overloadStartMs_ = millis();
        overloadTriggered_ = true;
      }
      overload = true;
    } else if (overloadActive_) {
      // Cooldown period
      if (millis() - overloadStartMs_ >= kPowerOverloadHoldoffMs) {
        overloadActive_ = false;
        overloadTriggered_ = false;
      }
      overload = overloadActive_;
    } else {
      overload = false;
    }
  }

  bool wasTriggered() {
    bool t = overloadTriggered_;
    overloadTriggered_ = false;
    return t;
  }

  bool isOverload() const { return overloadActive_; }
  void reset() { overloadActive_ = false; overloadTriggered_ = false; }

 private:
  bool overloadActive_ = false;
  bool overloadTriggered_ = false;
  uint32_t overloadStartMs_ = 0;
};

// =============================================================================
// SWR Sweep Engine
// =============================================================================

class SweepEngine {
 public:
  SweepEngine(CatInterface& cat, BridgeSensor& sensor, MemoryStore& memory)
      : cat_(cat), sensor_(sensor), memory_(memory) {}

  bool start(uint32_t startHz, uint32_t endHz, uint32_t stepHz, uint16_t dwellMs) {
    if (running_) return false;

    startHz_ = startHz;
    endHz_ = endHz;
    stepHz_ = stepHz;
    dwellMs_ = dwellMs;
    currentHz_ = startHz;
    points_.clear();
    running_ = true;
    lastStepMs_ = 0;

    return true;
  }

  void stop() { running_ = false; }
  bool isRunning() const { return running_; }

  // Call this in loop() - returns true when a new point is measured
  bool step() {
    if (!running_) return false;

    if (millis() - lastStepMs_ < dwellMs_) return false;
    lastStepMs_ = millis();

    // Set frequency via CAT
    cat_.setFrequency(currentHz_);
    delay(50);  // Let radio settle

    // Measure SWR
    SensorReading r = sensor_.readAverage(8, 5);

    SweepPoint p;
    p.freqHz = currentHz_;
    p.swr = r.swr;
    p.powerW = r.powerW;
    points_.push_back(p);

    // Advance
    currentHz_ += stepHz_;
    if (currentHz_ > endHz_) {
      running_ = false;
      return true;  // Final point
    }

    return true;
  }

  const std::vector<SweepPoint>& getPoints() const { return points_; }

  float getProgress() const {
    if (endHz_ <= startHz_) return 1.0f;
    return static_cast<float>(currentHz_ - startHz_) / (endHz_ - startHz_);
  }

  // Find minimum SWR point
  bool findMinSwr(uint32_t& freqHz, float& swr) const {
    if (points_.empty()) return false;

    size_t minIdx = 0;
    float minSwr = 99.0f;
    for (size_t i = 0; i < points_.size(); ++i) {
      if (points_[i].swr < minSwr) {
        minSwr = points_[i].swr;
        minIdx = i;
      }
    }
    freqHz = points_[minIdx].freqHz;
    swr = points_[minIdx].swr;
    return true;
  }

 private:
  CatInterface& cat_;
  BridgeSensor& sensor_;
  MemoryStore& memory_;

  bool running_ = false;
  uint32_t startHz_ = 0;
  uint32_t endHz_ = 0;
  uint32_t stepHz_ = 0;
  uint16_t dwellMs_ = 0;
  uint32_t currentHz_ = 0;
  uint32_t lastStepMs_ = 0;
  std::vector<SweepPoint> points_;
};

// =============================================================================
// Tuning Algorithm
// =============================================================================

class TuningEngine {
 public:
  TuningEngine(RelayController& relays, BridgeSensor& sensor)
      : relays_(relays), sensor_(sensor) {}

  TuneResult tune(uint32_t freqHz, const RelayState* hint, bool force,
                  RelayState& resultState, float& resultSwr) {
    SensorReading initial = sensor_.readAverage(kCfg.measureSamples, kCfg.measureSampleSpacingMs);

    // Power limit protection
    if (initial.powerW >= kPowerLimitW) {
      return TuneResult::PowerOverload;
    }

    if (!force && initial.powerW < kCfg.minTunePowerW) {
      return TuneResult::NoPower;
    }
    if (initial.powerW > kCfg.maxTunePowerW) {
      return TuneResult::PowerHigh;
    }

    const atu::BandInfo* band = atu::findBand(freqHz);

    RelayState best = relays_.current();
    best.bypass = false;
    float bestSwr = measureState(best);

    if (hint && !hint->bypass) {
      float hintSwr = measureState(*hint);
      if (hintSwr < bestSwr) { best = *hint; bestSwr = hintSwr; }
    }

    if (bestSwr <= kCfg.targetSWR) {
      resultState = best;
      resultSwr = bestSwr;
      relays_.applyWithSettle(best);
      return TuneResult::Success;
    }

    for (uint8_t topo = 0; topo < 2; ++topo) {
      bool topology = (topo == 1);
      std::vector<RelayState> seeds;

      RelayState s1 = best; s1.topology = topology; seeds.push_back(s1);
      RelayState s2{}; s2.topology = topology; seeds.push_back(s2);
      RelayState s3{}; s3.lMask = 0x7F; s3.cMask = 0x7F; s3.topology = topology; seeds.push_back(s3);

      if (band) {
        RelayState s4{};
        s4.lMask = band->typicalLMask;
        s4.cMask = band->typicalCMask;
        s4.topology = band->typicalTopology;
        seeds.push_back(s4);
      }

      for (const auto& seed : seeds) {
        // Check power during tune
        SensorReading check = sensor_.readOnce();
        if (check.powerW >= kPowerLimitW) {
          return TuneResult::PowerOverload;
        }

        RelayState candidate = seed;
        float swr = coarseSearch(candidate);
        if (swr < bestSwr) { best = candidate; bestSwr = swr; }
        if (bestSwr <= kCfg.targetSWR) break;

        swr = fineSearch(candidate, swr);
        if (swr < bestSwr) { best = candidate; bestSwr = swr; }
        if (bestSwr <= kCfg.targetSWR) break;
      }
      if (bestSwr <= kCfg.targetSWR) break;
    }

    relays_.applyWithSettle(best);
    SensorReading final = sensor_.readAverage(kCfg.measureSamples, kCfg.measureSampleSpacingMs);
    resultState = best;
    resultSwr = final.swr;

    if (final.swr <= kCfg.targetSWR) return TuneResult::Success;
    if (final.swr <= kCfg.goodSWR) return TuneResult::GoodEnough;
    return TuneResult::Failed;
  }

 private:
  RelayController& relays_;
  BridgeSensor& sensor_;

  float measureState(const RelayState& state) {
    relays_.applyWithSettle(state);
    SensorReading r = sensor_.readAverage(kCfg.measureSamples, kCfg.measureSampleSpacingMs);
    if (r.powerW > kCfg.maxTunePowerW) return 99.0f;
    return r.swr;
  }

  float coarseSearch(RelayState& state) {
    float bestSwr = measureState(state);
    for (uint8_t pass = 0; pass < kCfg.coarseSearchPasses; ++pass) {
      for (int8_t bit = kRelayCount - 1; bit >= 0; --bit) {
        RelayState test = state;
        test.lMask ^= (1u << bit);
        float swr = measureState(test);
        if (swr + 0.05f < bestSwr) { state = test; bestSwr = swr; }
        if (bestSwr <= kCfg.targetSWR) return bestSwr;
      }
      for (int8_t bit = kRelayCount - 1; bit >= 0; --bit) {
        RelayState test = state;
        test.cMask ^= (1u << bit);
        float swr = measureState(test);
        if (swr + 0.05f < bestSwr) { state = test; bestSwr = swr; }
        if (bestSwr <= kCfg.targetSWR) return bestSwr;
      }
    }
    return bestSwr;
  }

  float fineSearch(RelayState& state, float startSwr) {
    float bestSwr = startSwr;
    for (uint8_t pass = 0; pass < kCfg.fineSearchPasses; ++pass) {
      bool improved = false;
      for (uint8_t bit = 0; bit < kRelayCount; ++bit) {
        RelayState test = state;
        test.lMask ^= (1u << bit);
        float swr = measureState(test);
        if (swr + 0.02f < bestSwr) { state = test; bestSwr = swr; improved = true; }
        if (bestSwr <= kCfg.targetSWR) return bestSwr;
      }
      for (uint8_t bit = 0; bit < kRelayCount; ++bit) {
        RelayState test = state;
        test.cMask ^= (1u << bit);
        float swr = measureState(test);
        if (swr + 0.02f < bestSwr) { state = test; bestSwr = swr; improved = true; }
        if (bestSwr <= kCfg.targetSWR) return bestSwr;
      }
      if (!improved) break;
    }
    return bestSwr;
  }
};

// =============================================================================
// Global State
// =============================================================================

RelayController gRelays;
BridgeSensor gSensor;
CatInterface gCat;
MemoryStore gMemory;
DisplayManager gDisplay;
StatusLed gLed;
PowerProtection gPowerProt;
TuningEngine* gTuner = nullptr;
SweepEngine* gSweep = nullptr;

DebouncedButton gTuneButton;
DebouncedButton gBypassButton;
DebouncedButton gAutoButton;

Preferences gPrefsState;

RelayState gState;
SensorReading gReading;
String gStatus = "BOOT";
bool gAutoTuneEnabled = kCfg.enableAutoTuneByDefault;
bool gCatEnabled = kCfg.enableCatByDefault;
bool gTuneInProgress = false;
bool gPowerWarning = false;
bool gPowerOverload = false;
bool gPowerProtEnabled = true;  // Can disable for testing without SWR bridge
uint32_t gCurrentFreqHz = 0;
uint32_t gLastSensorMs = 0;
uint32_t gLastDisplayMs = 0;
uint32_t gLastTuneMs = 0;
uint32_t gLastSwrHighMs = 0;

char gSerialLine[128]{};
size_t gSerialIndex = 0;

// =============================================================================
// State Persistence
// =============================================================================

void saveRuntimeState() {
  RuntimeState s{};
  s.version = 1;
  s.lMask = gState.lMask;
  s.cMask = gState.cMask;
  s.flags = (gState.topology ? FLAG_TOPOLOGY : 0) | (gState.bypass ? FLAG_BYPASS : 0);
  s.autoTune = gAutoTuneEnabled ? 1 : 0;
  s.checksum = crc32(reinterpret_cast<uint8_t*>(&s), sizeof(s) - sizeof(s.checksum));
  gPrefsState.putBytes("state", &s, sizeof(s));
}

void loadRuntimeState() {
  RuntimeState s{};
  size_t got = gPrefsState.getBytes("state", &s, sizeof(s));
  if (got != sizeof(s)) return;
  uint32_t calc = crc32(reinterpret_cast<uint8_t*>(&s), sizeof(s) - sizeof(s.checksum));
  if (calc != s.checksum || s.version != 1) return;
  gState.lMask = s.lMask;
  gState.cMask = s.cMask;
  gState.topology = (s.flags & FLAG_TOPOLOGY) != 0;
  gState.bypass = (s.flags & FLAG_BYPASS) != 0;
  gAutoTuneEnabled = (s.autoTune != 0);
}

// =============================================================================
// TX Request Control
// =============================================================================

void setTxRequest(bool active) {
  auto setPin = [](int pin, bool level) {
    if (pin < 0) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level ? HIGH : LOW);
  };
  bool req = kCfg.requestTxActiveHigh ? active : !active;
  setPin(kPins.txReq, req);
  setPin(kPins.txReqInv, !req);
}

// =============================================================================
// Tuning Interface
// =============================================================================

void applyState(const RelayState& state) {
  gState = state;
  gRelays.apply(gState);
}

void engageBypassProtection() {
  gState.bypass = true;
  applyState(gState);
  gDisplay.showOverloadWarning();
  gStatus = "!OVERLOAD!";
  Serial.println("*** POWER OVERLOAD - BYPASS ENGAGED ***");
}

TuneResult runTune(bool force) {
  if (gTuneInProgress) return TuneResult::InProgress;
  if (gPowerOverload) return TuneResult::PowerOverload;
  if (!force && (millis() - gLastTuneMs < kCfg.retuneHoldoffMs)) return TuneResult::Aborted;

  gTuneInProgress = true;
  gStatus = "TUNING";
  gLed.on();

  setTxRequest(true);
  delay(kCfg.txRequestLeadMs);

  RelayState resultState;
  float resultSwr;

  RelayState memHint;
  bool hasHint = gCurrentFreqHz > 0 && gMemory.lookup(gCurrentFreqHz, memHint, nullptr);

  TuneResult result = gTuner->tune(gCurrentFreqHz, hasHint ? &memHint : nullptr, force, resultState, resultSwr);

  delay(kCfg.txRequestTrailMs);
  setTxRequest(false);

  gLastTuneMs = millis();
  gTuneInProgress = false;
  gLed.off();

  if (result == TuneResult::PowerOverload) {
    engageBypassProtection();
    return result;
  }

  applyState(resultState);
  gReading = gSensor.readAverage(kCfg.measureSamples, kCfg.measureSampleSpacingMs);

  if ((result == TuneResult::Success || result == TuneResult::GoodEnough) && gCurrentFreqHz > 0) {
    gMemory.upsert(gCurrentFreqHz, resultState, resultSwr);
  }
  saveRuntimeState();

  switch (result) {
    case TuneResult::Success: gStatus = "TUNE OK"; break;
    case TuneResult::GoodEnough: gStatus = "TUNE GOOD"; break;
    case TuneResult::NoPower: gStatus = "NO RF"; break;
    case TuneResult::PowerHigh: gStatus = "PWR HIGH"; break;
    case TuneResult::Failed: gStatus = "TUNE FAIL"; break;
    default: gStatus = "TUNE ???"; break;
  }

  return result;
}

// =============================================================================
// Serial Command Interface
// =============================================================================

String stateSummary() {
  char buf[128];
  snprintf(buf, sizeof(buf), "SWR=%.2f P=%.0fW F=%luHz L=0x%02X C=0x%02X T=%d B=%d MEM=%u CAT=%s",
           gReading.swr, gReading.powerW, static_cast<unsigned long>(gCurrentFreqHz),
           gState.lMask, gState.cMask, gState.topology ? 1 : 0, gState.bypass ? 1 : 0,
           static_cast<unsigned>(gMemory.size()), gCat.protocolName());
  return String(buf);
}

void handleCommand(const char* line) {
  if (!line || !line[0]) return;

  if (strcasecmp(line, "help") == 0) {
    Serial.println("Commands:");
    Serial.println("  status        - Show current state");
    Serial.println("  tune          - Start tuning");
    Serial.println("  tune force    - Force tune without RF");
    Serial.println("  bypass on/off - Toggle bypass");
    Serial.println("  auto on/off   - Toggle auto-tune");
    Serial.println("  freq <hz>     - Set frequency");
    Serial.println("  l/c <hex>     - Set L/C mask");
    Serial.println("  topo hi/lo    - Set topology");
    Serial.println("  mem clear/size");
    Serial.println("  cat kenwood/icom/yaesu/auto");
    Serial.println("  sweep <startMHz> <endMHz> [stepkHz] [dwellMs]");
    Serial.println("  raw / cal     - ADC diagnostics");
    Serial.println("  power reset   - Reset overload protection");
    Serial.println("  power off/on  - Disable/enable power protection");
    Serial.println("  i2cscan       - Scan I2C bus for devices");
    return;
  }

  if (strcasecmp(line, "i2cscan") == 0) {
    Serial.println("Scanning I2C bus...");
    Serial.printf("SDA: GPIO%d, SCL: GPIO%d\n", kPins.i2cSda, kPins.i2cScl);
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("  Found device at 0x%02X\n", addr);
        found++;
      }
    }
    if (found == 0) {
      Serial.println("  No I2C devices found!");
      Serial.println("  Check wiring: SDA, SCL, VCC, GND");
    } else {
      Serial.printf("  %d device(s) found\n", found);
    }
    return;
  }

  if (strcasecmp(line, "status") == 0) {
    Serial.println(stateSummary());
    Serial.printf("Power limit: %.0fW, Warning: %.0fW\n", kPowerLimitW, kPowerWarningW);
    if (gPowerOverload) Serial.println("*** OVERLOAD ACTIVE ***");
    return;
  }

  if (strcasecmp(line, "tune") == 0) { runTune(false); Serial.println(stateSummary()); return; }
  if (strcasecmp(line, "tune force") == 0) { runTune(true); Serial.println(stateSummary()); return; }
  if (strcasecmp(line, "mem clear") == 0) { gMemory.clear(); Serial.println("Memory cleared"); return; }
  if (strcasecmp(line, "mem size") == 0) { Serial.printf("Memory: %u entries\n", (unsigned)gMemory.size()); return; }
  if (strcasecmp(line, "power reset") == 0) {
    gPowerProt.reset();
    gPowerOverload = false;
    gState.bypass = false;
    applyState(gState);
    gStatus = "PWR RESET";
    Serial.println("Power protection reset");
    return;
  }
  if (strcasecmp(line, "power off") == 0) {
    gPowerProtEnabled = false;
    gPowerProt.reset();
    gPowerOverload = false;
    gPowerWarning = false;
    gState.bypass = false;
    applyState(gState);
    gStatus = "PWR OFF";
    Serial.println("Power protection DISABLED (for testing)");
    return;
  }
  if (strcasecmp(line, "power on") == 0) {
    gPowerProtEnabled = true;
    gStatus = "PWR ON";
    Serial.println("Power protection ENABLED");
    return;
  }

  if (strcasecmp(line, "raw") == 0) {
    SensorReading r = gSensor.readAverage(8, 5);
    Serial.printf("FWD: %.0f raw, %.1f mV\n", r.fwdRaw, r.fwdMv);
    Serial.printf("REV: %.0f raw, %.1f mV\n", r.revRaw, r.revMv);
    Serial.printf("SWR: %.2f, Power: %.1f W\n", r.swr, r.powerW);
    return;
  }

  if (strcasecmp(line, "cal") == 0) {
    Serial.printf("FWD offset: %.1f mV, scale: %.3f\n", kCfg.fwdOffsetMv, kCfg.fwdScale);
    Serial.printf("REV offset: %.1f mV, scale: %.3f\n", kCfg.revOffsetMv, kCfg.revScale);
    Serial.printf("Power scale: %.6f\n", kCfg.powerScale);
    Serial.printf("PIC cal: FWD=%u, REV=%u\n", kCfg.picCalForward, kCfg.picCalReverse);
    return;
  }

  // CAT protocol selection
  if (strncasecmp(line, "cat ", 4) == 0) {
    const char* proto = line + 4;
    if (strcasecmp(proto, "kenwood") == 0) gCat.setProtocol(CatProtocol::Kenwood);
    else if (strcasecmp(proto, "icom") == 0) gCat.setProtocol(CatProtocol::Icom);
    else if (strcasecmp(proto, "yaesu") == 0) gCat.setProtocol(CatProtocol::YaesuOld);
    else if (strcasecmp(proto, "auto") == 0) gCat.setProtocol(CatProtocol::Auto);
    Serial.printf("CAT protocol: %s\n", gCat.protocolName());
    return;
  }

  // Sweep command: sweep <startMHz> <endMHz> [stepkHz] [dwellMs]
  if (strncasecmp(line, "sweep ", 6) == 0) {
    float startMHz, endMHz;
    uint32_t stepKHz = 25, dwellMs = 200;
    int n = sscanf(line + 6, "%f %f %lu %lu", &startMHz, &endMHz, &stepKHz, &dwellMs);
    if (n >= 2) {
      uint32_t startHz = static_cast<uint32_t>(startMHz * 1e6);
      uint32_t endHz = static_cast<uint32_t>(endMHz * 1e6);
      Serial.printf("Starting sweep %.3f - %.3f MHz, step %lu kHz, dwell %lu ms\n",
                    startMHz, endMHz, stepKHz, dwellMs);
      Serial.println("Transmit carrier (AM/FM/RTTY) during sweep!");
      gSweep->start(startHz, endHz, stepKHz * 1000, dwellMs);
    } else {
      Serial.println("Usage: sweep <startMHz> <endMHz> [stepkHz] [dwellMs]");
    }
    return;
  }

  if (strncasecmp(line, "freq ", 5) == 0) {
    uint32_t hz = strtoul(line + 5, nullptr, 10);
    if (hz > 0) {
      gCurrentFreqHz = hz;
      RelayState hit{};
      if (gMemory.lookup(gCurrentFreqHz, hit, nullptr)) {
        applyState(hit);
        gStatus = "MEM HIT";
      } else {
        gStatus = "MEM MISS";
      }
      Serial.println(stateSummary());
    }
    return;
  }

  if (strncasecmp(line, "l ", 2) == 0) {
    gState.lMask = strtoul(line + 2, nullptr, 16) & 0x7F;
    applyState(gState); saveRuntimeState();
    Serial.println(stateSummary());
    return;
  }

  if (strncasecmp(line, "c ", 2) == 0) {
    gState.cMask = strtoul(line + 2, nullptr, 16) & 0x7F;
    applyState(gState); saveRuntimeState();
    Serial.println(stateSummary());
    return;
  }

  if (strncasecmp(line, "topo ", 5) == 0) {
    if (strcasecmp(line + 5, "hi") == 0) gState.topology = true;
    else if (strcasecmp(line + 5, "lo") == 0) gState.topology = false;
    applyState(gState); saveRuntimeState();
    Serial.println(stateSummary());
    return;
  }

  if (strncasecmp(line, "bypass ", 7) == 0) {
    if (strcasecmp(line + 7, "on") == 0) { gState.bypass = true; gStatus = "BYPASS"; }
    else if (strcasecmp(line + 7, "off") == 0) { gState.bypass = false; gStatus = "ACTIVE"; }
    applyState(gState); saveRuntimeState();
    Serial.println(stateSummary());
    return;
  }

  if (strncasecmp(line, "auto ", 5) == 0) {
    if (strcasecmp(line + 5, "on") == 0) { gAutoTuneEnabled = true; gStatus = "AUTO ON"; }
    else if (strcasecmp(line + 5, "off") == 0) { gAutoTuneEnabled = false; gStatus = "AUTO OFF"; }
    saveRuntimeState();
    Serial.println(stateSummary());
    return;
  }

  Serial.println("Unknown command. Type 'help'");
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (gSerialIndex > 0) {
        gSerialLine[gSerialIndex] = '\0';
        handleCommand(gSerialLine);
        gSerialIndex = 0;
      }
      continue;
    }
    if (gSerialIndex < sizeof(gSerialLine) - 1 && isPrintable(ch)) {
      gSerialLine[gSerialIndex++] = ch;
    }
  }
}

// =============================================================================
// Button & CAT Handlers
// =============================================================================

void handleButtons() {
  gTuneButton.loop();
  gBypassButton.loop();
  gAutoButton.loop();

  if (gTuneButton.longPress()) runTune(true);
  else if (gTuneButton.shortPress()) runTune(false);

  if (gBypassButton.shortPress()) {
    gState.bypass = !gState.bypass;
    applyState(gState); saveRuntimeState();
    gStatus = gState.bypass ? "BYPASS" : "ACTIVE";
  }

  if (gAutoButton.shortPress()) {
    gAutoTuneEnabled = !gAutoTuneEnabled;
    saveRuntimeState();
    gStatus = gAutoTuneEnabled ? "AUTO ON" : "AUTO OFF";
  }
}

void handleCat() {
  if (!gCatEnabled) return;
  gCat.loop();

  uint32_t hz = 0;
  if (!gCat.takeUpdate(hz)) return;

  if (hz != gCurrentFreqHz) {
    gCurrentFreqHz = hz;
    RelayState hit{};
    if (gMemory.lookup(gCurrentFreqHz, hit, nullptr)) {
      applyState(hit);
      gStatus = "MEM HIT";
    } else {
      gStatus = "MEM MISS";
    }
  }
}

// =============================================================================
// Periodic Tasks
// =============================================================================

void periodicSensorUpdate() {
  if (millis() - gLastSensorMs < kCfg.sensorUpdateMs) return;
  gLastSensorMs = millis();

  gReading = gSensor.readAverage(kCfg.measureSamples, kCfg.measureSampleSpacingMs);

  // Power protection check (can be disabled for testing)
  if (gPowerProtEnabled) {
    gPowerProt.check(gReading.powerW, gPowerWarning, gPowerOverload);

    if (gPowerProt.wasTriggered()) {
      engageBypassProtection();
    }
  } else {
    gPowerWarning = false;
    gPowerOverload = false;
  }
}

void maybeAutoTune() {
  if (!gAutoTuneEnabled || gTuneInProgress || gState.bypass || gPowerOverload) return;
  if (millis() - gLastTuneMs < kCfg.retuneHoldoffMs) return;
  if (gReading.powerW < kCfg.minAutoRetunePowerW) return;
  if (gReading.swr < kCfg.autoRetuneSWR) { gLastSwrHighMs = 0; return; }

  if (gLastSwrHighMs == 0) { gLastSwrHighMs = millis(); return; }
  if (millis() - gLastSwrHighMs < 500) return;
  gLastSwrHighMs = 0;

  if (gCat.connected()) {
    RelayState hit{};
    if (gMemory.lookup(gCurrentFreqHz, hit, nullptr)) {
      applyState(hit);
      gStatus = "MEM APPLY";
      return;
    }
    if (!kCfg.autoTuneUnknownCatFrequency) {
      gStatus = "CAT NO MEM";
      return;
    }
  }

  runTune(false);
}

void handleSweep() {
  if (!gSweep->isRunning()) return;

  if (gSweep->step()) {
    float progress = gSweep->getProgress();
    Serial.printf("Sweep: %.0f%% complete\n", progress * 100);

    if (!gSweep->isRunning()) {
      // Sweep complete
      Serial.println("Sweep complete!");
      const auto& points = gSweep->getPoints();

      // Print results
      Serial.println("Freq(MHz)\tSWR\tPower(W)");
      for (const auto& p : points) {
        Serial.printf("%.3f\t%.2f\t%.1f\n", p.freqHz / 1e6, p.swr, p.powerW);
      }

      // Find minimum
      uint32_t minFreq;
      float minSwr;
      if (gSweep->findMinSwr(minFreq, minSwr)) {
        Serial.printf("Minimum SWR: %.2f at %.3f MHz\n", minSwr, minFreq / 1e6);
      }

      // Show graph on LCD
      gDisplay.showSweepGraph(points, points.front().freqHz, points.back().freqHz);
      delay(5000);
    }
  }
}

void periodicDisplay() {
  if (millis() - gLastDisplayMs < kCfg.displayUpdateMs) return;
  gLastDisplayMs = millis();

  if (gSweep->isRunning()) {
    // Show sweep progress
    char buf[20];
    snprintf(buf, sizeof(buf), "SWEEP %.0f%%", gSweep->getProgress() * 100);
    gStatus = buf;
  }

  gDisplay.render(gCurrentFreqHz, gCat.connected(), gAutoTuneEnabled, gState, gReading,
                  gRelays.totalL(), gRelays.totalC(), gStatus.c_str(),
                  gPowerWarning, gPowerOverload);
}

// =============================================================================
// Arduino Entry Points
// =============================================================================

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("ATU-1000 ESP32-S3 Controller v1.1");
  Serial.println("Icom/Yaesu/Kenwood CAT + 1kW Protect");
  Serial.println("=====================================");

  gPrefsState.begin("atu_state", false);
  loadRuntimeState();

  Serial.println("Init relays...");
  gRelays.begin();
  Serial.println("Init sensor...");
  gSensor.begin();
  Serial.println("Init memory...");
  gMemory.begin();
  Serial.println("Init display...");
  gDisplay.begin();
  Serial.println("Init LED...");
  gLed.begin();

  gDisplay.showSplash("v1.1");

  gTuneButton.begin(kPins.tuneButton);
  gBypassButton.begin(kPins.bypassButton);
  gAutoButton.begin(kPins.autoButton);

  if (kCfg.enableBypassOnBoot) gState.bypass = true;
  applyState(gState);
  setTxRequest(false);

  if (kCfg.enableCatByDefault) {
    gCat.begin();
    gCatEnabled = true;
  }

  gTuner = new TuningEngine(gRelays, gSensor);
  gSweep = new SweepEngine(gCat, gSensor, gMemory);

  delay(100);
  gReading = gSensor.readAverage(4, 4);

  gStatus = "READY";
  Serial.println("Ready. Type 'help' for commands.");
  Serial.printf("Power limit: %.0fW\n", kPowerLimitW);
}

void loop() {
  pollSerialCommands();
  handleButtons();
  handleCat();
  periodicSensorUpdate();
  handleSweep();
  maybeAutoTune();
  gMemory.loop();
  periodicDisplay();

  if (gPowerOverload) {
    gLed.blink(50, 50);  // Fast blink = overload
  } else if (gTuneInProgress || gSweep->isRunning()) {
    gLed.blink(100, 100);
  } else if (gState.bypass) {
    gLed.blink(500, 500);
  } else if (gPowerWarning) {
    gLed.blink(200, 200);  // Medium blink = warning
  }
}
