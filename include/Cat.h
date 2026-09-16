#pragma once
//
// Multi-protocol CAT frequency reader.
//
// The original auto-detect was broken: the Yaesu binary parser consumed one
// byte from the ring on *every* received byte once five bytes were buffered,
// so the buffer could never grow past five. Kenwood needs a 14-byte command and
// Icom needs at least 11, so in Auto mode neither could ever match and the only
// protocol that worked was Yaesu.
//
// This version:
//   * runs the framed protocols (Icom, Kenwood) first and only lets the
//     unframed Yaesu parser consume bytes when nothing else is plausibly
//     mid-frame,
//   * requires several consistent Yaesu frames before it will lock, because a
//     5-byte unframed protocol will otherwise happily "detect" random ASCII,
//   * keeps the selected and the detected protocol distinct, and uses the
//     selected one for transmits (the old code keyed setFrequency() off the
//     detected protocol, so a sweep did nothing until the radio had already
//     reported a frequency).
//

#include <Arduino.h>
#include <HardwareSerial.h>

#include <cctype>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

// Icom CI-V
constexpr uint8_t kIcomPreamble = 0xFE;
constexpr uint8_t kIcomEOM = 0xFD;
constexpr uint8_t kIcomControllerAddr = 0xE0;
constexpr uint8_t kIcomBroadcast = 0x00;

class CatInterface {
 public:
  void begin(Settings* settings) {
    cfg_ = settings;
    if (kPins.catRx < 0 || !cfg_->catEnabled) {
      enabled_ = false;
      return;
    }
    serial_.begin(cfg_->catBaud, SERIAL_8N1, kPins.catRx, kPins.catTx);
    enabled_ = true;
    selected_ = static_cast<CatProtocol>(cfg_->catProtocol);
    detected_ = CatProtocol::None;
    bufIdx_ = 0;
    lastRxMs_ = 0;
    everRx_ = false;
  }

  void end() {
    if (enabled_) serial_.end();
    enabled_ = false;
  }

  void restart() { end(); begin(cfg_); }

  bool enabled() const { return enabled_; }

  void setProtocol(CatProtocol p) {
    selected_ = p;
    cfg_->catProtocol = static_cast<uint8_t>(p);
    if (p != CatProtocol::Auto) detected_ = p;
    bufIdx_ = 0;
    yaesuConfidence_ = 0;
  }

  CatProtocol selectedProtocol() const { return selected_; }
  CatProtocol detectedProtocol() const { return detected_; }

  // What the UI should show: the locked protocol if we have one, else the
  // selection.
  const char* protocolName() const {
    return catProtocolName(detected_ != CatProtocol::None ? detected_ : selected_);
  }

  void loop() {
    if (!enabled_) return;

    int guard = 0;
    while (serial_.available() > 0 && guard++ < 256) {
      uint8_t b = static_cast<uint8_t>(serial_.read());
      if (bufIdx_ < sizeof(buffer_)) {
        buffer_[bufIdx_++] = b;
      } else {
        // Drop the oldest quarter rather than the whole buffer, so a frame
        // straddling the boundary still has a chance.
        memmove(buffer_, buffer_ + 32, bufIdx_ - 32);
        bufIdx_ -= 32;
        buffer_[bufIdx_++] = b;
      }
      parse();
    }
  }

  // Considered connected only once a frequency has actually been decoded.
  // The old version treated lastRxMs_ == 0 as "just heard from", so the display
  // claimed CAT for the first five seconds after every boot.
  bool connected() const {
    return enabled_ && everRx_ && elapsed(lastRxMs_) <= cfg_->catTimeoutMs;
  }

  bool takeUpdate(uint32_t& freqHz) {
    if (!updated_) return false;
    updated_ = false;
    freqHz = lastFreqHz_;
    return true;
  }

  uint32_t lastFrequency() const { return lastFreqHz_; }

  bool setFrequency(uint32_t hz) {
    if (!enabled_ || kPins.catTx < 0) return false;
    CatProtocol p = (selected_ != CatProtocol::Auto) ? selected_ : detected_;
    switch (p) {
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

  // Ask the radio for its current frequency (used to snapshot VFO before a
  // sweep so it can be restored afterwards).
  bool requestFrequency() {
    if (!enabled_ || kPins.catTx < 0) return false;
    CatProtocol p = (selected_ != CatProtocol::Auto) ? selected_ : detected_;
    switch (p) {
      case CatProtocol::Kenwood:
      case CatProtocol::YaesuNew:
        serial_.print("FA;");
        return true;
      case CatProtocol::Icom: {
        uint8_t cmd[6] = {kIcomPreamble, kIcomPreamble, icomRadioAddr_,
                          kIcomControllerAddr, 0x03, kIcomEOM};
        serial_.write(cmd, 6);
        return true;
      }
      case CatProtocol::YaesuOld: {
        uint8_t cmd[5] = {0, 0, 0, 0, 0x03};
        serial_.write(cmd, 5);
        return true;
      }
      default:
        return false;
    }
  }

 private:
  HardwareSerial serial_{1};
  Settings* cfg_ = nullptr;
  bool enabled_ = false;
  bool updated_ = false;
  bool everRx_ = false;
  uint32_t lastFreqHz_ = 0;
  uint32_t lastRxMs_ = 0;
  CatProtocol selected_ = CatProtocol::Auto;
  CatProtocol detected_ = CatProtocol::None;
  uint8_t buffer_[192]{};
  size_t bufIdx_ = 0;
  uint8_t icomRadioAddr_ = kIcomBroadcast;

  // Yaesu's 5-byte protocol has no framing at all, so a lock is only granted
  // after this many consecutive plausible frames.
  uint8_t yaesuConfidence_ = 0;
  uint32_t lastYaesuHz_ = 0;
  static constexpr uint8_t kYaesuLockFrames = 3;

  bool want(CatProtocol p) const {
    return selected_ == CatProtocol::Auto || selected_ == p;
  }

  void freqUpdated(uint32_t hz, CatProtocol proto) {
    if (hz < 1000000UL || hz > 60000000UL) return;
    lastFreqHz_ = hz;
    lastRxMs_ = millis();
    everRx_ = true;
    updated_ = true;
    detected_ = proto;
  }

  void consume(size_t n) {
    if (n >= bufIdx_) { bufIdx_ = 0; return; }
    memmove(buffer_, buffer_ + n, bufIdx_ - n);
    bufIdx_ -= n;
  }

  // Framed protocols get first refusal. Yaesu, being unframed and destructive,
  // only runs when neither framed parser could be mid-frame.
  void parse() {
    if (want(CatProtocol::Icom) && tryParseIcom()) return;
    if (want(CatProtocol::Kenwood) || want(CatProtocol::YaesuNew)) {
      if (tryParseKenwood()) return;
    }
    if (want(CatProtocol::YaesuOld) && !framedInProgress()) {
      tryParseYaesuOld();
    }
    // Nothing matched and nothing looks like a partial frame: stop the buffer
    // growing without bound on a link that is just noise.
    if (bufIdx_ >= sizeof(buffer_) - 8 && !framedInProgress()) consume(64);
  }

  // True when the buffer holds the start of an Icom frame or ASCII that could
  // still turn into a Kenwood command.
  bool framedInProgress() const {
    if (selected_ == CatProtocol::YaesuOld) return false;
    for (size_t i = 0; i < bufIdx_; ++i) {
      if (buffer_[i] == kIcomPreamble) return true;
    }
    for (size_t i = 0; i + 1 < bufIdx_; ++i) {
      if ((buffer_[i] == 'F' && buffer_[i + 1] == 'A') ||
          (buffer_[i] == 'I' && buffer_[i + 1] == 'F')) {
        return true;
      }
    }
    return false;
  }

  // ---- Kenwood / Elecraft / Yaesu-new ASCII -------------------------------

  bool tryParseKenwood() {
    for (size_t i = 0; i < bufIdx_; ++i) {
      if (buffer_[i] != ';') continue;

      // Work on a copy: the original wrote a NUL straight into the ring, which
      // corrupted any Icom binary frame that happened to contain 0x3B.
      char cmd[64];
      size_t n = (i < sizeof(cmd) - 1) ? i : sizeof(cmd) - 1;
      memcpy(cmd, buffer_, n);
      cmd[n] = '\0';

      bool ok = false;
      const char* fa = strstr(cmd, "FA");
      if (fa) {
        uint32_t hz = parseAsciiFreq(fa + 2);
        if (hz > 0) { freqUpdated(hz, CatProtocol::Kenwood); ok = true; }
      }
      if (!ok) {
        const char* ifc = strstr(cmd, "IF");
        if (ifc && strlen(ifc) >= 13) {
          uint32_t hz = parseAsciiFreq(ifc + 2);
          if (hz > 0) { freqUpdated(hz, CatProtocol::Kenwood); ok = true; }
        }
      }

      // Either way the command is complete; drop it and keep whatever arrived
      // behind it (the original threw the remainder away on success).
      consume(i + 1);
      return ok;
    }
    return false;
  }

  static uint32_t parseAsciiFreq(const char* p) {
    uint64_t val = 0;
    uint8_t digits = 0;
    // isdigit() on a plain char is undefined for bytes >= 0x80.
    while (isdigit(static_cast<unsigned char>(*p)) && digits < 12) {
      val = val * 10ULL + static_cast<uint64_t>(*p - '0');
      ++digits;
      ++p;
    }
    if (digits >= 7 && val >= 1000000ULL && val <= 60000000ULL) {
      return static_cast<uint32_t>(val);
    }
    return 0;
  }

  bool sendKenwoodFreq(uint32_t hz) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "FA%011lu;", static_cast<unsigned long>(hz));
    serial_.print(cmd);
    return true;
  }

  // ---- Icom CI-V ----------------------------------------------------------

  bool tryParseIcom() {
    for (size_t i = 0; i + 1 < bufIdx_; ++i) {
      if (buffer_[i] != kIcomPreamble || buffer_[i + 1] != kIcomPreamble) continue;

      for (size_t j = i + 4; j < bufIdx_; ++j) {
        if (buffer_[j] != kIcomEOM) continue;

        uint8_t toAddr = buffer_[i + 2];
        uint8_t fromAddr = buffer_[i + 3];
        uint8_t cmd = buffer_[i + 4];

        // Only accept frames aimed at us or broadcast, so a second controller
        // on the CI-V bus cannot drag us to the wrong frequency.
        bool forUs = (toAddr == kIcomControllerAddr || toAddr == kIcomBroadcast);

        if (fromAddr != kIcomControllerAddr) icomRadioAddr_ = fromAddr;

        // 0x00 (freq broadcast) and 0x03 (freq reply) carry 5 BCD bytes.
        if (forUs && (cmd == 0x00 || cmd == 0x03) && (j - i) >= 10) {
          uint32_t hz = bcdToHz(&buffer_[i + 5], 5);
          if (hz > 0) freqUpdated(hz, CatProtocol::Icom);
        }

        consume(j + 1);
        return true;
      }
      // Preamble seen but no EOM yet: wait for more bytes.
      if (i > 0) consume(i);
      return false;
    }
    return false;
  }

  static uint32_t bcdToHz(const uint8_t* bcd, uint8_t len) {
    uint32_t hz = 0;
    uint32_t mult = 1;
    for (uint8_t i = 0; i < len; ++i) {
      hz += (bcd[i] & 0x0F) * mult;
      mult *= 10;
      hz += ((bcd[i] >> 4) & 0x0F) * mult;
      mult *= 10;
    }
    return hz;
  }

  static void hzToBcd(uint32_t hz, uint8_t* bcd, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
      bcd[i] = static_cast<uint8_t>(hz % 10);
      hz /= 10;
      bcd[i] |= static_cast<uint8_t>((hz % 10) << 4);
      hz /= 10;
    }
  }

  bool sendIcomFreq(uint32_t hz) {
    uint8_t cmd[11];
    cmd[0] = kIcomPreamble;
    cmd[1] = kIcomPreamble;
    cmd[2] = icomRadioAddr_;
    cmd[3] = kIcomControllerAddr;
    cmd[4] = 0x05;              // set frequency
    hzToBcd(hz, &cmd[5], 5);
    cmd[10] = kIcomEOM;
    serial_.write(cmd, sizeof(cmd));
    return true;
  }

  // ---- Yaesu 5-byte binary ------------------------------------------------

  bool tryParseYaesuOld() {
    if (bufIdx_ < 5) return false;

    uint32_t hz = 0;
    hz += ((buffer_[0] >> 4) & 0x0F) * 10000000UL;
    hz += (buffer_[0] & 0x0F) * 1000000UL;
    hz += ((buffer_[1] >> 4) & 0x0F) * 100000UL;
    hz += (buffer_[1] & 0x0F) * 10000UL;
    hz += ((buffer_[2] >> 4) & 0x0F) * 1000UL;
    hz += (buffer_[2] & 0x0F) * 100UL;
    hz += ((buffer_[3] >> 4) & 0x0F) * 10UL;
    hz += (buffer_[3] & 0x0F);
    hz *= 10;  // Yaesu reports in 10 Hz units

    bool nibblesValid = true;
    for (uint8_t i = 0; i < 4; ++i) {
      if ((buffer_[i] & 0x0F) > 9 || ((buffer_[i] >> 4) & 0x0F) > 9) {
        nibblesValid = false;
        break;
      }
    }

    if (nibblesValid && hz >= 1000000UL && hz <= 60000000UL) {
      if (selected_ == CatProtocol::YaesuOld) {
        freqUpdated(hz, CatProtocol::YaesuOld);
      } else {
        // Auto mode: require repeated, self-consistent frames before locking,
        // so an ASCII stream cannot masquerade as BCD.
        uint32_t delta = (hz > lastYaesuHz_) ? (hz - lastYaesuHz_) : (lastYaesuHz_ - hz);
        if (yaesuConfidence_ > 0 && delta < 1000000UL) {
          ++yaesuConfidence_;
        } else {
          yaesuConfidence_ = 1;
        }
        lastYaesuHz_ = hz;
        if (yaesuConfidence_ >= kYaesuLockFrames) {
          freqUpdated(hz, CatProtocol::YaesuOld);
        }
      }
      consume(5);
      return true;
    }

    consume(1);
    return false;
  }

  bool sendYaesuOldFreq(uint32_t hz) {
    uint8_t cmd[5];
    hz /= 10;  // 10 Hz units
    cmd[0] = static_cast<uint8_t>(((hz / 10000000UL) << 4) | ((hz / 1000000UL) % 10));
    cmd[1] = static_cast<uint8_t>((((hz / 100000UL) % 10) << 4) | ((hz / 10000UL) % 10));
    cmd[2] = static_cast<uint8_t>((((hz / 1000UL) % 10) << 4) | ((hz / 100UL) % 10));
    cmd[3] = static_cast<uint8_t>((((hz / 10UL) % 10) << 4) | (hz % 10));
    cmd[4] = 0x01;  // set frequency
    serial_.write(cmd, sizeof(cmd));
    return true;
  }
};

}  // namespace atu
