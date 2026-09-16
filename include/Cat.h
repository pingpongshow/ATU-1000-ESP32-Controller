#pragma once
//
// Multi-protocol CAT interface.
//
// Frequency tracking:
//   * framed protocols (Icom, Kenwood/Elecraft/Flex ASCII) are parsed first;
//     the unframed Yaesu 5-byte parser only consumes bytes when nothing else
//     is plausibly mid-frame, and needs several consistent frames to lock in
//     Auto mode, because it will otherwise "detect" random ASCII,
//   * the radio is polled every `catpoll` ms once the protocol is known, so
//     radios with auto-information off, and Icoms with CI-V transceive off,
//     still report their frequency. Nothing is ever sent while the protocol is
//     unknown: probing an FT-817 with Kenwood text could land on its PTT
//     opcode.
//
// FlexRadio: SmartSDR CAT speaks Kenwood plus ZZ extensions. It is polled with
// ZZFA; and keyed for tuning with ZZTU (the radio's own TUNE carrier and TUNE
// power setting).
//
// USB passthrough (`catusb on`): the ESP32-S3's native USB port appears as a
// serial port on a PC. Everything the PC sends goes to the radio and
// everything the radio sends goes to the PC, so logging software and the tuner
// share the radio's single CAT port. While the PC is actively polling, the
// tuner stops polling and just reads the replies. The tuner's own commands are
// held back until the PC is between frames, so they never split one of its
// commands. Replies to the tuner's own commands are also seen by the PC.
//
// Transmit control (CAT tune): query and save the radio's power and mode, set
// a low-power carrier, key, unkey, restore.
//

#include <Arduino.h>
#include <HardwareSerial.h>

#include <cctype>

#include "Config.h"
#include "Settings.h"
#include "Types.h"

#if ARDUINO_USB_MODE && !ARDUINO_USB_CDC_ON_BOOT
#include "HWCDC.h"
#define ATU_HAS_USB_CAT 1
#else
#define ATU_HAS_USB_CAT 0
#endif

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
    if (selected_ == CatProtocol::None) selected_ = CatProtocol::Auto;
    detected_ = CatProtocol::None;
    bufIdx_ = 0;
    lastRxMs_ = 0;
    everRx_ = false;
    txHead_ = txTail_ = 0;
    icomRadioAddr_ = cfg_->icomAddress ? cfg_->icomAddress : kIcomBroadcast;

#if ATU_HAS_USB_CAT
    if (cfg_->catUsbPassthrough && !usbStarted_) {
      USBSerial.setTxTimeoutMs(0);   // never block when no PC is reading
      USBSerial.begin();
      usbStarted_ = true;
    }
#endif
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

  // The protocol commands go out in: the selection, or what Auto locked on to.
  CatProtocol activeProtocol() const {
    return (selected_ != CatProtocol::Auto) ? selected_ : detected_;
  }

  // What the UI should show: the locked protocol if we have one, else the
  // selection.
  const char* protocolName() const {
    return catProtocolName(detected_ != CatProtocol::None ? detected_ : selected_);
  }

  void loop() {
    if (!enabled_) return;

    pumpUsb();

    uint8_t chunk[64];
    int guard = 0;
    while (serial_.available() > 0 && guard++ < 256) {
      size_t n = 0;
      while (n < sizeof(chunk) && serial_.available() > 0) {
        chunk[n++] = static_cast<uint8_t>(serial_.read());
      }
      forwardToUsb(chunk, n);
      for (size_t i = 0; i < n; ++i) {
        uint8_t b = chunk[i];
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

    pollFrequency();
    flushTx();
  }

  // Considered connected only once a frequency has actually been decoded.
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

  // True while a PC on the USB passthrough has sent something recently.
  bool pcActive() const {
    return everPc_ && elapsed(lastPcMs_) < kPcActiveMs;
  }
  bool usbPassthrough() const { return usbStarted_ && cfg_->catUsbPassthrough; }

  bool setFrequency(uint32_t hz) {
    if (!canSend()) return false;
    char cmd[24];
    switch (activeProtocol()) {
      case CatProtocol::Kenwood:
        snprintf(cmd, sizeof(cmd), "FA%011lu;", static_cast<unsigned long>(hz));
        sendText(cmd);
        return true;
      case CatProtocol::YaesuNew:
        // FT-991 / FTDX10 / FTDX101 take nine digits.
        snprintf(cmd, sizeof(cmd), "FA%09lu;", static_cast<unsigned long>(hz));
        sendText(cmd);
        return true;
      case CatProtocol::Flex:
        snprintf(cmd, sizeof(cmd), "ZZFA%011lu;", static_cast<unsigned long>(hz));
        sendText(cmd);
        return true;
      case CatProtocol::Icom: {
        uint8_t body[6];
        body[0] = 0x05;              // set frequency
        hzToBcd(hz, &body[1], 5);
        return sendIcom(body, sizeof(body));
      }
      case CatProtocol::YaesuOld: {
        uint8_t f[5];
        uint32_t v = hz / 10;        // 10 Hz units
        f[0] = static_cast<uint8_t>(((v / 10000000UL) << 4) | ((v / 1000000UL) % 10));
        f[1] = static_cast<uint8_t>((((v / 100000UL) % 10) << 4) | ((v / 10000UL) % 10));
        f[2] = static_cast<uint8_t>((((v / 1000UL) % 10) << 4) | ((v / 100UL) % 10));
        f[3] = static_cast<uint8_t>((((v / 10UL) % 10) << 4) | (v % 10));
        f[4] = 0x01;                 // set frequency
        enqueue(f, sizeof(f));
        return true;
      }
      default:
        return false;
    }
  }

  // Ask the radio for its current frequency.
  bool requestFrequency() {
    if (!canSend()) return false;
    switch (activeProtocol()) {
      case CatProtocol::Kenwood:
      case CatProtocol::YaesuNew:
        sendText("FA;");
        return true;
      case CatProtocol::Flex:
        sendText("ZZFA;");
        return true;
      case CatProtocol::Icom: {
        uint8_t body[1] = {0x03};
        return sendIcom(body, sizeof(body));
      }
      case CatProtocol::YaesuOld: {
        uint8_t cmd[5] = {0, 0, 0, 0, 0x03};   // also returns the mode byte
        enqueue(cmd, sizeof(cmd));
        return true;
      }
      default:
        return false;
    }
  }

  // ---- Transmit control (CAT tune) ----------------------------------------

  // A protocol is known and there is a TX line to the radio.
  bool canKey() const {
    if (!canSend()) return false;
    CatProtocol p = activeProtocol();
    if (p == CatProtocol::Icom && icomRadioAddr_ == kIcomBroadcast) return false;
    return p == CatProtocol::Kenwood || p == CatProtocol::YaesuNew ||
           p == CatProtocol::Flex || p == CatProtocol::Icom ||
           p == CatProtocol::YaesuOld;
  }

  // Clears what we know about the radio's power and mode and asks again.
  void queryTxSettings() {
    havePower_ = haveMode_ = false;
    capturing_ = true;
    switch (activeProtocol()) {
      case CatProtocol::Kenwood:
        sendText("PC;MD;");
        break;
      case CatProtocol::YaesuNew:
        sendText("PC;MD0;");
        break;
      case CatProtocol::Icom: {
        uint8_t mode[1] = {0x04};
        uint8_t power[2] = {0x14, 0x0A};
        sendIcom(mode, sizeof(mode));
        sendIcom(power, sizeof(power));
        break;
      }
      case CatProtocol::YaesuOld:
        requestFrequency();   // the reply carries the mode
        break;
      default:
        break;
    }
  }

  // Everything needed to put the radio back afterwards has been read.
  bool txSettingsKnown() const {
    switch (activeProtocol()) {
      case CatProtocol::Flex: return true;              // uses the radio's TUNE power
      case CatProtocol::YaesuOld: return haveMode_;     // no CAT power control
      default: return havePower_ && haveMode_;
    }
  }

  // Power is not settable over CAT on the Yaesu 5-byte radios; Flex uses its
  // own TUNE power, so neither changes power here.
  void setCarrier(uint8_t watts, CarrierMode mode) {
    capturing_ = false;
    char cmd[16];
    switch (activeProtocol()) {
      case CatProtocol::Kenwood:
        snprintf(cmd, sizeof(cmd), "PC%03u;MD%c;", watts, kenwoodMode(mode));
        sendText(cmd);
        break;
      case CatProtocol::YaesuNew:
        snprintf(cmd, sizeof(cmd), "PC%03u;MD0%c;", watts, kenwoodMode(mode));
        sendText(cmd);
        break;
      case CatProtocol::Icom: {
        uint8_t m[3] = {0x06, icomMode(mode), 0x01};
        sendIcom(m, sizeof(m));
        uint16_t level = static_cast<uint16_t>((watts * 255U + 50U) / 100U);   // % of a 100 W radio
        uint8_t p[4] = {0x14, 0x0A, 0, 0};
        levelToBcd(level > 255 ? 255 : level, &p[2]);
        sendIcom(p, sizeof(p));
        break;
      }
      case CatProtocol::YaesuOld: {
        uint8_t m[5] = {yaesuOldMode(mode), 0, 0, 0, 0x07};
        enqueue(m, sizeof(m));
        break;
      }
      default:
        break;
    }
  }

  void setKey(bool on) {
    switch (activeProtocol()) {
      case CatProtocol::Kenwood:
        sendText(on ? "TX;" : "RX;");
        break;
      case CatProtocol::YaesuNew:
        sendText(on ? "TX1;" : "TX0;");
        break;
      case CatProtocol::Flex:
        sendText(on ? "ZZTU1;" : "ZZTU0;");
        break;
      case CatProtocol::Icom: {
        uint8_t k[3] = {0x1C, 0x00, static_cast<uint8_t>(on ? 0x01 : 0x00)};
        sendIcom(k, sizeof(k));
        break;
      }
      case CatProtocol::YaesuOld: {
        uint8_t k[5] = {0, 0, 0, 0, static_cast<uint8_t>(on ? 0x08 : 0x88)};
        enqueue(k, sizeof(k));
        break;
      }
      default:
        break;
    }
    // Not forced out mid PC frame: that would splice the two into a command
    // the radio rejects. loop() sends it within kPcFrameStaleMs, and CatTune
    // verifies on the bridge that RF actually stopped.
  }

  void restoreTxSettings() {
    char cmd[16];
    switch (activeProtocol()) {
      case CatProtocol::Kenwood:
      case CatProtocol::YaesuNew:
        if (havePower_) {
          snprintf(cmd, sizeof(cmd), "PC%03u;", savedPower_);
          sendText(cmd);
        }
        if (haveMode_) {
          snprintf(cmd, sizeof(cmd), "MD%s;", savedModeText_);
          sendText(cmd);
        }
        break;
      case CatProtocol::Icom:
        if (haveMode_) {
          uint8_t m[3] = {0x06, savedIcomMode_, savedIcomFilter_};
          sendIcom(m, sizeof(m));
        }
        if (havePower_) {
          uint8_t p[4] = {0x14, 0x0A, 0, 0};
          levelToBcd(savedPower_, &p[2]);
          sendIcom(p, sizeof(p));
        }
        break;
      case CatProtocol::YaesuOld:
        if (haveMode_) {
          uint8_t m[5] = {savedYaesuMode_, 0, 0, 0, 0x07};
          enqueue(m, sizeof(m));
        }
        break;
      default:
        break;
    }
  }

 private:
  static constexpr uint32_t kPcActiveMs = 3000;
  static constexpr uint32_t kPcFrameStaleMs = 150;

  HardwareSerial serial_{1};
  Settings* cfg_ = nullptr;
  bool enabled_ = false;
  bool updated_ = false;
  bool everRx_ = false;
  uint32_t lastFreqHz_ = 0;
  uint32_t lastRxMs_ = 0;
  uint32_t lastPollMs_ = 0;
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

  // Saved transmit settings for CAT tune
  bool capturing_ = false;
  bool havePower_ = false;
  bool haveMode_ = false;
  uint16_t savedPower_ = 0;          // watts (ASCII radios) or 0-255 level (Icom)
  char savedModeText_[4] = "";       // "4" (Kenwood) or "04" (Yaesu new)
  uint8_t savedIcomMode_ = 0;
  uint8_t savedIcomFilter_ = 1;
  uint8_t savedYaesuMode_ = 0;

  // Outbound queue, so our commands never interleave with a PC's frame.
  uint8_t txBuf_[512]{};
  size_t txHead_ = 0, txTail_ = 0;

  // USB passthrough
  bool usbStarted_ = false;
  bool everPc_ = false;
  uint32_t lastPcMs_ = 0;
  bool pcFrameOpen_ = false;
  uint8_t pcYaesuCount_ = 0;

  bool canSend() const { return enabled_ && kPins.catTx >= 0; }

  bool want(CatProtocol p) const {
    return selected_ == CatProtocol::Auto || selected_ == p;
  }

  // ---- outbound ---------------------------------------------------------------

  void enqueue(const uint8_t* data, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      size_t next = (txHead_ + 1) % sizeof(txBuf_);
      if (next == txTail_) return;     // full: drop rather than block
      txBuf_[txHead_] = data[i];
      txHead_ = next;
    }
    flushTx();
  }

  void sendText(const char* s) {
    enqueue(reinterpret_cast<const uint8_t*>(s), strlen(s));
  }

  bool sendIcom(const uint8_t* body, size_t n) {
    if (icomRadioAddr_ == kIcomBroadcast) return false;   // address not known yet
    uint8_t frame[16];
    if (n + 5 > sizeof(frame)) return false;
    frame[0] = kIcomPreamble;
    frame[1] = kIcomPreamble;
    frame[2] = icomRadioAddr_;
    frame[3] = kIcomControllerAddr;
    memcpy(frame + 4, body, n);
    frame[4 + n] = kIcomEOM;
    enqueue(frame, n + 5);
    return true;
  }

  void flushTx() {
    if (!enabled_ || txHead_ == txTail_) return;
    if (usbPassthrough() && pcFrameOpen_ && elapsed(lastPcMs_) < kPcFrameStaleMs) {
      return;   // the PC is mid-command; wait for its terminator
    }
    while (txTail_ != txHead_) {
      serial_.write(txBuf_[txTail_]);
      txTail_ = (txTail_ + 1) % sizeof(txBuf_);
    }
  }

  void pollFrequency() {
    if (cfg_->catPollMs == 0 || !canSend()) return;
    if (pcActive()) return;   // the PC is polling; its replies update us
    if (elapsed(lastPollMs_) < cfg_->catPollMs) return;
    lastPollMs_ = millis();
    requestFrequency();
  }

  // ---- USB passthrough ------------------------------------------------------

  void pumpUsb() {
#if ATU_HAS_USB_CAT
    if (!usbPassthrough()) return;
    int guard = 0;
    while (USBSerial.available() > 0 && guard++ < 256) {
      uint8_t b = static_cast<uint8_t>(USBSerial.read());
      serial_.write(b);
      notePcByte(b);
    }
    if (!pcFrameOpen_ || elapsed(lastPcMs_) >= kPcFrameStaleMs) flushTx();
#endif
  }

  void forwardToUsb(const uint8_t* data, size_t n) {
#if ATU_HAS_USB_CAT
    if (usbPassthrough() && HWCDC::isConnected()) USBSerial.write(data, n);
#else
    (void)data;
    (void)n;
#endif
  }

  void notePcByte(uint8_t b) {
    // A pause means the PC finished whatever it was sending; resynchronise the
    // unframed Yaesu byte count.
    if (elapsed(lastPcMs_) >= kPcFrameStaleMs) pcYaesuCount_ = 0;
    everPc_ = true;
    lastPcMs_ = millis();
    switch (activeProtocol()) {
      case CatProtocol::Icom:
        if (b == kIcomPreamble) pcFrameOpen_ = true;
        else if (b == kIcomEOM) pcFrameOpen_ = false;
        break;
      case CatProtocol::YaesuOld:
        pcYaesuCount_ = static_cast<uint8_t>((pcYaesuCount_ + 1) % 5);
        pcFrameOpen_ = pcYaesuCount_ != 0;
        break;
      case CatProtocol::Kenwood:
      case CatProtocol::YaesuNew:
      case CatProtocol::Flex:
        pcFrameOpen_ = (b != ';');
        break;
      default:
        pcFrameOpen_ = true;   // unknown framing: rely on the idle timeout
        break;
    }
  }

  // ---- inbound ----------------------------------------------------------------

  void freqUpdated(uint32_t hz, CatProtocol proto) {
    if (hz < 1000000UL || hz > 60000000UL) return;
    lastFreqHz_ = hz;
    lastRxMs_ = millis();
    everRx_ = true;
    updated_ = true;
    if (selected_ == CatProtocol::Auto || detected_ == CatProtocol::None) detected_ = proto;
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
    if (want(CatProtocol::Kenwood) || want(CatProtocol::YaesuNew) || want(CatProtocol::Flex)) {
      if (tryParseAscii()) return;
    }
    if (want(CatProtocol::YaesuOld) && !framedInProgress()) {
      tryParseYaesuOld();
    }
    // Nothing matched and nothing looks like a partial frame: stop the buffer
    // growing without bound on a link that is just noise.
    if (bufIdx_ >= sizeof(buffer_) - 8 && !framedInProgress()) consume(64);
  }

  // True when the buffer holds the start of an Icom frame or ASCII that could
  // still turn into a Kenwood-style command.
  bool framedInProgress() const {
    if (selected_ == CatProtocol::YaesuOld) return false;
    for (size_t i = 0; i < bufIdx_; ++i) {
      if (buffer_[i] == kIcomPreamble) return true;
    }
    for (size_t i = 0; i + 1 < bufIdx_; ++i) {
      if ((buffer_[i] == 'F' && buffer_[i + 1] == 'A') ||
          (buffer_[i] == 'I' && buffer_[i + 1] == 'F') ||
          (buffer_[i] == 'Z' && buffer_[i + 1] == 'Z')) {
        return true;
      }
    }
    return false;
  }

  // ---- Kenwood / Elecraft / Yaesu-new / Flex ASCII ----------------------------

  bool tryParseAscii() {
    for (size_t i = 0; i < bufIdx_; ++i) {
      if (buffer_[i] != ';') continue;

      // Work on a copy: writing a NUL into the ring would corrupt any Icom
      // binary frame that happened to contain 0x3B.
      char cmd[64];
      size_t n = (i < sizeof(cmd) - 1) ? i : sizeof(cmd) - 1;
      memcpy(cmd, buffer_, n);
      cmd[n] = '\0';

      bool ok = false;
      if (const char* zz = strstr(cmd, "ZZFA")) {
        uint32_t hz = parseAsciiFreq(zz + 4);
        if (hz > 0) { freqUpdated(hz, CatProtocol::Flex); ok = true; }
      }
      if (!ok) {
        if (const char* fa = strstr(cmd, "FA")) {
          uint32_t hz = parseAsciiFreq(fa + 2);
          if (hz > 0) { freqUpdated(hz, CatProtocol::Kenwood); ok = true; }
        }
      }
      if (!ok) {
        const char* ifc = strstr(cmd, "IF");
        if (ifc && strlen(ifc) >= 13) {
          uint32_t hz = parseAsciiFreq(ifc + 2);
          if (hz > 0) { freqUpdated(hz, CatProtocol::Kenwood); ok = true; }
        }
      }
      if (!ok) ok = parseTxSettingReply(cmd);

      // Either way the command is complete; drop it and keep whatever arrived
      // behind it.
      consume(i + 1);
      return ok;
    }
    return false;
  }

  // "PC050" (power) and "MD4" / "MD04" (mode) replies, for CAT tune. Only
  // captured while a query is outstanding, so a radio that reports our own
  // carrier settings back cannot overwrite what we are meant to restore.
  bool parseTxSettingReply(const char* cmd) {
    if (!capturing_) return false;
    const char* pc = findUnprefixed(cmd, "PC");
    if (pc && isdigit(static_cast<unsigned char>(pc[2])) &&
        isdigit(static_cast<unsigned char>(pc[3])) &&
        isdigit(static_cast<unsigned char>(pc[4]))) {
      savedPower_ = static_cast<uint16_t>((pc[2] - '0') * 100 + (pc[3] - '0') * 10 + (pc[4] - '0'));
      havePower_ = true;
      return true;
    }
    const char* md = findUnprefixed(cmd, "MD");
    if (md && isalnum(static_cast<unsigned char>(md[2]))) {
      size_t len = strlen(md + 2);
      if (len > sizeof(savedModeText_) - 1) len = sizeof(savedModeText_) - 1;
      memcpy(savedModeText_, md + 2, len);
      savedModeText_[len] = '\0';
      haveMode_ = true;
      return true;
    }
    return false;
  }

  // strstr() that skips Flex "ZZ.." extended commands.
  static const char* findUnprefixed(const char* s, const char* key) {
    for (const char* p = strstr(s, key); p; p = strstr(p + 1, key)) {
      if (p - s >= 2 && p[-2] == 'Z' && p[-1] == 'Z') continue;
      return p;
    }
    return nullptr;
  }

  static char kenwoodMode(CarrierMode m) {
    switch (m) {
      case CarrierMode::Am: return '5';
      case CarrierMode::Cw: return '3';
      default: return '4';   // FM
    }
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

  // ---- Icom CI-V ----------------------------------------------------------------

  bool tryParseIcom() {
    for (size_t i = 0; i + 1 < bufIdx_; ++i) {
      if (buffer_[i] != kIcomPreamble || buffer_[i + 1] != kIcomPreamble) continue;

      for (size_t j = i + 4; j < bufIdx_; ++j) {
        if (buffer_[j] != kIcomEOM) continue;

        uint8_t toAddr = buffer_[i + 2];
        uint8_t fromAddr = buffer_[i + 3];
        uint8_t cmd = buffer_[i + 4];

        // Only accept frames aimed at us or broadcast, so a second controller
        // on the CI-V bus cannot drag us to the wrong frequency. Our own
        // transmissions echo back on the bus and are ignored here too.
        bool forUs = (toAddr == kIcomControllerAddr || toAddr == kIcomBroadcast) &&
                     fromAddr != kIcomControllerAddr;

        if (forUs && cfg_->icomAddress == 0) icomRadioAddr_ = fromAddr;

        size_t len = j - i;   // bytes before the EOM, from the first preamble
        // 0x00 (freq broadcast) and 0x03 (freq reply) carry 5 BCD bytes.
        if (forUs && (cmd == 0x00 || cmd == 0x03) && len >= 10) {
          uint32_t hz = bcdToHz(&buffer_[i + 5], 5);
          if (hz > 0) freqUpdated(hz, CatProtocol::Icom);
        } else if (forUs && capturing_ && cmd == 0x04 && len >= 6) {
          savedIcomMode_ = buffer_[i + 5];
          savedIcomFilter_ = (len >= 7) ? buffer_[i + 6] : 0x01;
          haveMode_ = true;
        } else if (forUs && capturing_ && cmd == 0x14 && len >= 8 && buffer_[i + 5] == 0x0A) {
          savedPower_ = static_cast<uint16_t>(bcdByte(buffer_[i + 6]) * 100 + bcdByte(buffer_[i + 7]));
          havePower_ = true;
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

  static uint8_t bcdByte(uint8_t b) { return static_cast<uint8_t>(((b >> 4) & 0x0F) * 10 + (b & 0x0F)); }

  // 0-255 as two BCD bytes, most significant first: 128 -> 0x01 0x28.
  static void levelToBcd(uint16_t level, uint8_t* out) {
    out[0] = static_cast<uint8_t>(((level / 1000) % 10) << 4 | ((level / 100) % 10));
    out[1] = static_cast<uint8_t>(((level / 10) % 10) << 4 | (level % 10));
  }

  static uint8_t icomMode(CarrierMode m) {
    switch (m) {
      case CarrierMode::Am: return 0x02;
      case CarrierMode::Cw: return 0x03;
      default: return 0x05;   // FM
    }
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

  // ---- Yaesu 5-byte binary --------------------------------------------------

  static uint8_t yaesuOldMode(CarrierMode m) {
    switch (m) {
      case CarrierMode::Am: return 0x04;
      case CarrierMode::Cw: return 0x02;
      default: return 0x08;   // FM
    }
  }

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
      bool locked = false;
      if (selected_ == CatProtocol::YaesuOld) {
        locked = true;
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
        locked = yaesuConfidence_ >= kYaesuLockFrames;
      }
      if (locked) {
        freqUpdated(hz, CatProtocol::YaesuOld);
        if (capturing_) {
          savedYaesuMode_ = buffer_[4];
          haveMode_ = true;
        }
      }
      consume(5);
      return true;
    }

    consume(1);
    return false;
  }
};

}  // namespace atu
