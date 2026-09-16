#pragma once
//
// Display abstraction with runtime auto-detection.
//
// One firmware image drives either a PCF8574-backed HD44780 character LCD or a
// 128x64 SSD1306/SSD1309 OLED. The bus is probed at boot and the right driver
// is instantiated; nothing is allocated for the other one.
//
// Everything the UI needs is passed in a single UiModel, replacing the old
// ten-argument render() whose extra parameters the OLED build silently ignored
// (which is how it ended up never drawing the status line, L or C at all).
//

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <vector>

#include "Config.h"
#include "Settings.h"
#include "Thermal.h"
#include "Types.h"

namespace atu {

struct UiModel {
  uint32_t freqHz = 0;
  const char* bandName = "";
  bool catConnected = false;
  bool autoTune = false;
  RelayState state;
  SensorReading reading;
  float totalL = 0.0f;
  uint16_t totalC = 0;
  const char* status = "";
  bool powerWarning = false;
  bool powerOverload = false;
  bool protectHold = false;     // overload latched, waiting for RF to drop
  bool swrAlarm = false;
  bool tuning = false;
  uint8_t tunePercent = 0;
  bool tempValid = false;
  float tempC = 0.0f;
  ThermalLevel tempLevel = ThermalLevel::Ok;
  uint8_t antenna = 0;
  bool antennaSupported = false;
  bool wifiUp = false;
  float powerLimitW = 1000.0f;
};

// -----------------------------------------------------------------------------

class IDisplay {
 public:
  virtual ~IDisplay() {}
  virtual bool begin() = 0;
  virtual void clear() = 0;
  virtual void splash(const char* version) = 0;
  virtual void render(const UiModel& m) = 0;
  virtual void tuning(const UiModel& m) = 0;
  virtual void sweepGraph(const std::vector<SweepPoint>& pts) = 0;
  virtual void overload(const char* reason) = 0;
  virtual void message(const char* l1, const char* l2) = 0;
  virtual void setDim(bool dim) {}
  virtual void setBlank(bool blank) {}
  virtual void tickBurnIn() {}
  virtual const char* name() const = 0;
};

class NullDisplay : public IDisplay {
 public:
  bool begin() override { return true; }
  void clear() override {}
  void splash(const char*) override {}
  void render(const UiModel&) override {}
  void tuning(const UiModel&) override {}
  void sweepGraph(const std::vector<SweepPoint>&) override {}
  void overload(const char*) override {}
  void message(const char*, const char*) override {}
  const char* name() const override { return "none"; }
};

// -----------------------------------------------------------------------------
// Character LCD (HD44780 via PCF8574)
// -----------------------------------------------------------------------------

class LcdDisplay : public IDisplay {
 public:
  LcdDisplay(uint8_t addr, uint8_t cols, uint8_t rows)
      : addr_(addr), cols_(cols > 40 ? 40 : cols), rows_(rows > 4 ? 4 : rows),
        lcd_(addr, cols, rows) {}

  bool begin() override {
    Wire.beginTransmission(addr_);
    if (Wire.endTransmission() != 0) return false;
    lcd_.init();
    lcd_.backlight();
    loadHorizontalChars();
    lcd_.clear();
    return true;
  }

  void clear() override { lcd_.clear(); }

  void splash(const char* version) override {
    lcd_.clear();
    writeLine(0, "ATU-1000 ESP32-S3");
    char buf[24];
    snprintf(buf, sizeof(buf), "Firmware v%s", version);
    writeLine(1, buf);
    if (rows_ > 2) writeLine(2, "Initializing...");
  }

  void render(const UiModel& m) override {
    char buf[48];

    // Row 0: link + frequency + band + mode
    {
      char freq[12];
      formatFreq(m.freqHz, freq, sizeof(freq));
      if (cols_ >= 20) {
        snprintf(buf, sizeof(buf), "%s %s %-4s %c",
                 m.catConnected ? "CAT" : "---", freq, m.bandName,
                 m.autoTune ? 'A' : 'M');
      } else {
        snprintf(buf, sizeof(buf), "%s %s %c",
                 m.catConnected ? "CAT" : "---", freq, m.autoTune ? 'A' : 'M');
      }
      writeLine(0, buf);
    }

    // Row 1: SWR + power bargraph + watts
    writeLine(1, powerRow(m, buf, sizeof(buf)));

    if (rows_ > 2) {
      // Row 2: network state
      char topo = m.state.topology ? 'H' : 'L';
      char byp = m.state.bypass ? 'B' : ' ';
      if (m.tempValid && cols_ >= 20) {
        snprintf(buf, sizeof(buf), "%4.2fuH %4upF %c%c %2.0fC",
                 m.totalL, static_cast<unsigned>(m.totalC), topo, byp, m.tempC);
      } else {
        snprintf(buf, sizeof(buf), "%4.2fuH %4upF %c%c",
                 m.totalL, static_cast<unsigned>(m.totalC), topo, byp);
      }
      writeLine(2, buf);
    }

    if (rows_ > 3) {
      // Row 3: status, with any active alarm taking precedence. The old build
      // painted a full-screen overload page that the next 200 ms refresh
      // immediately wiped; the alarm now lives in the normal frame.
      const char* alarm = nullptr;
      if (m.protectHold) alarm = "!! PROTECT - RF ON";
      else if (m.powerOverload) alarm = "!! PROTECT - BYPASS";
      else if (m.tempLevel == ThermalLevel::Shutdown) alarm = "!! OVER TEMP - BYPASS";
      else if (m.tempLevel == ThermalLevel::Foldback) alarm = "! HOT - TX INHIBITED";
      else if (m.swrAlarm) alarm = "! HIGH SWR";
      else if (m.powerWarning) alarm = "! POWER WARNING";

      if (alarm) {
        writeLine(3, alarm);
      } else {
        char tail[24] = "";
        if (m.antennaSupported) snprintf(tail, sizeof(tail), " A%u", m.antenna + 1);
        snprintf(buf, sizeof(buf), "%s%s%s", m.status, tail, m.wifiUp ? " *" : "");
        writeLine(3, buf);
      }
    }
  }

  void tuning(const UiModel& m) override {
    char buf[16];
    lcdBar(m.tunePercent / 100.0f, cols_ >= 20 ? 14 : 10, buf, sizeof(buf));
    writeLine(0, "TUNING");
    char line[48];
    snprintf(line, sizeof(line), "%s %3u%%", buf, m.tunePercent);
    writeLine(1, line);
    if (rows_ > 2) {
      snprintf(line, sizeof(line), "SWR %.2f  %.0fW",
               m.reading.valid ? m.reading.swr : 0.0f, m.reading.powerW);
      writeLine(2, line);
    }
    if (rows_ > 3) writeLine(3, "Press TUNE to abort");
  }

  void sweepGraph(const std::vector<SweepPoint>& pts) override {
    if (pts.empty()) return;
    loadVerticalChars();
    lcd_.clear();

    char buf[48];
    snprintf(buf, sizeof(buf), "SWR %.2f-%.2fMHz",
             pts.front().freqHz / 1e6, pts.back().freqHz / 1e6);
    writeLine(0, buf);

    if (rows_ < 4) { loadHorizontalChars(); return; }

    // Rows 1..3, 8 pixel rows each => SWR 1.0-3.0 mapped to 0..24.
    const uint8_t graphCols = cols_;
    for (uint8_t col = 0; col < graphCols; ++col) {
      size_t idx = (pts.size() * col) / graphCols;
      if (idx >= pts.size()) idx = pts.size() - 1;
      int height = static_cast<int>((pts[idx].swr - 1.0f) / 2.0f * 24.0f);
      height = constrain(height, 0, 24);
      for (int row = 3; row >= 1; --row) {
        int rowHeight = constrain(height - (3 - row) * 8, 0, 8);
        lcd_.setCursor(col, row);
        if (rowHeight == 0) lcd_.write(' ');
        else if (rowHeight >= 8) lcd_.write(0xFF);
        else lcd_.write(static_cast<uint8_t>(rowHeight - 1));
      }
    }
    loadHorizontalChars();
  }

  void overload(const char* reason) override {
    lcd_.clear();
    writeLine(0, "!!! PROTECTION !!!");
    writeLine(1, reason);
    if (rows_ > 2) writeLine(2, "TX INHIBIT, BYPASS");
    if (rows_ > 3) writeLine(3, "'power reset' clears");
  }

  void message(const char* l1, const char* l2) override {
    lcd_.clear();
    writeLine(0, l1);
    if (l2 && rows_ > 1) writeLine(1, l2);
  }

  void setBlank(bool blank) override {
    if (blank == blank_) return;
    blank_ = blank;
    if (blank) { lcd_.noBacklight(); lcd_.noDisplay(); }
    else { lcd_.display(); lcd_.backlight(); }
  }

  void setDim(bool dim) override {
    // A PCF8574 backpack only has on/off backlight control.
    if (dim == dim_) return;
    dim_ = dim;
    if (dim) lcd_.noBacklight(); else lcd_.backlight();
  }

  const char* name() const override { return "LCD"; }

 private:
  uint8_t addr_, cols_, rows_;
  LiquidCrystal_I2C lcd_;
  bool blank_ = false;
  bool dim_ = false;
  bool horizontalChars_ = true;

  static void formatFreq(uint32_t hz, char* out, size_t len) {
    if (hz == 0) { snprintf(out, len, "--.---"); return; }
    snprintf(out, len, "%2lu.%03lu", static_cast<unsigned long>(hz / 1000000UL),
             static_cast<unsigned long>((hz / 1000UL) % 1000UL));
  }

  // Chars 0-4: a cell filled from the LEFT by 1..5 pixel columns.
  //
  // The original firmware only ever built bottom-up *vertical* bars and then
  // used them for the horizontal power bar, so the bargraph rendered as a row
  // of ragged bottom-edge dashes. HD44780 only has eight CGRAM slots, so the
  // two shapes are swapped in as needed.
  void loadHorizontalChars() {
    for (uint8_t i = 0; i < 5; ++i) {
      uint8_t pattern[8];
      uint8_t bits = static_cast<uint8_t>(0x1F << (4 - i)) & 0x1F;
      for (uint8_t row = 0; row < 8; ++row) pattern[row] = bits;
      lcd_.createChar(i, pattern);
    }
    horizontalChars_ = true;
  }

  // Chars 0-7: bar of height 1..8 pixels growing from the bottom.
  void loadVerticalChars() {
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t pattern[8];
      for (uint8_t row = 0; row < 8; ++row) {
        pattern[row] = (row >= (7 - i)) ? 0x1F : 0x00;
      }
      lcd_.createChar(i, pattern);
    }
    horizontalChars_ = false;
  }

  // Builds a bargraph into `out` using CGRAM chars 1..4 plus 0xFF for a full
  // cell. Char 0 is deliberately unused: it cannot travel through a C string.
  void lcdBar(float pct, uint8_t cells, char* out, size_t len) {
    pct = clampf(pct, 0.0f, 1.0f);
    int segments = static_cast<int>(pct * cells * 5.0f + 0.5f);
    size_t o = 0;
    for (uint8_t i = 0; i < cells && o + 1 < len; ++i) {
      int seg = segments - i * 5;
      char c;
      if (seg <= 0) c = ' ';
      else if (seg >= 5) c = static_cast<char>(0xFF);
      else c = static_cast<char>(seg);   // CGRAM 1..4
      out[o++] = c;
    }
    out[o] = '\0';
  }

  const char* powerRow(const UiModel& m, char* buf, size_t len) {
    char swr[8];
    if (!m.reading.valid) snprintf(swr, sizeof(swr), "S --");
    else if (m.reading.swr < 10.0f) snprintf(swr, sizeof(swr), "S%.1f", m.reading.swr);
    else snprintf(swr, sizeof(swr), "S>10");

    uint8_t barCells = (cols_ >= 20) ? 6 : 4;
    char bar[16];
    lcdBar(m.powerLimitW > 0 ? m.reading.powerW / m.powerLimitW : 0.0f,
           barCells, bar, sizeof(bar));

    char tail[8];
    if (m.powerOverload) snprintf(tail, sizeof(tail), "!OVL!");
    else if (m.powerWarning) snprintf(tail, sizeof(tail), " WARN");
    else if (m.reading.powerW >= 9999.0f) snprintf(tail, sizeof(tail), " HIGH");
    else snprintf(tail, sizeof(tail), "%4dW", static_cast<int>(m.reading.powerW));

    snprintf(buf, len, "%s %s%s", swr, bar, tail);
    return buf;
  }

  // Pads or truncates to the panel width so no stale characters survive.
  void writeLine(uint8_t row, const char* text) {
    if (row >= rows_) return;
    char line[41];
    size_t n = 0;
    for (; text[n] != '\0' && n < cols_ && n < sizeof(line) - 1; ++n) line[n] = text[n];
    for (; n < cols_ && n < sizeof(line) - 1; ++n) line[n] = ' ';
    line[n] = '\0';
    lcd_.setCursor(0, row);
    // Byte at a time: the class declares write(uint8_t) only, which hides
    // Print's buffer overload. Going through print() would also be wrong, since
    // it stops at the first NUL and the CGRAM bargraph glyphs are raw bytes.
    for (size_t i = 0; i < n; ++i) lcd_.write(static_cast<uint8_t>(line[i]));
  }
};

// -----------------------------------------------------------------------------
// 128x64 OLED (SSD1306 / SSD1309)
// -----------------------------------------------------------------------------

class OledDisplay : public IDisplay {
 public:
  OledDisplay(uint8_t addr, uint8_t driver, uint8_t contrast, int scl, int sda)
      : addr_(addr), driver_(driver), contrast_(contrast), scl_(scl), sda_(sda) {}

  ~OledDisplay() override { delete u8g2_; }

  bool begin() override {
    Wire.beginTransmission(addr_);
    if (Wire.endTransmission() != 0) return false;

    switch (driver_) {
      case 1:
        u8g2_ = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(
            U8G2_R0, U8X8_PIN_NONE, scl_, sda_);
        break;
      case 2:
        u8g2_ = new U8G2_SSD1309_128X64_NONAME0_F_HW_I2C(
            U8G2_R0, U8X8_PIN_NONE, scl_, sda_);
        break;
      default:
        u8g2_ = new U8G2_SSD1309_128X64_NONAME2_F_HW_I2C(
            U8G2_R0, U8X8_PIN_NONE, scl_, sda_);
        break;
    }
    if (!u8g2_) return false;

    u8g2_->setI2CAddress(static_cast<uint8_t>(addr_ << 1));
    u8g2_->begin();
    u8g2_->setContrast(contrast_);
    u8g2_->setPowerSave(0);
    u8g2_->clearBuffer();
    u8g2_->sendBuffer();
    return true;
  }

  void clear() override {
    if (!u8g2_) return;
    u8g2_->clearBuffer();
    u8g2_->sendBuffer();
  }

  void splash(const char* version) override {
    if (!u8g2_) return;
    u8g2_->clearBuffer();
    u8g2_->setFont(u8g2_font_helvB14_tr);
    centered(20, "ATU-1000");
    u8g2_->setFont(u8g2_font_helvR08_tr);
    centered(35, "ESP32-S3 Controller");
    u8g2_->drawHLine(10, 42, W - 20);
    u8g2_->setFont(u8g2_font_6x10_tr);
    char buf[24];
    snprintf(buf, sizeof(buf), "v%s  starting...", version);
    centered(56, buf);
    u8g2_->sendBuffer();
  }

  void render(const UiModel& m) override {
    if (!u8g2_) return;
    u8g2_->clearBuffer();

    // ---- Frequency, right-sized so it can never collide with the unit ----
    char freqBuf[12];
    if (m.freqHz > 0) {
      snprintf(freqBuf, sizeof(freqBuf), "%lu.%03lu",
               static_cast<unsigned long>(m.freqHz / 1000000UL),
               static_cast<unsigned long>((m.freqHz / 1000UL) % 1000UL));
    } else {
      snprintf(freqBuf, sizeof(freqBuf), "--.---");
    }

    // Measure and step down a font rather than trusting hardcoded offsets.
    u8g2_->setFont(u8g2_font_logisoso20_tn);
    int fw = u8g2_->getStrWidth(freqBuf);
    if (fw > W - 30) {
      u8g2_->setFont(u8g2_font_logisoso16_tn);
      fw = u8g2_->getStrWidth(freqBuf);
    }
    u8g2_->drawStr(x(0), y(22), freqBuf);

    u8g2_->setFont(u8g2_font_5x7_tr);
    u8g2_->drawStr(x(fw + 3), y(15), "MHz");
    if (m.bandName && m.bandName[0]) {
      u8g2_->drawStr(x(fw + 3), y(23), m.bandName);
    }

    // Wi-Fi / CAT corner markers.
    u8g2_->setFont(u8g2_font_4x6_tr);
    if (m.wifiUp) u8g2_->drawStr(x(W - 10), y(6), "WiFi");
    if (m.tempValid) {
      char t[10];
      snprintf(t, sizeof(t), "%.0fC", m.tempC);
      int tw = u8g2_->getStrWidth(t);
      u8g2_->drawStr(x(W - tw), y(m.wifiUp ? 13 : 6), t);
    }

    // ---- SWR bar ----
    const int barW = 78;
    drawBar(0, 26, barW, 8, m.reading.valid
                                ? clampf((m.reading.swr - 1.0f) / 2.0f, 0.0f, 1.0f)
                                : 0.0f,
            false);
    u8g2_->setFont(u8g2_font_helvB10_tr);
    char swrBuf[8];
    if (!m.reading.valid) snprintf(swrBuf, sizeof(swrBuf), "--");
    else if (m.reading.swr < 10.0f) snprintf(swrBuf, sizeof(swrBuf), "%.1f", m.reading.swr);
    else snprintf(swrBuf, sizeof(swrBuf), ">10");
    u8g2_->drawStr(x(barW + 4), y(34), swrBuf);

    // ---- Power bar ----
    float pct = (m.powerLimitW > 0) ? m.reading.powerW / m.powerLimitW : 0.0f;
    drawBar(0, 37, barW, 8, clampf(pct, 0.0f, 1.0f), m.powerWarning || m.powerOverload);
    char pwrBuf[10];
    if (m.reading.powerW >= 1000.0f) snprintf(pwrBuf, sizeof(pwrBuf), "%.1fk", m.reading.powerW / 1000.0f);
    else snprintf(pwrBuf, sizeof(pwrBuf), "%dW", static_cast<int>(m.reading.powerW));
    u8g2_->drawStr(x(barW + 4), y(45), pwrBuf);

    // ---- L / C readout ----
    u8g2_->setFont(u8g2_font_4x6_tr);
    char lc[24];
    if (m.state.bypass) {
      snprintf(lc, sizeof(lc), "BYPASS");
    } else {
      snprintf(lc, sizeof(lc), "%.2fuH %upF %s", m.totalL,
               static_cast<unsigned>(m.totalC), m.state.topology ? "HiZ" : "LoZ");
    }
    u8g2_->drawStr(x(0), y(52), lc);

    // ---- Status row ----
    u8g2_->drawHLine(x(0), y(54), W);
    u8g2_->setFont(u8g2_font_5x7_tr);

    const char* alarm = nullptr;
    if (m.protectHold) alarm = "! PROTECT - RF STILL ON";
    else if (m.powerOverload) alarm = "! PROTECT - BYPASS";
    else if (m.tempLevel == ThermalLevel::Shutdown) alarm = "! OVER TEMP - BYPASS";
    else if (m.tempLevel == ThermalLevel::Foldback) alarm = "! HOT - TX INHIBIT";
    else if (m.swrAlarm) alarm = "! HIGH SWR";
    else if (m.powerWarning) alarm = "! POWER WARNING";

    if (alarm) {
      // Invert the row so it is unmistakable at a glance.
      u8g2_->drawBox(x(0), y(56), W, 8);
      u8g2_->setDrawColor(0);
      u8g2_->drawStr(x(1), y(63), alarm);
      u8g2_->setDrawColor(1);
    } else {
      u8g2_->drawStr(x(0), y(63), m.status);
      int rx = W;
      if (m.antennaSupported) {
        char a[6];
        snprintf(a, sizeof(a), "A%u", m.antenna + 1);
        rx -= u8g2_->getStrWidth(a) + 2;
        u8g2_->drawStr(x(rx), y(63), a);
      }
      const char* mode = m.autoTune ? "AUTO" : "MAN";
      rx -= u8g2_->getStrWidth(mode) + 3;
      u8g2_->drawStr(x(rx), y(63), mode);
      if (m.catConnected) {
        rx -= u8g2_->getStrWidth("CAT") + 3;
        u8g2_->drawStr(x(rx), y(63), "CAT");
      }
    }

    u8g2_->sendBuffer();
  }

  void tuning(const UiModel& m) override {
    if (!u8g2_) return;
    u8g2_->clearBuffer();

    u8g2_->setFont(u8g2_font_helvB14_tr);
    centered(18, "TUNING");

    u8g2_->drawFrame(x(8), y(24), 112, 12);
    int barW = (static_cast<int>(m.tunePercent) * 108) / 100;
    if (barW > 0) u8g2_->drawBox(x(10), y(26), barW, 8);

    u8g2_->setFont(u8g2_font_6x10_tr);
    char buf[32];
    snprintf(buf, sizeof(buf), "%u%%   SWR %.2f", m.tunePercent,
             m.reading.valid ? m.reading.swr : 0.0f);
    centered(48, buf);

    u8g2_->setFont(u8g2_font_4x6_tr);
    centered(62, "press TUNE to abort");
    u8g2_->sendBuffer();
  }

  void sweepGraph(const std::vector<SweepPoint>& pts) override {
    if (!u8g2_ || pts.empty()) return;
    u8g2_->clearBuffer();

    size_t minIdx = 0;
    float minSwr = 99.0f, maxSwr = 1.0f;
    for (size_t i = 0; i < pts.size(); ++i) {
      if (pts[i].swr < minSwr) { minSwr = pts[i].swr; minIdx = i; }
      if (pts[i].swr > maxSwr) maxSwr = pts[i].swr;
    }
    // Autoscale the Y axis instead of always assuming 1..5.
    float top = clampf(maxSwr * 1.1f, 2.0f, 10.0f);

    const int gl = 12, gr = W - 2, gt = 10, gb = 52;
    const int gw = gr - gl, gh = gb - gt;

    u8g2_->setFont(u8g2_font_5x7_tr);
    char title[28];
    snprintf(title, sizeof(title), "%.2f-%.2f MHz",
             pts.front().freqHz / 1e6, pts.back().freqHz / 1e6);
    u8g2_->drawStr(x(0), y(7), title);

    u8g2_->drawHLine(x(gl), y(gb), gw);
    u8g2_->drawVLine(x(gl), y(gt), gh);

    u8g2_->setFont(u8g2_font_4x6_tr);
    char lbl[8];
    snprintf(lbl, sizeof(lbl), "%.0f", top);
    u8g2_->drawStr(x(0), y(gt + 5), lbl);
    u8g2_->drawStr(x(0), y(gb), "1");

    for (size_t i = 0; i < pts.size(); ++i) {
      int px = gl + static_cast<int>((i * gw) / pts.size());
      float s = std::min(pts[i].swr, top);
      int py = gb - static_cast<int>((s - 1.0f) / (top - 1.0f) * gh);
      py = constrain(py, gt, gb);
      u8g2_->drawVLine(x(px), y(py), gb - py);
    }

    int mx = gl + static_cast<int>((minIdx * gw) / pts.size());
    int my = gb - static_cast<int>((std::min(minSwr, top) - 1.0f) / (top - 1.0f) * gh);
    my = constrain(my, gt, gb);
    u8g2_->drawCircle(x(mx), y(my), 3);

    u8g2_->setFont(u8g2_font_5x7_tr);
    char info[32];
    snprintf(info, sizeof(info), "min %.2f @ %.3f MHz", minSwr, pts[minIdx].freqHz / 1e6);
    u8g2_->drawStr(x(0), y(62), info);
    u8g2_->sendBuffer();
  }

  void overload(const char* reason) override {
    if (!u8g2_) return;
    u8g2_->clearBuffer();
    u8g2_->drawBox(0, 0, W, H);
    u8g2_->setDrawColor(0);
    u8g2_->setFont(u8g2_font_helvB14_tr);
    centered(26, "! PROTECT !");
    u8g2_->setFont(u8g2_font_helvR10_tr);
    centered(44, reason);
    u8g2_->setFont(u8g2_font_5x7_tr);
    centered(58, "TX INHIBIT + BYPASS");
    u8g2_->setDrawColor(1);
    u8g2_->sendBuffer();
  }

  void message(const char* l1, const char* l2) override {
    if (!u8g2_) return;
    u8g2_->clearBuffer();
    u8g2_->setFont(u8g2_font_helvB10_tr);
    centered(28, l1);
    if (l2) {
      u8g2_->setFont(u8g2_font_6x10_tr);
      centered(46, l2);
    }
    u8g2_->sendBuffer();
  }

  void setDim(bool dim) override {
    if (!u8g2_ || dim == dim_) return;
    dim_ = dim;
    u8g2_->setContrast(dim ? 1 : contrast_);
  }

  void setBlank(bool blank) override {
    if (!u8g2_ || blank == blank_) return;
    blank_ = blank;
    u8g2_->setPowerSave(blank ? 1 : 0);
  }

  // OLEDs burn in. Walk the whole frame around a few pixels so a static layout
  // does not etch itself into the panel.
  void tickBurnIn() override {
    shift_ = static_cast<uint8_t>((shift_ + 1) & 0x07);
  }

  const char* name() const override { return "OLED"; }

 private:
  static constexpr int W = 128;
  static constexpr int H = 64;

  uint8_t addr_, driver_, contrast_;
  int scl_, sda_;
  U8G2* u8g2_ = nullptr;
  bool dim_ = false;
  bool blank_ = false;
  uint8_t shift_ = 0;

  // Burn-in offset: 0..3 px horizontally, 0..1 px vertically.
  int x(int v) const { return v + (shift_ & 0x03); }
  int y(int v) const { return v + ((shift_ >> 2) & 0x01); }

  void centered(int yy, const char* s) {
    int w = u8g2_->getStrWidth(s);
    int xx = (W - w) / 2;
    if (xx < 0) xx = 0;
    u8g2_->drawStr(x(xx), y(yy), s);
  }

  // hatched = draw as vertical stripes, used to make an alarming bar visually
  // distinct from a normal solid one on a monochrome panel.
  void drawBar(int bx, int by, int bw, int bh, float pct, bool hatched) {
    u8g2_->drawFrame(x(bx), y(by), bw, bh);
    int fill = static_cast<int>(clampf(pct, 0.0f, 1.0f) * (bw - 2));
    if (fill <= 0) return;
    if (hatched) {
      for (int i = 0; i < fill; i += 2) u8g2_->drawVLine(x(bx + 1 + i), y(by + 1), bh - 2);
    } else {
      u8g2_->drawBox(x(bx + 1), y(by + 1), fill, bh - 2);
    }
  }
};

// -----------------------------------------------------------------------------
// Detection + lifecycle
// -----------------------------------------------------------------------------

class DisplayManager {
 public:
  void begin(Settings* settings, Print& log) {
    cfg_ = settings;
    delete impl_;
    impl_ = nullptr;

    if (kPins.i2cSda < 0 || kPins.i2cScl < 0) {
      log.println("  I2C pins disabled - running headless");
      impl_ = new NullDisplay();
      impl_->begin();
      return;
    }

    Wire.begin(kPins.i2cSda, kPins.i2cScl);
    Wire.setTimeOut(50);
    delay(50);  // let a freshly powered panel come up

    DisplayKind want = static_cast<DisplayKind>(cfg_->displayType);
    uint8_t addr = 0;

    if (want == DisplayKind::Auto) {
      // OLEDs live at 0x3C/0x3D. That range overlaps PCF8574A (0x38-0x3F), so
      // OLED wins the tie; use `set disptype 2` to force an LCD backpack that
      // happens to be strapped to one of those two addresses.
      if (probe(0x3C)) { want = DisplayKind::Oled; addr = 0x3C; }
      else if (probe(0x3D)) { want = DisplayKind::Oled; addr = 0x3D; }
      else {
        for (uint8_t a = 0x20; a <= 0x27 && !addr; ++a) if (probe(a)) addr = a;
        if (!addr) for (uint8_t a = 0x38; a <= 0x3F && !addr; ++a) if (probe(a)) addr = a;
        want = addr ? DisplayKind::Lcd : DisplayKind::None;
      }
    } else if (want == DisplayKind::Oled) {
      addr = cfg_->oledAddress;
    } else if (want == DisplayKind::Lcd) {
      addr = cfg_->lcdAddress;
    }

    if (want == DisplayKind::Oled) {
      log.printf("  OLED detected at 0x%02X (driver %u)\n", addr, cfg_->oledDriver);
      OledDisplay* d = new OledDisplay(addr, cfg_->oledDriver, cfg_->oledContrast,
                                       kPins.i2cScl, kPins.i2cSda);
      if (d->begin()) {
        impl_ = d;
        cfg_->oledAddress = addr;
        detected_ = DisplayKind::Oled;
      } else {
        log.println("  OLED failed to initialise");
        delete d;
      }
    } else if (want == DisplayKind::Lcd) {
      log.printf("  LCD detected at 0x%02X (%ux%u)\n", addr, cfg_->lcdCols, cfg_->lcdRows);
      LcdDisplay* d = new LcdDisplay(addr, cfg_->lcdCols, cfg_->lcdRows);
      if (d->begin()) {
        impl_ = d;
        cfg_->lcdAddress = addr;
        detected_ = DisplayKind::Lcd;
      } else {
        log.println("  LCD failed to initialise");
        delete d;
      }
    }

    if (!impl_) {
      log.println("  No display found - running headless");
      log.printf("  I2C: SDA=GPIO%d SCL=GPIO%d. Try 'i2cscan'.\n",
                 kPins.i2cSda, kPins.i2cScl);
      impl_ = new NullDisplay();
      impl_->begin();
      detected_ = DisplayKind::None;
    }
    lastActivityMs_ = millis();
    lastShiftMs_ = millis();
  }

  IDisplay& operator*() const { return *impl_; }
  IDisplay* operator->() const { return impl_; }
  bool present() const { return detected_ != DisplayKind::None; }
  DisplayKind kind() const { return detected_; }
  const char* kindName() const { return impl_ ? impl_->name() : "none"; }

  // Any user or RF activity wakes the panel back up.
  void noteActivity() {
    lastActivityMs_ = millis();
    if (blanked_) { impl_->setBlank(false); blanked_ = false; }
    if (dimmed_) { impl_->setDim(false); dimmed_ = false; }
  }

  void loop() {
    if (!impl_) return;
    uint32_t idleSec = elapsed(lastActivityMs_) / 1000U;

    if (cfg_->blankAfterSec && idleSec >= cfg_->blankAfterSec) {
      if (!blanked_) { impl_->setBlank(true); blanked_ = true; }
    } else if (cfg_->dimAfterSec && idleSec >= cfg_->dimAfterSec) {
      if (!dimmed_) { impl_->setDim(true); dimmed_ = true; }
    }

    if (cfg_->burnInShiftSec &&
        elapsed(lastShiftMs_) >= cfg_->burnInShiftSec * 1000U) {
      lastShiftMs_ = millis();
      impl_->tickBurnIn();
    }
  }

  bool blanked() const { return blanked_; }

  static bool probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

 private:
  Settings* cfg_ = nullptr;
  IDisplay* impl_ = nullptr;
  DisplayKind detected_ = DisplayKind::None;
  bool dimmed_ = false;
  bool blanked_ = false;
  uint32_t lastActivityMs_ = 0;
  uint32_t lastShiftMs_ = 0;
};

}  // namespace atu
