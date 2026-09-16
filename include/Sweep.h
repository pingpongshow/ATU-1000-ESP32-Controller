#pragma once
//
// Non-blocking SWR sweep.
//
// Fixes over the original: it can be stopped, it no longer blocks the whole
// firmware for five seconds when it finishes, it restores the radio's original
// VFO frequency afterwards instead of leaving it parked at the end of the
// sweep, and auto-tune is suppressed while it runs.
//

#include <Arduino.h>

#include <vector>

#include "Cat.h"
#include "Sensor.h"
#include "Settings.h"
#include "Types.h"

namespace atu {

class SweepEngine {
 public:
  void begin(Settings* settings, CatInterface* cat, BridgeSensor* sensor) {
    cfg_ = settings;
    cat_ = cat;
    sensor_ = sensor;
  }

  bool start(uint32_t startHz, uint32_t endHz, uint32_t stepHz, uint16_t dwellMs) {
    if (running_) return false;
    if (endHz <= startHz || stepHz == 0) return false;
    if (!cat_->enabled()) return false;

    // Snapshot the VFO so it can be put back when we are done.
    restoreHz_ = cat_->lastFrequency();

    startHz_ = startHz;
    endHz_ = endHz;
    stepHz_ = stepHz;
    dwellMs_ = dwellMs < 20 ? 20 : dwellMs;
    currentHz_ = startHz;
    points_.clear();
    points_.reserve((endHz - startHz) / stepHz + 2);
    running_ = true;
    pending_ = false;
    complete_ = false;
    lastStepMs_ = millis() - dwellMs_;
    return true;
  }

  void stop() {
    if (!running_) return;
    running_ = false;
    pending_ = false;
    restoreVfo();
  }

  bool isRunning() const { return running_; }

  // True once, when a sweep has just finished normally.
  bool takeComplete() {
    if (!complete_) return false;
    complete_ = false;
    return true;
  }

  void loop() {
    if (!running_) return;

    if (!pending_) {
      if (elapsed(lastStepMs_) < dwellMs_) return;
      // Command the frequency, then let the radio settle during the dwell
      // instead of blocking on delay().
      cat_->setFrequency(currentHz_);
      settleStartMs_ = millis();
      pending_ = true;
      return;
    }

    if (elapsed(settleStartMs_) < kRadioSettleMs) return;

    SensorReading r = sensor_->readAverage(8, 2);
    SweepPoint p;
    p.freqHz = currentHz_;
    p.swr = r.valid ? r.swr : 0.0f;
    p.powerW = r.powerW;
    points_.push_back(p);

    pending_ = false;
    lastStepMs_ = millis();

    if (currentHz_ >= endHz_ || points_.size() >= kMaxPoints) {
      running_ = false;
      complete_ = true;
      restoreVfo();
      return;
    }
    uint32_t next = currentHz_ + stepHz_;
    currentHz_ = (next > endHz_) ? endHz_ : next;
  }

  const std::vector<SweepPoint>& points() const { return points_; }

  uint8_t percent() const {
    if (endHz_ <= startHz_) return 100;
    uint32_t done = (currentHz_ > startHz_) ? (currentHz_ - startHz_) : 0;
    uint32_t total = endHz_ - startHz_;
    uint32_t p = (static_cast<uint64_t>(done) * 100U) / total;
    return static_cast<uint8_t>(p > 100 ? 100 : p);
  }

  bool findMinSwr(uint32_t& freqHz, float& swr) const {
    if (points_.empty()) return false;
    size_t minIdx = 0;
    float minSwr = 1e9f;
    bool any = false;
    for (size_t i = 0; i < points_.size(); ++i) {
      if (points_[i].swr < 1.0f) continue;   // no valid reading at that point
      if (points_[i].swr < minSwr) { minSwr = points_[i].swr; minIdx = i; any = true; }
    }
    if (!any) return false;
    freqHz = points_[minIdx].freqHz;
    swr = points_[minIdx].swr;
    return true;
  }

  uint32_t restoreFrequency() const { return restoreHz_; }

 private:
  static constexpr uint16_t kRadioSettleMs = 40;
  static constexpr size_t kMaxPoints = 512;

  Settings* cfg_ = nullptr;
  CatInterface* cat_ = nullptr;
  BridgeSensor* sensor_ = nullptr;

  bool running_ = false;
  bool pending_ = false;
  bool complete_ = false;
  uint32_t startHz_ = 0, endHz_ = 0, stepHz_ = 0, currentHz_ = 0;
  uint32_t restoreHz_ = 0;
  uint16_t dwellMs_ = 200;
  uint32_t lastStepMs_ = 0;
  uint32_t settleStartMs_ = 0;
  std::vector<SweepPoint> points_;

  void restoreVfo() {
    if (restoreHz_ >= 1000000UL && restoreHz_ <= 60000000UL) {
      cat_->setFrequency(restoreHz_);
    }
  }
};

}  // namespace atu
