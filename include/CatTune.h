#pragma once
//
// One-button tune through CAT.
//
//   Query     read the radio's power and mode so they can be put back
//   Carrier   set `catpwr` watts and the `catmode` carrier mode
//   Key       transmit, and wait for RF to show up on the bridge
//   Tune      run a normal tune
//   Unkey     stop transmitting
//   Restore   put power and mode back
//
// Every exit path, including protection, an abort and the `catmaxms` key-down
// limit, unkeys the radio and restores its settings. FlexRadio uses its own
// TUNE function and TUNE power instead of a mode change.
//

#include <Arduino.h>

#include "Cat.h"
#include "Sensor.h"
#include "Settings.h"
#include "Tuner.h"
#include "Types.h"

namespace atu {

class CatTuneSequencer {
 public:
  using BeginTuneFn = bool (*)(bool force);
  using TuneAllowedFn = bool (*)(bool force);   // sets the status when refusing
  using StatusFn = void (*)(const char* text);

  void begin(Settings* settings, CatInterface* cat, TuningEngine* tuner,
             BridgeSensor* sensor, BeginTuneFn beginTune, TuneAllowedFn tuneAllowed,
             StatusFn status) {
    cfg_ = settings;
    cat_ = cat;
    tuner_ = tuner;
    sensor_ = sensor;
    beginTune_ = beginTune;
    tuneAllowed_ = tuneAllowed;
    status_ = status;
  }

  bool active() const { return state_ != State::Idle; }

  // Returns false (with a status message) if CAT tune cannot start.
  bool start() {
    if (active()) return false;
    // Everything that would make beginTune refuse is checked before the radio
    // is touched, so it is never keyed only to be told no.
    if (!tuneAllowed_(false)) return false;
    if (!cat_->canKey()) {
      status_("CAT: NO RADIO");
      return false;
    }
    SensorReading r = sensor_->latest();
    if (r.valid && r.powerW >= cfg_->minTunePowerW) {
      // Already transmitting: nothing to key, just tune.
      return beginTune_(false);
    }
    carrierSet_ = false;
    keyed_ = false;
    queryResent_ = false;
    cat_->queryTxSettings();
    enter(State::Query);
    status_("CAT TUNE");
    return true;
  }

  // Normal abort: unkey, restore, and leave `why` on the status line.
  void abort(const char* why) {
    if (!active()) return;
    if (why) status_(why);
    if (tuner_->running()) tuner_->abort();
    unkeyAndRestore();
  }

  // Protection path: unkey immediately, before anything else happens.
  void emergencyUnkey() {
    if (!active()) return;
    if (tuner_->running()) tuner_->abort();
    unkeyAndRestore();
  }

  void loop() {
    if (state_ == State::Idle) return;

    if (keyed_ && elapsed(keyMs_) > cfg_->catTuneMaxMs) {
      abort("CAT TX LIMIT");
      return;
    }

    switch (state_) {
      case State::Query:
        if (cat_->txSettingsKnown()) {
          cat_->setCarrier(cfg_->catTunePowerW, static_cast<CarrierMode>(cfg_->catTuneMode));
          carrierSet_ = true;
          enter(State::Carrier);
        } else if (!queryResent_ && elapsed(stateMs_) > 400) {
          queryResent_ = true;
          cat_->queryTxSettings();
        } else if (elapsed(stateMs_) > 1000) {
          abort("CAT NO REPLY");
        }
        break;

      case State::Carrier:
        if (elapsed(stateMs_) < kCarrierSettleMs) break;
        cat_->setKey(true);
        keyed_ = true;
        keyMs_ = millis();
        rfSeenMs_ = 0;
        enter(State::Key);
        break;

      case State::Key: {
        SensorReading r = sensor_->latest();
        if (r.valid && r.powerW >= cfg_->minTunePowerW) {
          if (rfSeenMs_ == 0) rfSeenMs_ = millis() | 1;
          // Let ALC settle before judging the power level.
          if (elapsed(rfSeenMs_) < kRfSettleMs) break;
          if (r.powerW > cfg_->maxTunePowerW) {
            abort("CAT PWR HIGH");
            break;
          }
          if (!beginTune_(false)) {
            abort(nullptr);   // beginTune has already set the reason
            break;
          }
          enter(State::Tune);
        } else {
          rfSeenMs_ = 0;
          if (elapsed(stateMs_) > kKeyTimeoutMs) abort("CAT NO RF");
        }
        break;
      }

      case State::Tune:
        if (!tuner_->running()) unkeyAndRestore();
        break;

      case State::Unkey: {
        if (elapsed(stateMs_) < kUnkeySettleMs) break;
        // Confirm on the bridge that the carrier really stopped; a lost or
        // garbled unkey would otherwise leave the radio transmitting.
        SensorReading r = sensor_->latest();
        if (r.valid && r.powerW >= cfg_->minTunePowerW) {
          if (unkeyRetries_ < kUnkeyRetries) {
            ++unkeyRetries_;
            cat_->setKey(false);
            enter(State::Unkey);
            break;
          }
          status_("CAT UNKEY FAILED");
          Serial.println("*** CAT tune: radio still transmitting after unkey - check the radio ***");
        }
        if (carrierSet_) cat_->restoreTxSettings();
        carrierSet_ = false;
        enter(State::Idle);
        break;
      }

      default:
        break;
    }
  }

 private:
  enum class State : uint8_t { Idle, Query, Carrier, Key, Tune, Unkey };

  static constexpr uint32_t kCarrierSettleMs = 250;
  static constexpr uint32_t kRfSettleMs = 150;
  static constexpr uint32_t kKeyTimeoutMs = 3000;
  static constexpr uint32_t kUnkeySettleMs = 250;
  static constexpr uint8_t kUnkeyRetries = 3;

  Settings* cfg_ = nullptr;
  CatInterface* cat_ = nullptr;
  TuningEngine* tuner_ = nullptr;
  BridgeSensor* sensor_ = nullptr;
  BeginTuneFn beginTune_ = nullptr;
  TuneAllowedFn tuneAllowed_ = nullptr;
  uint8_t unkeyRetries_ = 0;
  StatusFn status_ = nullptr;

  State state_ = State::Idle;
  uint32_t stateMs_ = 0;
  uint32_t keyMs_ = 0;
  uint32_t rfSeenMs_ = 0;
  bool keyed_ = false;
  bool carrierSet_ = false;
  bool queryResent_ = false;

  void enter(State s) {
    state_ = s;
    stateMs_ = millis();
  }

  void unkeyAndRestore() {
    bool mayBeTransmitting = keyed_ || state_ == State::Key || state_ == State::Tune ||
                             state_ == State::Unkey;
    unkeyRetries_ = 0;
    if (keyed_) {
      cat_->setKey(false);
      keyed_ = false;
    }
    // Pass through Unkey whenever the radio may have been keyed, so the carrier
    // is verified gone even when no settings were changed (Flex).
    enter(mayBeTransmitting || carrierSet_ ? State::Unkey : State::Idle);
  }
};

}  // namespace atu
