#pragma once
//
// Global application state.
//
// Declared here and defined once in main.cpp, so the command parser and the
// web UI can share it without either one having to be included in a particular
// order.
//

#include <Arduino.h>
#include <Preferences.h>

#include "Cat.h"
#include "CatTune.h"
#include "Config.h"
#include "Controls.h"
#include "Display.h"
#include "MemStore.h"
#include "Relays.h"
#include "Sensor.h"
#include "Settings.h"
#include "Sweep.h"
#include "Thermal.h"
#include "Tuner.h"
#include "Types.h"

namespace atu {

struct RuntimeState {
  uint8_t version;
  uint8_t lMask;
  uint8_t cMask;
  uint8_t flags;
  uint8_t antenna;
  uint32_t checksum;
} __attribute__((packed));

constexpr uint8_t kRuntimeStateVersion = 2;

enum class ProtectState : uint8_t {
  Clear,
  WaitRfDrop,   // TX inhibit asserted, relays untouched until RF falls
  Bypassed,     // RF fell, bypass engaged
};

struct App {
  SettingsStore settingsStore;
  RelayController relays;
  TxLine tx;
  BridgeSensor sensor;
  Thermal thermal;
  CatInterface cat;
  MemoryStore memory;
  DisplayManager display;
  StatusLed led;
  PowerProtection powerProt;
  TuningEngine tuner;
  SweepEngine sweep;
  CatTuneSequencer catTune;

  DebouncedButton tuneButton;
  DebouncedButton bypassButton;
  DebouncedButton autoButton;

  Preferences statePrefs;

  RelayState state;
  SensorReading reading;
  StatusText status;

  uint32_t currentFreqHz = 0;
  uint32_t lastSensorMs = 0;
  uint32_t lastDisplayMs = 0;
  uint32_t lastTuneMs = 0;
  uint32_t lastSwrHighMs = 0;
  uint32_t sweepResultShownMs = 0;
  uint32_t protectionShownMs = 0;
  uint32_t bootMs = 0;

  bool showingSweep = false;
  bool showingProtection = false;
  bool protectionLatched = false;
  ProtectState protectState = ProtectState::Clear;
  uint32_t protectSinceMs = 0;
  bool protectHoldAnnounced = false;
  char protectionReason[24] = "";

  // A relay change requested while RF was above pwrmax, applied once it drops.
  bool relayPending = false;
  RelayState pendingState;

  // Release of the TX request line after a tune, without blocking the loop.
  bool txReleasePending = false;
  uint32_t txReleaseAtMs = 0;   // when the trail period started

  // Relays only move once peak RF has stayed at or below pwrmax this long.
  uint32_t rfQuietSinceMs = 0;

  bool swrAlarm = false;
  uint32_t swrHighSinceMs = 0;
  uint32_t swrLowSinceMs = 0;

  // Set when a memory entry has already been tried for the current frequency,
  // so auto-retune escalates to a real tune instead of re-applying a stale
  // entry forever.
  uint32_t memTriedBin = UINT32_MAX;

  Settings& cfg() { return settingsStore.get(); }
};

extern App gApp;

// Implemented in main.cpp; used by both the console and the web UI.

// Applies now if RF is at or below pwrmax, otherwise defers until it drops.
// Returns true if the relays moved immediately.
bool applyRelayState(const RelayState& s);
// The state the relays are at, or are waiting to move to.
RelayState desiredRelayState();
bool rfSafeToSwitch();
bool tuneAllowed(bool force);
bool beginTune(bool force);
bool startTuneFromButton();
void abortActivity(const char* status);
void engageProtection(const char* reason);
void clearProtection();
void saveRuntimeState();
void selectAntenna(uint8_t index);
void setStatus(const char* text);
const char* displayKindName();

}  // namespace atu
