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

struct App {
  SettingsStore settingsStore;
  RelayController relays;
  BridgeSensor sensor;
  Thermal thermal;
  CatInterface cat;
  MemoryStore memory;
  DisplayManager display;
  StatusLed led;
  PowerProtection powerProt;
  TuningEngine tuner;
  SweepEngine sweep;

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
  char protectionReason[24] = "";

  // Set when a memory entry has already been tried for the current frequency,
  // so auto-retune escalates to a real tune instead of re-applying a stale
  // entry forever.
  uint32_t memTriedBin = UINT32_MAX;

  Settings& cfg() { return settingsStore.get(); }
};

extern App gApp;

// Implemented in main.cpp; used by both the console and the web UI.
void applyRelayState(const RelayState& s);
void setTxRequest(bool active);
bool beginTune(bool force);
void engageProtection(const char* reason);
void clearProtection();
void saveRuntimeState();
void selectAntenna(uint8_t index);
const char* displayKindName();

}  // namespace atu
