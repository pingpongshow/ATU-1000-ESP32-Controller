# Changelog

## v2.1

After flashing this version, recalibrate the bridge and measure the relay
settle time. See [Upgrading to v2.1](README.md#upgrading-to-v21) for the
step-by-step procedure.

### Safety

- **Overload protection runs in its own task.** A FreeRTOS task on core 0 owns
  the SWR bridge ADC and samples it about once a millisecond. Two consecutive
  samples at or above `pwrlimit` assert the TX inhibit line within about 2 ms.
  This no longer depends on the main loop, which in v2.0 checked power only
  every 100 ms.
- **No hot switching during protection.** v2.0 opened the relay bank the moment
  it saw an overload, with the kilowatt still present. Protection now asserts
  TX inhibit, stops any tune or sweep without moving the relays, and waits for
  RF to drop before engaging bypass. If RF is still present after `protwait` ms
  it keeps the current match and shows `PROTECT - RF ON`. A sustained overload
  trips again immediately after `power reset`.
- **Relay changes wait for low RF.** Outside a tune, the relays only move once
  peak forward power has stayed at or below `pwrmax` for 300 ms. This covers
  memory recall on a CAT frequency change, `freq`, `l`, `c`, `topo`, `bypass`,
  the BYPASS button and antenna changes. Requests made meanwhile show
  `RF HIGH - WAITING` and apply when RF drops. In v2.0, auto-tune memory recall
  and CAT frequency changes switched relays at any power.
- **No more transmission chopping.** v2.0 auto-tune asserted TX request and only
  then refused because power was too high, interrupting the operator every two
  seconds. The power check now happens first, and auto-tune shows `RETUNE: <nW`
  instead.
- **Protection keeps bypass engaged.** While protection is active, nothing can
  take the network out of bypass; requested changes apply after `power reset`.
- **High-SWR alarm.** `swrmax` existed in v2.0 but did nothing. SWR above it for
  half a second while transmitting now shows `! HIGH SWR` and blinks the LED.
- **Task watchdog** on both the main loop and the sampling task (5 s).
- **Wi-Fi starts in the background.** `wifi on` used to block the whole firmware
  for up to 15 s.
- **Relay state is restored before the relay bank is driven at boot**, instead
  of dropping every relay and then picking them up again.
- The TX request trail no longer blocks the loop.
- **Pulsed "latching relay" mode removed** (`relaymode`, `latchms`). Each relay
  has a single low-side driver, which cannot deliver the reset pulse a latching
  relay needs, so the mode could never have worked.
- The ESP32's internal die temperature is now displayed only. It measures the
  chip, not the RF parts, and could trigger false foldback with Wi-Fi on.

### Calibration

- **`cal zero`** (new) measures both detector offsets with no RF.
- **`cal rev <swr>`** (new) solves `revscale` from a known mismatch, e.g. a
  100 ohm load for SWR 2.0. v2.0 never calibrated `revscale`.
- **`cal rev`** with no argument now only reports bridge balance into a 50 ohm
  load. In v2.0 it wrote that reading into `revoffset`. That residual is bridge
  imbalance and diode leakage, which scale with power, so SWR came out wrong at
  every other power level. **If you used v2.0's `cal rev`, recalibrate.**
- **`settle test`** (new) measures how long a relay change takes to show up at
  the detectors and recommends a `settle` value.

### Tuning

- **Model-based search.** The tuner estimates the antenna's complex impedance
  from the first dozen measurements, including which way round the K2 topology
  relay is wired, then measures the relay combinations the model predicts are
  best. When no frequency is known it estimates the band too.
- **Value-order pattern search** replaces bit flipping. It steps L and C in
  order of real value, with diagonal moves and a repeat of any successful
  direction. v2.0's single-bit refinement could not reach an adjacent value such
  as 0x3F to 0x40.
- **Noise-aware decisions.** Each measurement is the median of per-sample
  |Γ| ratios with a noise estimate. A move must beat the best by more than the
  noise, and a lucky quick reading is re-measured before it is believed.
- **Adaptive averaging:** `samples` pairs for quick measurements, `psamples`
  near the match.
- **Tunes stop cleanly.** A tune ends if power rises above `pwrmax` (25%
  headroom for SSB peaks) and never moves the relays afterwards until RF is low.
  If something else takes over the sensor, the tuner asks for its measurement
  again rather than hanging.
- **Benchmark.** On a simulated tuner with component tolerances, strays, losses
  and noise, the model search reaches SWR 1.5 on 100% of matchable loads in
  about 24 measurements, and 98% in about 31 with no frequency. v2.0's search
  managed 61% in about 41. Run `pio test -e native -v` to reproduce.

### CAT

- **Frequency polling** every `catpoll` ms once the protocol is known.
  Nothing is sent while the protocol is still unknown.
- **FlexRadio** (`cat flex`): `ZZFA` polling and setting, auto-detection, and CAT
  tune through the radio's TUNE function (`ZZTU`).
- **Icom CI-V address** setting (`civaddr`) for radios that never transmit
  unprompted.
- **USB CAT passthrough** (`catusb`) on the native USB port, so PC software and
  the tuner share one CAT port. The tuner's commands never interleave with the
  PC's.
- **One-button CAT tune** (`tune cat`, the web UI, or TUNE with `cattune on`).
  It saves power and mode, keys a `catpwr` watt carrier, tunes, then unkeys and
  restores. The unkey is verified on the bridge and resent if needed. All
  refusal checks happen before the radio is keyed, and `catmaxms` limits
  key-down time.
- **Fixed:** newer Yaesu ASCII radios were sent 11-digit frequencies. They are
  now sent 9 digits, so sweeps work.

### Memory

- **Interpolation**: blends the nearest stored tunes either side of the
  frequency (same antenna, band and topology, within `meminterp` Hz). Status
  shows `MEM INTERP`.

### Settings

- The settings layout is now version 3. **A v2.0 configuration, including its
  calibration, migrates automatically** on first boot (the log says "migrated
  from v2.0").
- New keys: `protwait`, `psamples`, `tunemodel`, `meminterp`, `catpoll`, `catusb`,
  `civaddr`, `cattune`, `catpwr`, `catmode`, `catmaxms`.
- Removed keys: `relaymode`, `latchms`, `spacing`.
- Changed default: `samples` 4 → 8. Samples are now taken as FWD/REV pairs
  every millisecond.

### Development

- The tuning maths moved to `include/core/`, which has no Arduino dependencies.
- New desktop test harness: `pio test -e native`.
- `hardware.md` documents recommended hardware upgrades: external ADC,
  frequency counter, hardware overload trip, relay coil economiser, and driver
  pull-downs.
- Builds as C++17.

## v2.0

- One firmware image for the LCD and OLED variants, with runtime display
  detection. This replaces the separate `ATU-1000 LCD` and `ATU-1000 - OLED`
  projects.
- Non-blocking tuning with abort and progress display, analytic L-network
  seeding, runtime settings in NVS, multi-protocol CAT, frequency memory with
  CSV import/export, sweep, thermal monitoring, antenna selector, relay cycle
  counters, Wi-Fi web UI and OTA update.
