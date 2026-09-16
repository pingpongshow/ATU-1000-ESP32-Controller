# ATU-1000 ESP32-S3 Controller

ESP32-S3 replacement firmware for the ATU-1000 (and variants) automatic antenna
tuner, replacing the original PIC16F1938 microcontroller.

**One firmware image drives both display variants.** The I2C bus is probed at
boot and the matching driver is instantiated — there is no longer a separate
"OLED build".

## Features

- 7x7 L-network control (7 inductors, 7 capacitors, 1 topology relay)
- **Model-based tuning**: estimates the antenna's complex impedance from the
  first few measurements and jumps to the predicted match, then polishes with a
  noise-aware pattern search. Typically 20–30 measurements.
- Works without a frequency too: the load model also estimates the band
- SWR and forward/reverse power measurement with a guided calibration wizard
- **Dedicated sampling task** on its own CPU core: ~1 kHz FWD/REV sampling,
  per-sample SWR ratios, and an overload trip within a few milliseconds
- **Never hot-switches outside a tune**: relay changes wait until RF is at or
  below `pwrmax`, including protection bypass
- **Automatic display detection**: 20x4 / 16x2 HD44780 LCD (PCF8574 backpack) or
  128x64 SSD1306 / SSD1309 OLED
- **Non-blocking tuning** with a live progress bar and an abort button
- Frequency memory with interpolation, per-band fallback and CSV import/export
- Multi-protocol CAT (Kenwood/Elecraft, FlexRadio SmartSDR, Icom CI-V, Yaesu
  binary and ASCII) with active frequency polling
- **One-button CAT tune**: saves the radio's power and mode, keys a low-power
  carrier, tunes, and restores everything
- **USB CAT passthrough**: share the radio's CAT port with PC logging software
- Auto-tune on an SWR threshold, and a high-SWR alarm
- Power protection with a configurable limit
- Thermal monitoring with foldback (I2C sensor or NTC)
- Antenna selector support, with per-antenna memories
- Relay cycle counters and a relay settle-time test
- Runtime configuration in NVS — no recompile needed to change a calibration;
  v2.0 settings are migrated automatically
- Task watchdog on both the main loop and the sampling task
- Optional Wi-Fi web UI with over-the-air firmware update
- Serial console with echo, backspace, and the same command set as the web UI
- Desktop test harness that benchmarks the tuning search against a simulated
  tuner

## Building

Requires [PlatformIO](https://platformio.org/) and an ESP32-S3 board.

```bash
pio run
```

```bash
pio run -t upload
```

```bash
pio device monitor -b 115200
```

First boot logs a settings-defaults message and NVS "NOT_FOUND" notices. Both
are normal on a fresh chip. A board upgraded from v2.0 logs "migrated from
v2.0" instead and keeps its calibration.

### Tuning test harness

The tuning maths in `include/core/` has no Arduino dependencies and runs on a
desktop machine:

```bash
pio test -e native
```

It drives the search against a simulated tuner that deliberately does not
match the firmware's own model (component tolerances, wiring inductance, stray
capacitance, inductor losses and measurement noise), scores each run against
the best match the hardware could reach, and prints a comparison with the v2.0
search. Run it with `-v` to see the benchmark table.

## Display auto-detection

At boot the firmware scans I2C and picks a driver:

| Address range | Assumed device            | Driver |
|---------------|---------------------------|--------|
| 0x3C, 0x3D    | SSD1306 / SSD1309 OLED    | OLED   |
| 0x20 - 0x27   | PCF8574 LCD backpack      | LCD    |
| 0x38 - 0x3F   | PCF8574A LCD backpack     | LCD    |
| nothing       | headless (console only)   | none   |

0x3C and 0x3D fall inside the PCF8574A range, so an OLED wins the tie. If your
LCD backpack is strapped to one of those two addresses, force it:

```bash
set disptype 2
```

Then reboot. `set disptype 0` restores auto-detection, `1` forces headless, `3`
forces OLED. Run `i2cscan` to see what is actually on the bus — it labels each
address it finds.

If your OLED is a plain SSD1306 rather than an SSD1309, try `set oleddrv 1`
(or `2` for the SSD1309 NONAME0 variant) and reboot.

## How tuning works

1. **Baseline** — measure the current setting.
2. **Seeds** — the memory entry for this frequency, the band's typical setting,
   and ideal L-networks for a spread of resistive loads in both topologies. With
   no frequency, a coarse grid over both topologies instead.
3. **Load model** — a single-directional bridge only measures |Γ|, but a
   dozen |Γ| readings at known network settings are enough to estimate the
   antenna's complex impedance. The fit also decides which way round the K2
   topology relay is wired, and when no frequency is known it estimates the band
   as well. Every relay combination is then evaluated against the model and the
   predicted best few are measured. If they disappoint, the model is refitted
   once with those new points.
4. **Pattern search** — steps through L and C in order of actual value
   (adjacent values can be several relays apart), including diagonal moves that
   follow the L/C valley, halving the step until it converges. A move only
   counts if it beats the current best by more than the measurement noise, and
   a lucky quick reading is re-measured precisely before it is believed.
5. **Confirm** — a precise re-measurement of the winner.

Quick measurements use `samples` FWD/REV pairs; measurements near the match use
`psamples`. The search stops as soon as `swrtarget` is reached.

Power and SWR limits are checked on every measurement. A tune stops if power
rises above `pwrmax` (with 25% headroom for SSB peaks), and the tune never moves
the relays afterwards unless RF is back at or below `pwrmax`.

Run `pio test -e native -v` for current benchmark figures.

## Relay switching safety

Switching the relay bank under high power arcs the contacts. Outside a tune,
which only runs at or below `pwrmax`, the firmware only moves the relays once
**peak** forward power has stayed at or below `pwrmax` for 300 ms. It uses the
peak rather than the average so that a pause between SSB syllables does not
count as RF being off:

- Memory recall on a CAT frequency change, `freq`, `l`, `c`, `topo`, `bypass`,
  the BYPASS button and antenna changes all apply immediately if RF is low, and
  otherwise show `RF HIGH - WAITING` and apply as soon as RF drops.
- Auto-tune does not start (or recall a memory) above `pwrmax`; it shows
  `RETUNE: <50W` so you know to drop the power.
- `tune` refuses above `pwrmax` *before* asserting the TX request line, so a
  refused tune never interrupts your transmission.
- While protection is active, nothing takes the network out of bypass. Changes
  requested in the meantime are applied after `power reset`.

## Serial commands

Type `help` for the full list. Everything below also works from the web UI's
console box.

### General

| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `status` | Full state dump |
| `reboot` | Flush everything to NVS and restart |

### Tuning

| Command | Description |
|---------|-------------|
| `tune` | Start a tune (requires RF) |
| `tune force` | Tune without the minimum-power check |
| `tune cat` | Key the radio over CAT, tune, restore (see CAT tune) |
| `abort` | Stop a tune, CAT tune or sweep in progress |
| `bypass on/off` | Bypass mode |
| `auto on/off` | Auto-tune |
| `freq <hz>` | Set the working frequency, e.g. `freq 14200000` |

### Relay control

| Command | Description |
|---------|-------------|
| `l <hex>` | Set the L relay mask, e.g. `l 01` |
| `c <hex>` | Set the C relay mask, e.g. `c 7f` |
| `topo hi/lo` | Topology relay (Hi-Z / Lo-Z) |
| `ant <1-8>` | Select antenna |
| `cycles` | Per-relay operation counters |
| `cycles reset` | Zero the counters |

**L relays (inductors):** `01`=K1 0.05uH, `02`=K4 0.1uH, `04`=K6 0.22uH,
`08`=K8 0.45uH, `10`=K10 1.0uH, `20`=K12 2.2uH, `40`=K14 4.7uH

**C relays (capacitors):** `01`=K3 10pF, `02`=K5 22pF, `04`=K7 47pF,
`08`=K9 100pF, `10`=K11 220pF, `20`=K13 470pF, `40`=K15 1nF

### Diagnostics

| Command | Description |
|---------|-------------|
| `raw` | Detector millivolts, SWR, \|Γ\| with its noise, and power (not during a tune or sweep) |
| `cal` | Show calibration constants |
| `cal zero` | Zero both detector offsets (transmitter unkeyed) |
| `cal fwd <watts>` | Solve `pwrscale` from a known power into a dummy load |
| `cal rev <swr>` | Solve `revscale` from a known mismatch |
| `cal rev` | Check bridge balance into a 50 ohm load (changes nothing) |
| `settle test` | Measure how long a relay change takes to settle |
| `solve <ohms>` | Ideal L/C for that load at the current frequency |
| `temp` | Temperature sensor reading and source |
| `i2cscan` | Scan and identify I2C devices |

### Memory

| Command | Description |
|---------|-------------|
| `mem size` | Entry count |
| `mem list` | Table of stored tunes |
| `mem clear` | Erase all |
| `mem del <hz>` | Delete one entry |
| `mem export` | Dump as CSV |
| `mem import` | Read CSV rows until a line containing `end` |

CSV columns: `freqHz,lMask,cMask,topology,bypass,antenna,swr,useCount`.
Masks accept `0x..` or decimal. The web UI can download and upload the same file.

Recall order for a frequency: an exact or adjacent memory bin, then a blend of
the nearest stored tunes either side (same antenna, band and topology, no more
than `meminterp` Hz apart), then the nearest entry in the same band. Blending
interpolates L·f and C·f, which is what varies smoothly between two tunes of
the same antenna. The status line shows `MEM HIT`, `MEM INTERP` or `MEM BAND`.

### Protection

| Command | Description |
|---------|-------------|
| `power reset` | Clear the latch and release bypass |
| `power off` | Disable power protection (bench testing only) |
| `power on` | Re-enable |

### Sweep

| Command | Description |
|---------|-------------|
| `sweep <startMHz> <endMHz> [stepkHz] [dwellMs]` | Run an SWR sweep |
| `sweep stop` | Abort |

Requires CAT with a TX line, and a steady carrier for the duration. The radio's
original VFO frequency is restored when the sweep finishes or is aborted.

### CAT

| Command | Description |
|---------|-------------|
| `cat auto` | Auto-detect the protocol |
| `cat kenwood` | Kenwood / Elecraft ASCII |
| `cat flex` | FlexRadio SmartSDR CAT (Kenwood plus ZZ commands) |
| `cat icom` | Icom CI-V |
| `cat yaesu` | Yaesu 5-byte binary (FT-817/857/897) |
| `cat yaesun` | Yaesu newer ASCII |
| `cat off` | Disable CAT |

Auto-detect works, but naming your radio's protocol explicitly is always faster
and more reliable — the Yaesu binary format has no framing at all, so it can
only be identified by repeated self-consistent frames.

### Configuration

| Command | Description |
|---------|-------------|
| `config` | List every setting with its current value |
| `config <text>` | List settings whose key contains `text` |
| `get <key>` | Read one setting |
| `set <key> <value>` | Write one setting (range-checked, saved automatically) |
| `config save` | Force an immediate write |
| `config reset` | Restore factory defaults |

Settings live in NVS. Values are clamped to safe ranges on both write and load,
so a typo cannot disable protection or corrupt the config.

### Wi-Fi and the web UI

| Command | Description |
|---------|-------------|
| `wifi status` | Show state, address, hostname |
| `wifi <ssid> <passphrase>` | Store credentials |
| `wifi on` / `wifi off` | Bring the interface up or down |

Wi-Fi is **off by default** — a Wi-Fi radio next to an SWR bridge is not free,
and the tuner must work with no network at all. `wifi on` connects in the
background (the tuner keeps running meanwhile) and prints the address on the
serial console once connected; then browse to it or to
`http://atu1000.local/`. If the configured network cannot be joined within
15 seconds the firmware falls back to an access point named `atu1000-setup`.

The web UI provides live SWR/power, tune, CAT tune and bypass controls, the full
command console, memory download/upload, and **over-the-air firmware update** —
upload `.pio/build/esp32s3/firmware.bin` on the "Firmware update" card. Do not
update while transmitting: the relays drop out while the board reboots.

The passphrase is never echoed back by `get`, `config`, or the web API.

## CAT

### Frequency polling

Many radios only report their frequency when asked (Kenwood/Yaesu with
auto-information off, Icom with CI-V transceive off). Once the protocol is
selected or detected, the tuner asks every `catpoll` milliseconds (default
1000; `set catpoll 0` to only listen).

Nothing is sent while the protocol is still unknown in auto mode: probing a
Yaesu 5-byte radio with Kenwood text could land on its transmit command. Select
the protocol explicitly if the radio stays silent until polled.

Icom radios need a CI-V address to be polled. The tuner learns it from the
radio's own traffic; if the radio never sends anything unprompted, set it:
`set civaddr 148` (0x94, for example, is the IC-7300).

### FlexRadio

`cat flex` uses SmartSDR CAT's Kenwood-compatible command set with the ZZ
extensions: the frequency is polled with `ZZFA;`, sweeps set it with `ZZFA`,
and CAT tune uses the radio's own TUNE function (`ZZTU1;` / `ZZTU0;`) at the
radio's TUNE power setting. Auto-detect also recognises `ZZFA` replies.

Connect the tuner to a serial port that SmartSDR CAT is configured to serve (a
physical COM port on the SmartSDR PC through an RS-232 level shifter, or a
USB-serial adapter mapped as a SmartSDR CAT port).

### CAT tune

`tune cat`, the web UI's **CAT Tune** button, or — with `set cattune on` — a
short press of TUNE:

1. reads the radio's power and mode,
2. sets `catpwr` watts (default 10) and a `catmode` carrier (0=FM default,
   1=AM, 2=CW),
3. keys the radio and waits for RF,
4. tunes,
5. unkeys and restores the original power and mode.

Any failure, an abort, protection, or the `catmaxms` key-down limit (default
15 s) unkeys the radio and restores its settings. The tuner then checks the
bridge to confirm the carrier really stopped, resends the unkey up to three
times if it did not, and shows `CAT UNKEY FAILED` if the radio still will not
stop. Everything that would refuse a tune (holdoff, protection, a tune already
running) is checked before the radio is keyed. If the radio is already
transmitting when you press TUNE, it just tunes.

| Radio family | Power | Mode | Key |
|--------------|-------|------|-----|
| Kenwood / Elecraft | `PCnnn;` | `MDn;` | `TX;` / `RX;` |
| Yaesu new ASCII | `PCnnn;` | `MD0n;` | `TX1;` / `TX0;` |
| FlexRadio | radio's TUNE power | — | `ZZTU1;` / `ZZTU0;` |
| Icom CI-V | 0x14 0x0A (% of a 100 W radio) | 0x06 | 0x1C 0x00 |
| Yaesu 5-byte | not settable — set it on the radio | 0x07 | 0x08 / 0x88 |

Notes:

- `catpwr` is sent as watts, which on 100 W radios is also percent. For Icom
  it is scaled as a percentage of 100 W.
- Yaesu FT-817/857/897 cannot have their power set over CAT. Turn the power
  down on the radio first; a tune above `pwrmax` is refused.
- If the tuner's TX request output is wired to the radio's **TX inhibit**
  input, the radio will refuse to transmit the CAT tune carrier while the tune
  runs. Leave that line unconnected if you use CAT tune.

### USB passthrough

`set catusb on`, then reboot. The ESP32-S3's native USB port (the connector
marked **USB** on a DevKitC-1, not the one marked UART that carries the
console) then appears as a serial port on the PC. Point logging or contest
software at it, at any baud rate:

- everything the PC sends goes to the radio, and every reply goes back to the
  PC, while the tuner reads the same replies for its frequency;
- while the PC is polling, the tuner stops polling;
- the tuner's own commands (sweeps, CAT tune) are held until the PC finishes
  its current command, so they never interleave with it.

The PC also sees the replies to the tuner's own commands. Most software ignores
unsolicited frequency replies; if yours does not, avoid sweeps and CAT tune
while it is connected.

## SWR bridge calibration

Power formula:

```
Vf = max(0, (fwdMv - fwdOffsetMv)) * fwdScale
P  = (Vf / 1000)^2 / powerScale
```

Use the wizard rather than doing this by hand:

1. `power off` (so a mis-scaled reading cannot trip protection mid-calibration)
2. With the transmitter **unkeyed**, `cal zero` — measures both detectors'
   no-signal offsets
3. Connect a 50 ohm dummy load, key a steady carrier at a known power, e.g. 25 W
4. `cal fwd 25` — solves and saves `pwrscale`, then reports what it now reads
5. Still into the dummy load, `cal rev` — reports the bridge's residual SWR.
   It changes nothing; above about 1.1, adjust the bridge balance trimmer.
6. Connect a known mismatch — a 100 ohm non-inductive load is SWR 2.0 — key the
   same carrier and `cal rev 2.0`. This solves and saves `revscale`.
7. `power on`

Verify with `raw` at a few power levels. Diode detectors are non-linear, so
calibrate near your normal operating power.

v2.0's `cal rev` wrote the reverse reading into a matched load as `revoffset`.
That residual is bridge imbalance and diode leakage, which change with power, so
it made SWR wrong at every other power level. If you calibrated with it, run
`cal zero` again.

To adjust by hand instead:

```bash
set pwrscale 0.0294
```

### Relay settle time

`settle test` needs a steady carrier between `pwrmin` and `pwrmax`. It steps the
capacitor bank away and back, records the detectors at the ADC's full rate, and
reports how long each change took to settle (relay operate time plus contact
bounce plus the detector's filter), with a recommended `settle` value. Too short
a settle time makes the tuner judge each setting by the previous one's reading.

## Protection

### Overload

The sampling task checks every FWD sample. Two consecutive samples at or above
`pwrlimit` (about 2 ms) and it:

1. asserts the TX request/inhibit line immediately, from the sampling task,
   independently of the main loop;
2. unkeys a CAT tune, and stops any tune or sweep **without moving the relays**;
3. waits for peak forward power to stay at or below `pwrmax` for 300 ms, then
   engages bypass;
4. if RF is still present after `protwait` ms (default 300), shows
   `PROTECT - RF ON` and keeps holding the current match rather than
   hot-switching the relay bank. Bypass engages as soon as RF drops.

The TX inhibit line stays asserted until `power reset` (or a long press of
AUTO). A sustained overload re-trips immediately after a reset.

Wire the TX request output to your radio's TX inhibit / amplifier interlock if
it has one — with a kilowatt present, not switching is the only safe thing the
relays can do.

### High SWR alarm

SWR at or above `swrmax` (default 10) for half a second while transmitting
above `pwrmin` shows `! HIGH SWR` on the display and web UI and blinks the
status LED. It clears after a second below the threshold.

### Thermal

Temperature is read from, in order of preference:

1. An LM75 / TMP102 / TMP75 compatible sensor on the display's I2C bus,
   auto-probed at 0x48–0x4F. This is the recommended option: it costs no GPIO,
   and every ADC1 pin on this board is already used.
2. An NTC thermistor on `kPins.ntcAdc`, if you free up an ADC1 pin.
3. The ESP32-S3 internal die sensor. This is **displayed only** and never
   triggers foldback or bypass: it measures the chip, which runs warm on its own
   with Wi-Fi up, not the relays or inductors.

Escalation: `tempwarn` shows a warning, `tempfold` inhibits TX and refuses to
tune, `templimit` runs the protection sequence above. There is 3 °C of
hysteresis on the way back down.

### Watchdog

Both the main loop and the sampling task are registered with the ESP32 task
watchdog (5 s). A hang resets the board rather than leaving it unprotected.

## Display layouts

### OLED (128x64)

```
14.250  MHz          25C
        20m
[####################]  1.2     <- SWR bar and value
[##########          ]  250W    <- power bar and value
2.20uH 470pF LoZ
--------------------------------
TUNE OK            CAT AUTO A1
```

An alarm (protection, over-temperature, high SWR, power warning) replaces the
bottom row with an inverted banner. During a tune the whole screen becomes a
progress bar. The frame is shifted a few pixels periodically to avoid burn-in,
dims after `dimsec` seconds idle, and can blank after `blanksec`.

### LCD (20x4)

```
Line 0: CAT 14.250 20m   A
Line 1: S1.2 [######] 250W
Line 2: 2.20uH  470pF L
Line 3: TUNE OK        A1 *
```

On 16x2 panels rows 2 and 3 are omitted automatically. Status messages:
`READY`, `TUNING`, `TUNE OK`, `TUNE GOOD`, `TUNE FAIL`, `NO RF`, `PWR HIGH`,
`ABORTED`, `MEM HIT`, `MEM INTERP`, `MEM BAND`, `MEM MISS`, `BYPASS`,
`SWEEP n%`, `CAT TUNE`, `CAT NO REPLY`, `CAT NO RF`, `CAT TX LIMIT`,
`CAT UNKEY FAILED`, `BUSY`,
`RF HIGH - WAITING`, `RETUNE: <nW`, `WAIT - HOLDOFF`.

## Button functions

| Button | Short press | Long press |
|--------|-------------|------------|
| Tune (GPIO 0) | Tune (CAT tune if `cattune` is on), or **abort** | Force tune, or abort |
| Bypass (GPIO 3) | Toggle bypass | Next antenna (if more than one) |
| Auto (GPIO 4) | Toggle auto-tune | Reset protection |

## Configuration reference

Run `config` for the authoritative list with current values. Frequently used
keys:

| Key | Meaning |
|-----|---------|
| `swrtarget` / `swrgood` / `swrretune` | Tune success, acceptable, and auto-retune thresholds |
| `swrmax` | High-SWR alarm threshold |
| `pwrmin` / `pwrmax` / `pwrautomin` | Tuning power window (`pwrmax` is also the relay-switching ceiling) and auto-retune floor |
| `pwrlimit` / `pwrwarn` / `pwrprot` | Overload limit, warning level, master enable |
| `protwait` | How long protection waits for RF to drop before holding |
| `fwdoffset` / `revoffset` / `fwdscale` / `revscale` / `pwrscale` | Bridge calibration |
| `samples` / `psamples` | FWD/REV pairs per quick / precise measurement |
| `maxsteps` / `refine` / `tunemodel` | Search effort limit, re-expansions after converging, load model on/off |
| `settle` | Relay settle time (use `settle test`) |
| `membin` / `memmax` / `memband` / `meminterp` | Memory bin width, capacity, band fallback, interpolation span |
| `catproto` / `catbaud` / `catpoll` / `civaddr` | CAT protocol, speed, polling period, Icom address |
| `catusb` | USB CAT passthrough |
| `cattune` / `catpwr` / `catmode` / `catmaxms` | CAT tune from the button, carrier power, mode, key-down limit |
| `disptype` / `oleddrv` / `lcdcols` / `lcdrows` | Display selection |
| `dimsec` / `blanksec` / `burnsec` | Screen dimming, blanking, burn-in shift |
| `tempwarn` / `tempfold` / `templimit` | Thermal thresholds |
| `antcount` / `ant` | Antenna selector |

v2.1 removed `relaymode`, `latchms` and `spacing`. The pulsed "latching relay"
mode could never have worked on this board: each relay has a single low-side
driver, which cannot deliver the reset pulse a latching relay needs to turn off.
Sample pacing is now fixed at one FWD/REV pair per millisecond.

## Troubleshooting

**Display not working** — run `i2cscan`; it names what it finds. Check SDA/SCL
wiring and that 4.7k pull-ups are present. If a device is detected but the wrong
driver was picked, force it with `set disptype` and reboot.

**False power overload** — the ADC inputs read noise when the SWR bridge is not
connected. Use `power off` while testing on the bench, and `power reset` to
clear a latched state.

**SWR / power readings wrong** — confirm FWD and REV are not swapped with `raw`,
then run the calibration wizard from `cal zero`.

**CAT not decoding** — set the protocol explicitly (`cat icom` etc.) rather than
relying on auto-detect, and check `catbaud` matches the radio. `status` shows
whether polling is active.

**Relays not switching** — test individually with `l XX` / `c XX`, verify 3.3 V
is enough to drive the MJD122 bases, and check the wiring to the PIC socket. If
the console says the change is held, RF is above `pwrmax`.

**Tune finds a poor match or wanders** — run `settle test` first; a settle time
that is too short is the most common cause. Then check calibration with
`cal rev`: a bridge that reads SWR 1.3 into a dummy load limits every tune.

**Tune takes too long or gives up early** — `set maxsteps 300` allows a longer
search; `set psamples 32` averages more near the match on a noisy bridge.

**Auto-tune says `RETUNE: <50W`** — the SWR is above `swrretune` but you are
transmitting above `pwrmax`. Drop the power for a moment and it will retune.

## Files

```
ATU-1000/
├── platformio.ini
├── README.md
├── hardware.md              ESP32 to PIC socket pin mapping, hardware upgrades
├── license.md
├── include/
│   ├── core/                Hardware-independent tuning maths
│   │   ├── Components.h     Relay component values, band table
│   │   ├── LoadFit.h        Complex load estimation and match planning
│   │   ├── Network.h        L-network circuit model, value ladders
│   │   ├── RelayState.h     Relay bank state
│   │   ├── Solver.h         Analytic L-network seeds
│   │   └── TuneSearch.h     Tuning search strategy
│   ├── App.h                Global application state
│   ├── Cat.h                Multi-protocol CAT, polling, USB passthrough
│   ├── CatTune.h            One-button CAT tune sequence
│   ├── Commands.h           Console command parser
│   ├── Config.h             Pin map
│   ├── Controls.h           Buttons, LED, TX line, power warning
│   ├── Display.h            Display interface, LCD and OLED drivers, detection
│   ├── MemStore.h           Frequency memory, interpolation, CSV
│   ├── Relays.h             Relay bank, cycle counters, antenna select
│   ├── Sensor.h             SWR bridge sampling task and fast overload trip
│   ├── Settings.h           NVS-backed runtime settings and v2.0 migration
│   ├── Sweep.h              Non-blocking SWR sweep
│   ├── Thermal.h            Temperature monitoring and foldback
│   ├── Tuner.h              Tuning engine (hardware glue for TuneSearch)
│   ├── Types.h              Shared value types
│   └── WebUi.h              Wi-Fi, web UI, OTA
├── src/
│   └── main.cpp             Wiring, protection, setup and loop
├── test/
│   └── test_tuning/         Desktop benchmark against a simulated tuner
└── Archive/                 Schematics and the original PIC firmware
```

## See also

- [hardware.md](hardware.md) — detailed pin mapping, hardware notes and
  recommended hardware upgrades
- [license.md](license.md)
