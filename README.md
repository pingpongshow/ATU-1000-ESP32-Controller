# ATU-1000 ESP32-S3 Controller

ESP32-S3 replacement firmware for the ATU-1000 (and variants) automatic antenna
tuner, replacing the original PIC16F1938 microcontroller.

**One firmware image drives both display variants.** The I2C bus is probed at
boot and the matching driver is instantiated — there is no longer a separate
"OLED build".

## Features

- 7x7 L-network control (7 inductors, 7 capacitors, 1 topology relay)
- SWR and forward/reverse power measurement with a guided calibration wizard
- **Automatic display detection**: 20x4 / 16x2 HD44780 LCD (PCF8574 backpack) or
  128x64 SSD1306 / SSD1309 OLED
- **Non-blocking tuning** with a live progress bar and an abort button
- Analytic L-network seeding plus successive-approximation search
- Frequency memory with per-band fallback and CSV import/export
- Multi-protocol CAT (Kenwood/Elecraft, Icom CI-V, Yaesu binary and ASCII)
- Auto-tune on an SWR threshold
- Power protection with a configurable limit
- Thermal monitoring with foldback (I2C sensor, NTC, or the internal die sensor)
- Antenna selector support, with per-antenna memories
- Relay cycle counters
- Runtime configuration in NVS — no recompile needed to change a calibration
- Optional Wi-Fi web UI with over-the-air firmware update
- Serial console with echo, backspace, and the same command set as the web UI

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
are normal on a fresh chip.

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
| `tune force` | Tune without the RF power check |
| `abort` | Stop a tune or sweep in progress |
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
| `raw` | Raw ADC counts, millivolts, SWR and power |
| `cal` | Show calibration constants |
| `cal fwd <watts>` | Solve `pwrscale` from a known power |
| `cal rev` | Zero the reverse channel into a matched load |
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
| `cat kenwood` | Kenwood / Elecraft / FlexRadio ASCII |
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
and the tuner must work with no network at all. Once up, browse to the printed
IP or `http://atu1000.local/`. If the configured network cannot be joined the
firmware falls back to an access point named `atu1000-setup`.

The web UI provides live SWR/power, tune and bypass controls, the full command
console, memory download/upload, and **over-the-air firmware update** — upload
`.pio/build/esp32s3/firmware.bin` on the "Firmware update" card.

The passphrase is never echoed back by `get`, `config`, or the web API.

## SWR bridge calibration

Power formula:

```
Vf = max(0, (fwdMv - fwdOffsetMv)) * fwdScale
P  = (Vf / 1000)^2 / powerScale
```

Use the wizard rather than doing this by hand:

1. `power off` (so a mis-scaled reading cannot trip protection mid-calibration)
2. Connect a 50 ohm dummy load
3. Key a steady carrier at a known power, e.g. 25 W
4. `cal fwd 25` — solves and saves `pwrscale`, then reports what it now reads
5. Still into the matched load, `cal rev` — zeroes the reverse detector offset
6. `power on`

Verify with `raw` at a few power levels. Diode detectors are non-linear, so
calibrate near your normal operating power.

To adjust by hand instead:

```bash
set pwrscale 0.0294
```

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

An alarm (overload, over-temperature, power warning) replaces the bottom row
with an inverted banner. During a tune the whole screen becomes a progress bar.
The frame is shifted a few pixels periodically to avoid burn-in, dims after
`dimsec` seconds idle, and can blank after `blanksec`.

### LCD (20x4)

```
Line 0: CAT 14.250 20m   A
Line 1: S1.2 [######] 250W
Line 2: 2.20uH  470pF L
Line 3: TUNE OK        A1 *
```

On 16x2 panels rows 2 and 3 are omitted automatically. Status messages:
`READY`, `TUNING`, `TUNE OK`, `TUNE GOOD`, `TUNE FAIL`, `NO RF`, `PWR HIGH`,
`ABORTED`, `MEM HIT`, `MEM BAND`, `MEM MISS`, `BYPASS`, `SWEEP n%`.

## Button functions

| Button | Short press | Long press |
|--------|-------------|------------|
| Tune (GPIO 0) | Tune, or **abort** if one is running | Force tune, or abort |
| Bypass (GPIO 3) | Toggle bypass | Next antenna (if more than one) |
| Auto (GPIO 4) | Toggle auto-tune | Reset protection |

## Thermal protection

Temperature is read from, in order of preference:

1. An LM75 / TMP102 / TMP75 compatible sensor on the display's I2C bus,
   auto-probed at 0x48–0x4F. This is the recommended option: it costs no GPIO,
   and every ADC1 pin on this board is already used.
2. An NTC thermistor on `kPins.ntcAdc`, if you free up an ADC1 pin.
3. The ESP32-S3 internal die sensor, as a coarse fallback.

Escalation: `tempwarn` shows a warning, `tempfold` inhibits TX and refuses to
tune, `templimit` engages bypass. There is 3 °C of hysteresis on the way back
down.

## Configuration reference

Run `config` for the authoritative list with current values. Frequently used
keys:

| Key | Meaning |
|-----|---------|
| `swrtarget` / `swrgood` / `swrretune` | Tune success, acceptable, and auto-retune thresholds |
| `pwrmin` / `pwrmax` / `pwrautomin` | Tuning power window and auto-retune floor |
| `pwrlimit` / `pwrwarn` / `pwrprot` | Overload limit, warning level, master enable |
| `fwdoffset` / `revoffset` / `pwrscale` | Bridge calibration |
| `disptype` / `oleddrv` / `lcdcols` / `lcdrows` | Display selection |
| `dimsec` / `blanksec` / `burnsec` | Screen dimming, blanking, burn-in shift |
| `maxsteps` / `refine` | Tuning search effort |
| `membin` / `memmax` / `memband` | Memory bin width, capacity, band fallback |
| `relaymode` / `latchms` | Continuous or pulsed (impulse relay) drive |
| `tempwarn` / `tempfold` / `templimit` | Thermal thresholds |
| `antcount` / `ant` | Antenna selector |

## Troubleshooting

**Display not working** — run `i2cscan`; it names what it finds. Check SDA/SCL
wiring and that 4.7k pull-ups are present. If a device is detected but the wrong
driver was picked, force it with `set disptype` and reboot.

**False power overload** — the ADC inputs read noise when the SWR bridge is not
connected. Use `power off` while testing on the bench, and `power reset` to
clear a latched state.

**SWR / power readings wrong** — confirm FWD and REV are not swapped with `raw`,
then run the `cal fwd` wizard.

**CAT not decoding** — set the protocol explicitly (`cat icom` etc.) rather than
relying on auto-detect, and check `catbaud` matches the radio.

**Relays not switching** — test individually with `l XX` / `c XX`, verify 3.3 V
is enough to drive the MJD122 bases, and check the wiring to the PIC socket.

**Tune takes too long or gives up early** — `set maxsteps 300` allows a longer
search; `set refine 5` spends more effort on the final polish.

## Files

```
ATU-1000/
├── platformio.ini
├── README.md
├── hardware.md              ESP32 to PIC socket pin mapping
├── license.md
├── include/
│   ├── App.h                Global application state
│   ├── Cat.h                Multi-protocol CAT reader
│   ├── Commands.h           Console command parser
│   ├── Config.h             Pin map, component values, band table
│   ├── Controls.h           Buttons, LED, power protection
│   ├── Display.h            Display interface, LCD and OLED drivers, detection
│   ├── MemStore.h           Frequency memory, CSV
│   ├── Relays.h             Relay bank, cycle counters, antenna select
│   ├── Sensor.h             SWR bridge front end
│   ├── Settings.h           NVS-backed runtime settings
│   ├── Solver.h             L-network seeding
│   ├── Sweep.h              Non-blocking SWR sweep
│   ├── Thermal.h            Temperature monitoring and foldback
│   ├── Tuner.h              Non-blocking tuning state machine
│   ├── Types.h              Shared value types
│   └── WebUi.h              Wi-Fi, web UI, OTA
├── src/
│   └── main.cpp             Wiring, setup and loop
└── Archive/                 Schematics and the original PIC firmware
```

## See also

- [hardware.md](hardware.md) — detailed pin mapping and hardware notes
- [license.md](license.md)
