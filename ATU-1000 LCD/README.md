# ATU-1000 ESP32-S3 Controller

ESP32-S3 replacement controller firmware for the ATU-1000 (and variants) automatic antenna tuner, replacing the original PIC16F1938 microcontroller.

## Features

- 7x7 L-network tuner control (7 inductors, 7 capacitors, 1 topology relay)
- SWR and power measurement with calibration
- 20x4 I2C LCD display with power bargraph
- Frequency memory - stores tuning settings per frequency
- Multi-protocol CAT support (Kenwood, Icom, Yaesu)
- Auto-tune on SWR threshold
- Power protection (configurable limit, default 1000W)
- Serial command interface for control and diagnostics

## Building

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or IDE)
- ESP32-S3 development board

### Build Commands

```bash
# Build
pio run

# Build and upload
pio run -t upload

# Monitor serial output
pio device monitor -b 115200
```

## Flashing

1. Connect ESP32-S3 via USB
2. Run `pio run -t upload`
3. Open serial monitor at 115200 baud

First boot will show NVS "NOT_FOUND" errors - this is normal (no saved settings yet).

## Serial Commands

### General

| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `status` | Show current state (SWR, power, frequency, relay masks) |

### Tuning

| Command | Description |
|---------|-------------|
| `tune` | Start tuning (requires RF power) |
| `tune force` | Force tune without RF power check |
| `bypass on/off` | Enable/disable bypass mode |
| `auto on/off` | Enable/disable auto-tune |
| `freq <hz>` | Set frequency manually (e.g., `freq 14200000`) |

### Relay Control

| Command | Description |
|---------|-------------|
| `l <hex>` | Set L relay mask (e.g., `l 01` for first inductor only) |
| `c <hex>` | Set C relay mask (e.g., `c 7f` for all capacitors) |
| `topo hi/lo` | Set topology relay (Hi-Z or Lo-Z) |

#### Individual Relay Commands

**L Relays (Inductors):**
```
l 01    # K1 ON  (0.05uH)
l 02    # K4 ON  (0.1uH)
l 04    # K6 ON  (0.22uH)
l 08    # K8 ON  (0.45uH)
l 10    # K10 ON (1.0uH)
l 20    # K12 ON (2.2uH)
l 40    # K14 ON (4.7uH)
l 00    # All OFF
```

**C Relays (Capacitors):**
```
c 01    # K3 ON  (10pF)
c 02    # K5 ON  (22pF)
c 04    # K7 ON  (47pF)
c 08    # K9 ON  (100pF)
c 10    # K11 ON (220pF)
c 20    # K13 ON (470pF)
c 40    # K15 ON (1nF)
c 00    # All OFF
```

### Diagnostics

| Command | Description |
|---------|-------------|
| `raw` | Show raw ADC values and calculated SWR/power |
| `cal` | Show calibration constants |
| `i2cscan` | Scan I2C bus for devices |

### Memory

| Command | Description |
|---------|-------------|
| `mem size` | Show number of stored frequency/settings pairs |
| `mem clear` | Clear all stored tuning memory |

### Power Protection

| Command | Description |
|---------|-------------|
| `power reset` | Reset overload protection and clear bypass |
| `power off` | Disable power protection (for testing) |
| `power on` | Enable power protection |

### CAT Control

| Command | Description |
|---------|-------------|
| `cat kenwood` | Set CAT protocol to Kenwood/Elecraft |
| `cat icom` | Set CAT protocol to Icom CI-V |
| `cat yaesu` | Set CAT protocol to Yaesu |
| `cat auto` | Auto-detect CAT protocol |

### SWR Sweep

| Command | Description |
|---------|-------------|
| `sweep <startMHz> <endMHz> [stepkHz] [dwellMs]` | Run SWR sweep (requires CAT TX control) |

## SWR Bridge Calibration

### Initial Setup

1. Disable power protection during calibration: `power off`
2. Connect a 50-ohm dummy load
3. Transmit at a known power level (e.g., 25W)
4. Run `raw` to see ADC readings

### Calibration Values

Edit `include/Config.h` to adjust these values:

```cpp
.fwdOffsetMv = 15.0f,       // Diode forward voltage drop (mV)
.revOffsetMv = 15.0f,
.fwdScale = 1.0f,           // Voltage scaling factor
.revScale = 1.0f,
.powerScale = 0.03f,        // V^2 to Watts conversion
.swrMinForward = 30.0f,     // Minimum mV for valid SWR reading
```

### Power Scale Calculation

The power formula is: `Power = (Vfwd - offset)^2 / powerScale`

To calculate powerScale:
1. Transmit known power (e.g., 25W) into dummy load
2. Run `raw` to get FWD mV reading
3. Calculate: `powerScale = (Vfwd_mV/1000 - offset/1000)^2 / actual_power`

Example:
- Transmitting 25W
- FWD reads 873mV, offset is 15mV
- `powerScale = (0.873 - 0.015)^2 / 25 = 0.736 / 25 = 0.0294`

### Verifying Calibration

1. Transmit into dummy load at various power levels
2. Run `raw` and verify power reading matches actual power
3. SWR should read ~1.0 into 50-ohm load

Note: Diode detectors have some non-linearity. Calibrate at your typical operating power for best accuracy.

## Display Layout

20x4 LCD display showing:

```
Line 0: [CAT] [FREQ  ] [M]
Line 1: [SWR] [BARGRAPH] [PWR]
Line 2: [L value] [C value] [T][B]
Line 3: [STATUS MESSAGE     ]
```

### Line 0 - CAT/Frequency
- `CAT` or `---` - CAT connection status
- `14.200` or `--.---` - Current frequency in MHz
- `A` or `M` - Auto-tune or Manual mode

### Line 1 - SWR/Power
- `S1.5` - SWR value (or `S>10` if very high)
- Bargraph - Visual power indicator (6 characters)
- `  25W` - Power in watts (or `!OVL!` / ` WARN`)

### Line 2 - Tuner State
- `2.20uH` - Total inductance switched in
- `470pF` - Total capacitance switched in
- `H` or `L` - Hi-Z or Lo-Z topology
- `B` or space - Bypass mode indicator

### Line 3 - Status
Shows current status messages:
- `READY` - Idle, ready to tune
- `TUNING` - Tune in progress
- `TUNE OK` - Successful tune
- `TUNE FAIL` - Could not achieve target SWR
- `NO RF` - Insufficient power to tune
- `PWR HIGH` - Power too high for tuning
- `MEM HIT` - Loaded settings from memory
- `MEM MISS` - No stored settings for frequency
- `BYPASS` - Bypass mode active
- `!OVERLOAD!` - Power protection triggered

## Configuration

Key settings in `include/Config.h`:

### Power Thresholds
```cpp
.minTunePowerW = 1.0f,      // Minimum power to tune
.maxTunePowerW = 50.0f,     // Maximum power for tuning
.minAutoRetunePowerW = 3.0f, // Minimum for auto-retune
```

### SWR Thresholds
```cpp
.targetSWR = 1.2f,          // Target SWR for "success"
.goodSWR = 1.5f,            // Acceptable SWR for "good enough"
.autoRetuneSWR = 2.5f,      // SWR threshold for auto-retune
```

### LCD Settings
```cpp
.lcdAddress = 0x27,         // I2C address (try 0x3F if not found)
.lcdCols = 20,
.lcdRows = 4,
```

## Button Functions

| Button | Short Press | Long Press |
|--------|-------------|------------|
| Tune (GPIO 0) | Normal tune (requires RF) | Force tune |
| Bypass (GPIO 3) | Toggle bypass mode | - |
| Auto (GPIO 4) | Toggle auto-tune | - |

## Troubleshooting

### Display Not Working
1. Run `i2cscan` to verify I2C device detected
2. Check I2C address (0x27 or 0x3F common)
3. Verify SDA/SCL wiring and pull-up resistors

### False Power Overload
- ADC pins reading noise when SWR bridge not connected
- Use `power off` during testing without bridge
- Use `power reset` to clear overload state

### SWR/Power Readings Wrong
- Check FWD/REV are not swapped
- Verify with `raw` command and compare to DVM measurements
- Adjust calibration values in Config.h

### Relays Not Switching
- Test individual relays with `l XX` and `c XX` commands
- Verify 3.3V logic is sufficient for MJD122 transistor drive
- Check wiring from ESP32 GPIO to PIC socket pins

## Files

```
ATU-1000/
├── platformio.ini          # PlatformIO project config
├── README.md               # This file
├── hardware.md             # Hardware pin mapping details
├── license.md              # License information
├── include/
│   └── Config.h            # Pin mappings, tuning constants, calibration
├── src/
│   └── main.cpp            # Complete firmware implementation
└── Archive/
    ├── schematic/          # Schematic images and PIC disassembly
    └── Firmware/           # Original PIC hex files
```

## See Also

- [hardware.md](hardware.md) - Detailed ESP32 to PIC pin mapping
- [license.md](license.md) - License information
