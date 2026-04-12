# ATU-1000 Hardware Pin Mapping

This document details the ESP32-S3 GPIO to PIC16F1938 socket pin mapping for the ATU-1000 controller replacement.

## Overview

The ESP32-S3 connects directly to the PIC16F1938 socket (PIC removed). The MJD122 Darlington transistors drive the relays, with RF chokes (inductors) between the microcontroller pins and transistor bases.

## Relay Output Mapping

### Inductor Relays (L)

| ESP32 GPIO | PIC Socket Pin | Transistor | Relay | Inductance | Mask Bit |
|:----------:|:--------------:|:----------:|:-----:|:----------:|:--------:|
| 5          | 24             | Q13        | K1    | 0.05 uH    | 0x01 |
| 6          | 4              | Q15        | K4    | 0.1 uH     | 0x02 |
| 7          | 25             | Q14        | K6    | 0.22 uH    | 0x04 |
| 8          | 5              | Q12        | K8    | 0.45 uH    | 0x08 |
| 9 | 26 | Q10 | K10 | 1.0 uH | 0x10 |
| 10 | 7 | Q11 | K12 | 2.2 uH | 0x20 |
| 11 | 6 | Q9 | K14 | 4.7 uH | 0x40 |

Total L range: 0.05 uH to 8.72 uH (all relays on)

### Capacitor Relays (C)

| ESP32 GPIO | PIC Socket Pin | Transistor | Relay | Capacitance | Mask Bit |
|:----------:|:--------------:|:----------:|:-----:|:-----------:|:--------:|
| 12         | 18             | Q6         | K3    | 10 pF       | 0x01 |
| 13         | 14             | Q8         | K5    | 22 pF       | 0x02 |
| 14 | 17 | Q7 | K7 | 47 pF | 0x04 |
| 15 | 13 | Q5 | K9 | 100 pF | 0x08 |
| 16 | 16 | Q3 | K11 | 220 pF | 0x10 |
| 17 | 12 | Q4 | K13 | 470 pF | 0x20 |
| 18 | 15 | Q2 | K15 | 1000 pF | 0x40 |

Total C range: 10 pF to 1869 pF (all relays on)

### Topology Relay (I/O)

| ESP32 GPIO | PIC Socket Pin | Transistor | Relay | Function |
|:----------:|:--------------:|:----------:|:-----:|:---------|
| 48 | 11 | Q1 | K2 | I/O (Topology) |

**K2 Function:** This is the L-network **topology relay**, NOT a bypass relay.
- **OFF (Lo-Z mode):** Capacitors on OUTPUT side - for low impedance loads
- **ON (Hi-Z mode):** Capacitors on INPUT side - for high impedance loads

Bypass mode is implemented in software by turning all L and C relays OFF.

## ADC Inputs

| ESP32 GPIO | PIC Socket Pin | PIC Function | Signal |
|:----------:|:--------------:|:------------:|:------:|
| 2          | 2              | RA0/AN0      | Forward Power |
| 1 | 3 | RA1/AN1 | Reverse Power |

**Note:** FWD/REV are swapped from the original PIC pinout to match the actual bridge wiring on this board.

### ADC Specifications
- ESP32-S3 ADC with 11dB attenuation
- Safe input range: 0 - 3.1V
- 12-bit resolution (0-4095)

## Additional ESP32 Pins (No PIC Equivalent)

These pins are directly connected to the ESP32 for new functionality:

### User Buttons

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 0 | Tune Button | Strapping pin (safe for input) |
| 3 | Bypass Button | Strapping pin (safe for input) |
| 4 | Auto Button | Weak pullup on reset |

Buttons connect to GND when pressed (active low, internal pull-up enabled).

### I2C Display (20x4 LCD)

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 47 | I2C SDA | USB OTG pin (safe after boot) |
| 21 | I2C SCL | USB OTG pin (safe after boot) |

Default I2C address: 0x27 (some modules use 0x3F)

### CAT Interface (UART)

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 39 | CAT RX | USB OTG pin (safe after boot) |
| 40 | CAT TX | USB OTG pin (safe after boot) |

### TX Request Outputs

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 41 | TX Request | USB OTG pin (safe after boot) |
| 42 | TX Request (inverted) | USB OTG pin (safe after boot) |

### Status LED

| ESP32 GPIO | Function | Notes |
|:----------:|:---------|:------|
| -1 | Disabled | RGB LED on GPIO 38 available separately |

## GPIO Classification

### Clean Pins (No Boot Conflicts)
GPIO 1, 2, 5-18, 48 - Used for relays and ADC

### Strapping Pins (Safe for Input After Boot)
GPIO 0, 3, 4 - Used for buttons

### USB OTG Pins (Safe After Boot)
GPIO 21, 39, 40, 41, 42, 47 - Used for I2C, CAT, TX Request

### Reserved/Avoided
- GPIO 19, 20 - Native USB D-/D+
- GPIO 35, 36, 37 - PSRAM
- GPIO 38 - RGB LED
- GPIO 43, 44 - USB Serial UART
- GPIO 45, 46 - Strapping pins (avoided)

## Logic Levels

| Device | Logic Level | Notes |
|--------|:-----------:|-------|
| ESP32-S3 | 3.3V | NOT 5V tolerant |
| PIC16F1938 | 5V | Original design |
| MJD122 | ~1.4V Vbe | Darlington, works with 3.3V drive |

The MJD122 Darlington transistors work reliably with 3.3V drive due to their high gain and low Vbe requirement. No level shifting is needed for relay control.

## Wiring Diagram

```
ESP32-S3                    PIC Socket                 MJD122          Relay
GPIO Pin  ─────────────────► PIC Pin ──[RF Choke]──► Base ──► Coil ──► K#
                                                        │
                                                       GND
```

## Complete Pin Summary

| Function | ESP32 GPIO | PIC Pin | Notes |
|----------|:----------:|:-------:|-------|
| L1 (K1)  | 5          | 24      | 0.05 uH |
| L2 (K4) | 6 | 4 | 0.1 uH |
| L3 (K6) | 7 | 25 | 0.22 uH |
| L4 (K8) | 8 | 5 | 0.45 uH |
| L5 (K10) | 9 | 26 | 1.0 uH |
| L6 (K12) | 10 | 7 | 2.2 uH |
| L7 (K14) | 11 | 6 | 4.7 uH |
| C1 (K3) | 12 | 18 | 10 pF |
| C2 (K5) | 13 | 14 | 22 pF |
| C3 (K7) | 14 | 17 | 47 pF |
| C4 (K9) | 15 | 13 | 100 pF |
| C5 (K11) | 16 | 16 | 220 pF |
| C6 (K13) | 17 | 12 | 470 pF |
| C7 (K15) | 18 | 15 | 1 nF |
| Topology (K2) | 48 | 11 | I/O relay |
| FWD ADC | 2 | 2 | AN0 |
| REV ADC | 1 | 3 | AN1 |
| Tune Button | 0 | - | New |
| Bypass Button | 3 | - | New |
| Auto Button | 4 | - | New |
| I2C SDA | 47 | - | New |
| I2C SCL | 21 | - | New |
| CAT RX | 39 | - | New |
| CAT TX | 40 | - | New |
| TX Request | 41 | - | New |
| TX Req Inv | 42 | - | New |
