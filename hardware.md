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

> **GPIO 48 caveat.** On the ESP32-S3-DevKitC-1 **v1.0** GPIO 48 drives the
> on-board addressable RGB LED. On v1.1 and later the LED moved to GPIO 38 and
> 48 is free. On a v1.0 board the WS2812 will load this line — cut the LED trace
> or move the topology relay to another free pin (35/36/37 on a module without
> octal PSRAM).

> **Hot switching.** Bypass here is software-only: there is no physical bypass
> relay. On an overload the firmware asserts the TX-request/inhibit line within
> a few milliseconds, then waits for forward power to fall to `pwrmax` before
> opening the relays. If RF does not drop it holds the current match rather
> than switching the bank under power. That only ends the overload if something
> stops the carrier, so wire the TX inhibit line if your radio or amplifier
> supports it, or add the hardware trip described under
> [Recommended hardware upgrades](#recommended-hardware-upgrades).

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

**Both bridge inputs must be ADC1 channels (GPIO 1-10).** ADC2 is unavailable
whenever the Wi-Fi radio is active, so putting either detector on GPIO 11-20
would break the moment the web UI is switched on. GPIO 1 and 2 are ADC1_CH0 and
ADC1_CH1.

**Protect the inputs.** These pins connect straight to the detector output.
A 1 kW bridge can swing well past 3.3 V into a fault; the ESP32 has no internal
clamp worth relying on. Fit a series resistor (1-10 k) and a 3.3 V zener or
Schottky clamp to 3V3 at each ADC pin.

## Additional ESP32 Pins (No PIC Equivalent)

These pins are directly connected to the ESP32 for new functionality:

### User Buttons

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 0 | Tune Button | BOOT strapping pin |
| 3 | Bypass Button | JTAG-source strapping pin |
| 4 | Auto Button | No strapping function |

Buttons connect to GND when pressed. `btnlow` defaults to on, which enables the
internal pull-up; setting it off switches the inputs to internal pull-down for
active-high wiring.

> GPIO 0 is the BOOT pin and is shared with the DevKit's own BOOT button.
> Holding the Tune button while resetting the board puts it into the serial
> download bootloader instead of running the firmware. That is harmless, but do
> not hold Tune during a power cycle unless you want to flash.

### I2C bus (display + optional temperature sensor)

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 47 | I2C SDA | General purpose |
| 21 | I2C SCL | General purpose |

Fit 4.7 k pull-ups to 3V3 on both lines if the display module does not have them.

The firmware probes this bus at boot and picks a driver automatically:

| Address | Device |
|:-------:|--------|
| 0x3C, 0x3D | SSD1306 / SSD1309 OLED |
| 0x20 - 0x27 | PCF8574 LCD backpack |
| 0x38 - 0x3F | PCF8574A LCD backpack |
| 0x48 - 0x4F | LM75 / TMP102 / TMP75 temperature sensor |

0x3C/0x3D overlap the PCF8574A range, so an OLED wins the tie; override with
`set disptype 2` if you have an LCD strapped there. Run `i2cscan` to see what is
actually present.

An I2C temperature sensor is the recommended way to add thermal protection,
because it needs no additional GPIO — and there are no spare ADC1 pins on this
board for a thermistor unless you give something else up.

### CAT Interface (UART1)

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 39 | CAT RX | JTAG MTCK |
| 40 | CAT TX | JTAG MTDO |

### TX Request Outputs

| ESP32 GPIO | Function | Pin Type |
|:----------:|:---------|:---------|
| 41 | TX Request | JTAG MTDI |
| 42 | TX Request (inverted) | JTAG MTMS |

The line is asserted around every tune and latched on by protection until
`power reset`. It is most useful wired to a radio's or amplifier's TX inhibit /
interlock input. If you do that, do not also use CAT tune: the tune asserts the
line, which would stop the very carrier CAT tune keyed.

GPIO 39-42 are the JTAG pins. They are ordinary GPIO unless you attach an
external JTAG probe; the built-in USB-Serial-JTAG on GPIO 19/20 is unaffected.

### Status LED

| ESP32 GPIO | Function | Notes |
|:----------:|:---------|:------|
| -1 | Disabled | Set `kPins.ledStatus` in Config.h to enable |

### Antenna selector (optional, disabled by default)

Up to 3 binary-coded outputs select up to 8 antennas. Disabled in Config.h
(`kPins.antenna = {-1,-1,-1}`) because there is no universally free pin.

GPIO 35, 36 and 37 are the usual candidates, but they are consumed by octal
PSRAM on `-N8R8` / `-N16R8` modules. Only enable them on a module without octal
PSRAM. Set `antcount` to the number of antennas once the pins are wired.

## GPIO Classification

### Clean pins (no boot conflicts)
GPIO 1, 2, 5-18, 21, 47 - relays, ADC, I2C

### Strapping pins (safe as inputs after boot)
GPIO 0, 3, 4 - buttons. GPIO 0 is BOOT; see the note above.

### JTAG pins (ordinary GPIO unless an external probe is attached)
GPIO 39, 40, 41, 42 - CAT and TX request

### Special
- GPIO 48 - topology relay; **also the RGB LED on DevKitC-1 v1.0**

### Reserved / avoided
- GPIO 19, 20 - native USB D-/D+ (USB-Serial-JTAG; carries CAT passthrough when `catusb` is on)
- GPIO 26-32 - SPI flash
- GPIO 33-37 - octal PSRAM on -N8R8 / -N16R8 modules
- GPIO 38 - RGB LED on DevKitC-1 v1.1 and later
- GPIO 43, 44 - USB serial UART (UART0)
- GPIO 45, 46 - strapping pins (VDD_SPI, boot mode)

### ADC availability
ADC1 is GPIO 1-10; ADC2 is GPIO 11-20 and is unusable while Wi-Fi is active.
GPIO 1 and 2 carry the bridge inputs, and 5-10 are L relays, so ADC1 is fully
committed. Use an I2C temperature sensor rather than a thermistor.

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
| TX Request | 41 | - | New (see the CAT tune note below) |
| TX Req Inv | 42 | - | New |
| Temp sensor | 47/21 | - | Optional, on the I2C bus at 0x48-0x4F |
| Antenna select | -1 | - | Optional, disabled by default |
| NTC thermistor | -1 | - | Optional, needs a free ADC1 pin |

## Recommended hardware upgrades

None of these are needed to run the firmware, and the current firmware does not
drive them yet. Each one fixes a limit of the stock board that software cannot
fully fix. Pin suggestions assume the pin map above.

**Free pins.** On a module **without** octal PSRAM, GPIO 35, 36 and 37 are free.
On a DevKitC-1 v1.1 or later, GPIO 38 carries the RGB LED, and cutting that
trace frees it. On an `-N8R8` / `-N16R8` module almost nothing is left, which is
one more reason for the external ADC below: it frees GPIO 1 and 2.

### 1. External ADC for the bridge

**Problem.** The ESP32-S3's internal ADC gives roughly 9–10 effective bits, its
calibration curve bends above about 2.5 V at 11 dB attenuation, and its readings
are noisy. Power goes as voltage squared, so across a 1 W to 1 kW range the
forward detector only swings about 32:1 in voltage. At tuning power the detector
sits in the bottom few percent of the ADC's range, exactly where the SWR
readings the tuner relies on are least accurate. The original PIC dealt with
this by switching reference voltages; the ESP32 has no equivalent.

**Fix.** An **ADS1115** (16-bit, programmable gain) on the existing I2C bus at
0x48–0x4B. It needs no GPIO, and moving FWD/REV onto it frees GPIO 1 and 2. Use
a gain of ±4.096 V at high power and ±1.024 V (4× the resolution) near tuning
power; the gain can be switched per reading.

- Wire detector FWD to AIN0 and REV to AIN1, single-ended, each through
  1–10 kΩ with a 3.3 V clamp as described under ADC Inputs, plus 1 nF to ground
  at the ADC pin.
- The ADS1115 tops out at 860 samples/s, so about 400 FWD/REV pairs per second
  when alternating. That is plenty for tuning, but too slow to be the only
  overload detector, so pair it with upgrade 3.
- ADDR pin: tie it so the address does not collide with an LM75 temperature
  sensor. ADDR to VDD gives 0x49; move the LM75 to 0x4A–0x4F.
- The ADS1015 is the 12-bit, 3300 samples/s part in the same package, if speed
  matters more than resolution.

For the widest dynamic range, replace the diode detectors with **AD8307**
logarithmic detectors (about 90 dB range, output linear in dB). That is a
bridge redesign, and the power and SWR maths would change to match.

### 2. RF frequency counter

**Problem.** Without CAT the tuner does not know the frequency, so it cannot
recall memories and has to estimate the band from the load model, which takes
more measurements. Many radios, and all amplifier-only setups, have no CAT.

**Fix.** Count a divided-down RF sample on the ESP32-S3's pulse counter (PCNT).

```
RF sample ──[1-2 turns on the bridge toroid, or a 2 pF tap]──┬──[100 Ω]──┐
                                                             │           │
                                                      2x BAT54S clamp  74LVC1G14
                                                        to 0 V / 3V3   Schmitt
                                                                         │
                                               74HC4040 ripple counter ◄─┘
                                                         │ Q4 (÷16)
                                                         ▼
                                                   ESP32 GPIO (PCNT)
```

- Divide by 16 so 54 MHz becomes 3.4 MHz, comfortably inside what PCNT counts
  reliably, even with its glitch filter enabled.
- A 100 ms gate then resolves 160 Hz, far finer than the 25 kHz memory bins.
- Keep the coupling light (a few volts at 1 kW) and the clamp diodes close to
  the Schmitt input. The counter only needs to work above the tuner's minimum
  tuning power.
- Pin: GPIO 1 or 2 once the ADC moves to the ADS1115, or GPIO 35–38 as
  available.

### 3. Hardware overload trip

**Problem.** Even at ~2 ms, the firmware trip depends on the firmware running.
A crash, a watchdog reset or an OTA reboot is exactly when protection is absent.

**Fix.** A comparator on the forward detector that drives TX inhibit directly
and latches until the firmware clears it.

```
FWD detector ──┬───────────────► + TLV3201 / LM393 ──┐
               │                                     │
Vref (trimmer) ┴──────────────► −                    ▼
                                            74LVC1G74 D-flip-flop (SET)
                                                     │ Q
                                  ┌──────────────────┼────────────────┐
                                  ▼                  ▼                ▼
                         2N7002 → TX inhibit   ESP32 GPIO (IRQ)   LED
                                  ▲
                    ESP32 GPIO ───┘ CLR (firmware reset after 'power reset')
```

- Set the threshold with the trimmer: at the limit power P, the detector reads
  `Vf = 1000 · sqrt(P · pwrscale) / fwdscale + fwdoffset` mV.
- Pull the flip-flop's CLR input low with a resistor so it powers up clear,
  and pull its SET input up so it only trips on a real comparator edge.
- The latch output ORs with GPIO 41 through the 2N7002, so either the firmware
  or the hardware can inhibit TX, and a firmware fault cannot release a
  hardware trip.
- Pins: one input (trip interrupt) and one output (clear), from GPIO 35–38.

### 4. Relay coil economiser

**Problem.** With up to 15 relays energised, coil current is several hundred
milliamps continuously. That heats the enclosure (which the temperature sensor
then reports) and loads the supply.

**Fix.** Switch the relay coil supply with a P-MOSFET under PWM. Drive it at
100% for pull-in, then drop to a hold duty cycle.

- High-side P-MOSFET (e.g. AO3401 for up to 12 V at about 1 A, or DMP3098L) on
  the coil supply rail, gate pulled up to the rail, driven through an NPN or
  2N7002 from an ESP32 LEDC output at 20–25 kHz, which is above audio and
  harmless to HF.
- Every coil must have a flyback diode. The PWM off-time current freewheels
  through them. Check the board has them on every relay; add 1N4148s where it
  does not.
- Firmware sequence (to be added): 100% duty from any relay change until
  `settle` + 20 ms, then 50–60% hold. Do not go below 50%. Hold voltage rises
  with coil temperature, and a relay that drops out under power arcs.
- Add 100 µF + 100 nF at the MOSFET's drain, and keep the PWM loop small.
- Pin: one LEDC-capable output from GPIO 35–38.

### 5. Pull-downs on the relay driver inputs

**Problem.** During reset, flashing and OTA reboots, ESP32 pins float or briefly
take strapping defaults. The MJD122's internal base resistors (about 8 kΩ and
120 Ω) keep a floating base mostly off, but the RF choke in series with each
base picks up RF. A relay chattering during a reboot with RF present is hot
switching.

**Fix.** A 10 kΩ resistor from each driver input (the ESP32 side of the RF
choke) to ground: 15 resistors, or three 8-way 10 kΩ SIP networks. Add 1 nF
across each resistor if RF pickup is a problem.

Pull-downs cannot keep a tune through a reboot, because every relay releases
while the ESP32 restarts. That is why firmware updates should never be done
while transmitting.

