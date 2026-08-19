# FlightCodePI hardware connections

Reference orientation: Raspberry Pi Pico 2 or Pico 2 W viewed from above, with
the USB connector at the top.

## Complete pin assignment

| Function | Pico GPIO | Physical pin | Direction | Notes |
|---|---:|---:|---|---|
| SBUS receiver signal | GP0 | 1 | Input | Inverted SBUS, 100 kbit/s |
| Motor 1 ESC signal | GP1 | 2 | Output | Rear right, DSHOT |
| Motor 2 ESC signal | GP2 | 4 | Output | Front right, DSHOT |
| Motor 3 ESC signal | GP3 | 5 | Output | Rear left, DSHOT |
| Motor 4 ESC signal | GP6 | 9 | Output | Front left, DSHOT |
| Active buzzer control | GP7 | 10 | Output | Active high, controlled by CH5 |
| VBAT sense pad | GP26 / ADC0 | 31 | Input | 11:1 divider required; never connect the battery directly |
| IMU MISO / AD0 | GP16 | 21 | Input | SPI0 RX |
| IMU chip select / NCS | GP17 | 22 | Output | Active low |
| IMU clock / SCL | GP18 | 24 | Output | SPI0 SCK |
| IMU MOSI / SDA | GP19 | 25 | Output | SPI0 TX |
| IMU power | 3V3(OUT) | 36 | Power | 3.3 V only |
| Common ground | GND | 3, 8, 13, 18, 23, 28, 33, 38 | Power | Receiver, ESCs, IMU and Pico must share ground |

Detailed guides:

- [SBUS receiver wiring](docs/SBUS_RECEIVER.md)
- [Motors and ESC wiring](docs/MOTORS_AND_ESC.md)

## MPU6500 / MPU9250 / MPU9255 (SPI0)

| IMU module | Pico 2 / Pico 2 W | Physical pin |
|---|---|---:|
| VCC | 3V3(OUT) | 36 |
| GND | GND | 23 |
| SCL | GP18 / SPI0 SCK | 24 |
| SDA | GP19 / SPI0 TX (MOSI) | 25 |
| AD0 | GP16 / SPI0 RX (MISO) | 21 |
| NCS | GP17 / chip select | 22 |
| INT | Not connected | - |
| EDA | Not connected | - |
| ECL | Not connected | - |
| FSYNC | Not connected | - |

Power the module from 3V3(OUT) only, not from VBUS. Keep the SPI wires short,
ideally below 10 cm, and route a ground wire close to the signal wires.

Mount the IMU flat and firmly, away from motor wiring. As an initial
orientation, point the printed X-axis arrow toward the front of the quad and
verify the direction of all three axes in the Configurator before fitting the
propellers.

## Buzzer

GP7, physical pin 10, produces two short active-high pulses every 500 ms when
receiver channel 5 is above 2000 us. The active buzzer determines the pitch;
the firmware controls its double-beep envelope.
Use a 3.3 V active buzzer only if its current is within the GPIO limit. For a
5 V or higher-current buzzer, drive it through a transistor or MOSFET with a
flyback diode when required. Always connect the driver ground to Pico GND.

## VBAT voltage sensing

GP26 (physical pin 31) is the FlightCodePI **VBAT** pad. Build an 11:1 divider
to match Betaflight's standard `vbat_scale=110`:

```text
Battery + ---- 100 kΩ ----+---- GP26 / VBAT
                          |
                         10 kΩ
                          |
Battery - / GND ----------+---- Pico GND
```

An optional 100 nF ceramic capacitor from GP26/VBAT to GND reduces motor and
ESC noise. The 100 kΩ/10 kΩ divider supports batteries up to 8S while keeping
the ADC input below 3.3 V. Never connect battery voltage directly to GP26.
The Configurator Setup page provides a persistent 0.500–1.500 final multiplier,
defaulting to 1.000; compare its reading with a multimeter before flight.

## Power and grounding

Do not power motors or ESC power stages from the Pico. Power them from the
battery and power-distribution system. If an ESC/BEC powers the Pico, use a
clean regulated supply suitable for VSYS and never feed that supply into the
3V3(OUT) pin.

All signal devices must share a common ground. A practical layout uses one
ground close to GP0 for the receiver and grounds close to the motor signal pins
for the ESC signal returns.

## Currently unused GPIOs

GP4, GP5, GP8-GP15, GP20-GP22, GP27 and GP28 are not assigned by the current
firmware. Do not connect new peripherals to them without also checking future
firmware changes and the Pico board documentation.
