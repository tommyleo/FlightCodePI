# FlightCodePI hardware connections — 1.6.4

Applies to Pico 2 and Pico 2 W running development firmware 1.6.4. View the board from above with USB at the top. GPIO numbers and physical pin numbers are different.

The table shows the default assignment: receiver UART1, VTX UART2.

| Function | Pico GPIO | Physical pin | Direction | Notes |
|---|---|---|---|---|
| SBUS / ELRS receiver signal | GP0 | 1 | Input | Receiver TX → Pico RX; inverted SBUS or non-inverted CRSF, PIO |
| ELRS receiver return signal | GP1 | 2 | Output | Pico TX → receiver RX; bind command only, CRSF telemetry not implemented |
| VTX / digital OSD TX | GP2 | 4 | Output | Pico TX → VTX RX; 115200 baud |
| VTX / digital OSD RX | GP3 | 5 | Input | VTX TX → Pico RX; required for bidirectional HDZero RF control |
| Motor 1 | GP4 | 6 | Output | Rear right, DSHOT |
| Motor 2 | GP5 | 7 | Output | Front right, DSHOT |
| Motor 3 | GP6 | 9 | Output | Rear left, DSHOT |
| Motor 4 | GP7 | 10 | Output | Front left, DSHOT |
| Active buzzer control | GP8 | 11 | Output | Active high; use a suitable driver for the buzzer load |
| VBAT sense | GP26 / ADC0 | 31 | Input | 11:1 divider required; never connect battery directly |
| IMU MISO / AD0 | GP16 | 21 | Input | SPI0 RX |
| IMU NCS | GP17 | 22 | Output | Active low |
| IMU clock / SCL | GP18 | 24 | Output | SPI0 SCK |
| IMU MOSI / SDA | GP19 | 25 | Output | SPI0 TX |
| IMU power | 3V3(OUT) | 36 | Power | 3.3 V |
| Regulated system power | VSYS | 39 | Power | 5 V regulator output; never raw battery voltage |
| Common ground | GND | 3, 8, 13, 18, 23, 28, 33, 38 | Power | Receiver, ESCs, IMU, VTX and regulator share ground |

GPIO signals use 3.3 V logic. The Configurator uses logical UART1 and UART2 as described below. The VTX uses hardware UART0; the receiver uses independent PIO.

The default IMU backend remains MPU6500/MPU9250 over SPI on GP16–GP19. If the optional MPU6050 I2C backend is selected at build time, use GP12 SDA (physical pin 16) and GP13 SCL (physical pin 17), I2C0. GP4/GP5 are reserved for motors.

For simultaneous USB and external regulator power, feed VSYS through a Schottky diode with cathode/band toward VSYS to prevent USB from backfeeding the regulator.

## Migration from v1.6.3

Move VTX TX/RX from GP4/GP5 to GP2/GP3. Move M1/M2/M3/M4 from GP1/GP2/GP3/GP6 to GP4/GP5/GP6/GP7. Move buzzer control from GP7 to GP8. Receiver, VBAT and SPI IMU pins stay the same. Pin assignments are compiled into the firmware and are not changed by restoring a configuration backup.

Before using the new build, update the wiring and verify motor order with propellers removed. Physical signal validation of the new pinout is still required.

## Detailed wiring guides

- [SBUS / ELRS receiver](SBUS_RECEIVER.md)
- [Motors and ESCs](MOTORS_AND_ESC.md)
- [HDZero MSP / DisplayPort](hdzero-msp.md)

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

## VBAT voltage sensing

GP26 (physical pin 31) is the FlightCodePI **VBAT** pad. Build an 11:1 divider
for the nominal battery-voltage conversion:

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

## Selectable serial ports

Firmware reports `SERIAL_PORTS UART1 UART2`. Both ports can be selected for either
SBUS/ELRS receiver or HDZero MSP/DisplayPort VTX. The same port cannot be shared
while VTX is enabled. Selections are saved in flash and restored at startup.
These are logical port names, not RP2350 hardware UART instance numbers.

| Selected function | UART1 (GP0/GP1) | UART2 (GP2/GP3) |
|---|---|---|
| Receiver RX / TX | GP0 (pin 1) / GP1 (pin 2) | GP2 (pin 4) / GP3 (pin 5) |
| VTX TX / RX | GP0 (pin 1) / GP1 (pin 2) | GP2 (pin 4) / GP3 (pin 5) |

Receiver TX is used only for ELRS binding; CRSF telemetry is not implemented.
Receiver RX uses PIO; VTX uses hardware UART0 (auxiliary pin function on GP2/GP3).
TX/RX directions therefore depend on the assigned function, as shown above.

To swap ports, disable VTX and apply, apply the receiver on the desired port,
then select the other VTX port and enable/apply VTX. Save both settings.
Disconnect power before moving wires and verify the new wiring before use.
Motor, buzzer and IMU pins do not change.

The settings schema is now 16. Updating from schema 15 resets settings to
defaults, including receiver UART1 and VTX UART2; configure and save again.
Backups from the previous schema cannot be restored directly. Physical signal
validation on both port assignments is still required.
