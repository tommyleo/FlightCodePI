# FlightCodePI hardware connections — 1.6.4

Applies to Pico 2 and Pico 2 W running development firmware 1.6.4. View the board from above with USB at the top. GPIO numbers and physical pin numbers are different.

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

GPIO signals use 3.3 V logic. VTX remains named UART1 in the Configurator and persistent settings for compatibility; internally it uses hardware UART0, auxiliary pin function on GP2/GP3. Receiver PIO remains independent.

The default IMU backend remains MPU6500/MPU9250 over SPI on GP16–GP19. If the optional MPU6050 I2C backend is selected at build time, use GP12 SDA (physical pin 16) and GP13 SCL (physical pin 17), I2C0. GP4/GP5 are reserved for motors.

For simultaneous USB and external regulator power, feed VSYS through a Schottky diode with cathode/band toward VSYS to prevent USB from backfeeding the regulator.

## Migration from v1.6.3

Move VTX TX/RX from GP4/GP5 to GP2/GP3. Move M1/M2/M3/M4 from GP1/GP2/GP3/GP6 to GP4/GP5/GP6/GP7. Move buzzer control from GP7 to GP8. Receiver, VBAT and SPI IMU pins stay the same. Pin assignments are compiled into the firmware and are not changed by restoring a configuration backup.

Before using the new build, update the wiring and verify motor order with propellers removed. Physical signal validation of the new pinout is still required.
