# FlightCodePI hardware connections

Reference orientation: Raspberry Pi Pico 2 W viewed from above, with the USB
connector at the top.

## MPU6500 / MPU9250 / MPU9255 (SPI0)

| IMU module | Pico 2 W | Physical pin |
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
