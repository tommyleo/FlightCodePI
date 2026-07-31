# FlightCodePI - collegamenti hardware

Orientamento di riferimento: Raspberry Pi Pico 2 W visto dall'alto, con la
porta USB verso l'alto.

## MPU6500 / MPU9250 / MPU9255 (SPI0)

| Modulo IMU | Pico 2 W | Pin fisico |
|---|---|---:|
| VCC | 3V3(OUT) | 36 |
| GND | GND | 23 |
| SCL | GP18 / SPI0 SCK | 24 |
| SDA | GP19 / SPI0 TX (MOSI) | 25 |
| AD0 | GP16 / SPI0 RX (MISO) | 21 |
| NCS | GP17 / chip select | 22 |
| INT | non collegato | - |
| EDA | non collegato | - |
| ECL | non collegato | - |
| FSYNC | non collegato | - |

Alimentare il modulo esclusivamente da 3V3(OUT), non da VBUS. Tenere i cavi
SPI corti, idealmente sotto 10 cm, e far correre GND vicino ai segnali.

Montare la IMU piatta e rigida, lontano dai cavi dei motori. Come prima
orientazione, puntare la freccia X serigrafata verso il muso del quad e
verificare nel Configurator il verso dei tre assi prima di collegare le eliche.
