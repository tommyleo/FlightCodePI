# SBUS receiver wiring

FlightCodePI reads a standard inverted SBUS signal at 100 kbit/s using PIO0.
The firmware decodes 16 receiver channels and rejects frames marked as lost or
failsafe.

## Connections

| Receiver connection | Raspberry Pi Pico 2 / Pico 2 W |
|---|---|
| SBUS signal | GP0, physical pin 1 |
| Ground | GND, physical pin 3 recommended |
| Receiver power | Supply required by the receiver; do not assume 3.3 V |

The receiver and Pico must share ground. Power the receiver according to its
own specification, normally from a regulated BEC output. Do not power a 5 V
receiver from the Pico 3V3(OUT) pin.

## Signal level and polarity

RP2350 GPIO inputs are 3.3 V logic and are not 5 V tolerant. Before connecting
the SBUS signal, verify its voltage with the receiver documentation or a meter.
Use a suitable level shifter if the receiver output can exceed 3.3 V.

Connect the receiver's standard **inverted SBUS** output. The current firmware
performs the SBUS inversion in software. An uninverted SBUS pad will therefore
not decode correctly unless an external inverter is added or the firmware is
changed.

## Channel assignment

| SBUS channel | FlightCodePI function |
|---:|---|
| CH1 | Throttle |
| CH2 | Roll |
| CH3 | Pitch |
| CH4 | Yaw |
| CH5 | Buzzer |
| CH6 | Arm/disarm switch |
| CH7-CH16 | Available to telemetry, currently unused for flight control |

Configure CH5 and CH6 so their high position reaches slightly above 2000 us.
The Configurator receiver page shows the measured channel values.

## Arming checks

FlightCodePI only arms when all of the following are true:

1. A valid SBUS signal is present and failsafe is inactive.
2. CH6 has first been observed in its low position.
3. Throttle is at or below 5%.
4. IMU calibration is complete.
5. The Configurator is disconnected, except during protected PID simulation.

Configure receiver failsafe to lower throttle and CH6. Confirm the behavior in
the Configurator with all propellers removed.

## Quick verification

1. Remove every propeller and leave the flight battery disconnected.
2. Connect the receiver, Pico USB and common ground.
3. Open the Receiver page in the Configurator.
4. Confirm CH1-CH6 move in the expected direction and reach their endpoints.
5. Turn off the transmitter and verify that receiver signal is reported lost.
