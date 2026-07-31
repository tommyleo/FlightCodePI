# FlightCodePI

Quad X rate-mode flight controller for Raspberry Pi Pico 2 W, compatible with
the shared configurator at `C:\SvilST\FlightCodeConfigurator`.

## Features

- 8 kHz control loop and gyroscope sampling;
- MPU6500, MPU9250, or MPU9255 on SPI0;
- 16-channel SBUS receiver;
- DSHOT150, DSHOT300, and DSHOT600;
- roll, pitch, and yaw PID control with anti-windup and filtered D-term;
- independent rates, expo, feedforward, and TPA;
- Quad X mixer with configurable idle and normal/reversed yaw direction;
- three-axis flight-controller alignment in software;
- automatic and manual gyroscope calibration;
- motor test with timeout and ARM-channel interlock;
- PID simulation with physical motor outputs always suppressed;
- extended telemetry and receiver diagnostics;
- persistent 200 Hz flight log with 4,096 samples;
- USB BOOTSEL restart from the configurator.

## Persistent storage

The last flash sector stores all settings. The preceding 25 sectors are
reserved for the latest flight log. These areas are separate and do not
overlap the firmware.

## Arming safety

The firmware requires:

1. a valid IMU and completed calibration;
2. a valid SBUS signal;
3. the ARM channel to have entered the low state first;
4. throttle at or below 5% when arming;
5. the configurator to be disconnected, except during protected PID simulation.

During PID simulation, control calculations remain active while all four
physical DSHOT outputs are forced to zero.

See [HARDWARE.md](HARDWARE.md) for wiring information.
