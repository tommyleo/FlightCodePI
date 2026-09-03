# FlightCodePI

## FlightCode in action! 🚀

**[Watch the flight video on YouTube](https://youtu.be/JjHND97abkM)**

Quad X rate-mode flight controller for Raspberry Pi Pico 2 and Pico 2 W,
compatible with the shared
**[FlightCode Configurator](https://github.com/tommyleo/FlightCodeConfigurator)**.

The Configurator provides firmware flashing, PID and rate tuning, filter and
motor setup, live telemetry, protected diagnostics, calibration, and flight-log
downloads through a single desktop, web, or Android interface.

## Features

- selectable 8 or 16 kHz main scheduler for motor output and timed system
  tasks;
- PID updates remain synchronized to fresh gyroscope samples;
- with an MPU6500/9250, the gyroscope and PID follow the 8 kHz scheduler
  directly; at 16 kHz the firmware uses the sensor's 32 kHz gyro-only path and
  samples it at the 16 kHz scheduler rate while armed;
- with an MPU6050, the gyroscope and PID remain limited to the configured 1 kHz
  filtered sample rate while the main scheduler and motor output can run at 8
  or 16 kHz;
- MPU6500, MPU9250, or MPU9255 on SPI0;
- 16-channel SBUS receiver;
- DSHOT300, DSHOT600, and DSHOT1200;
- roll, pitch, and yaw PID control with anti-windup and filtered D-term;
- independent rates, expo, feedforward, and TPA;
- persistent configurable gyroscope and D-term low-pass filters, defaulting to
  100 Hz and 60 Hz;
- Quad X mixer with configurable idle and normal/reversed yaw direction;
- three-axis flight-controller alignment in software;
- automatic and manual gyroscope calibration;
- motor test with timeout and ARM-channel interlock;
- PID simulation with physical motor outputs always suppressed;
- extended telemetry and receiver diagnostics;
- GP26/ADC0 battery sensing with Betaflight scale 110 and persistent final
  calibration multiplier;
- persistent 200 Hz flight log sized to the reserved flash area and retained
  above 10% throttle;
- USB BOOTSEL restart from the configurator.
- onboard status LED flashes once per second as a firmware heartbeat and twice
  per second when a valid SBUS signal is present;
- configured buzzer mode produces two short beeps every 500 ms.

Version 2 flight-log metadata and version 7 Configurator JSON logs record both
the measured main-scheduler period and the interval between fresh gyroscope/PID
updates. Each sample exposes the periods in microseconds and their derived
frequencies, alongside the separated P/I/D/FF terms.

## Supported boards

The same flight-control code supports both `pico2` and `pico2_w`. Wi-Fi is not
used. On Pico 2 W, the CYW43 device is initialized only to control the onboard
LED.

The default CMake target is Pico 2. Select the board when configuring a build:

```text
cmake -S . -B build-pico2 -DPICO_BOARD=pico2
cmake -S . -B build-pico2-w -DPICO_BOARD=pico2_w
```

From the Raspberry Pi Pico VS Code extension, **Run Project (USB)** invokes the
internal `Run Project` task. It first compiles the project and then loads it
through `picotool`. The board must be connected by USB; the first installation
may require holding BOOTSEL while connecting it.

## Project structure

```text
src/
├── app/                 Firmware entry point and main flight loop
├── control/             Rate controller, PID logic and mixer configuration
├── drivers/
│   ├── imu/             IMU abstraction and MPU6050/MPU6500 drivers
│   ├── motors/          DSHOT ESC output driver
│   └── receiver/        SBUS receiver and frame decoder
├── protocol/            Shared FlightCode Configurator protocol
└── storage/             Persistent settings and flight log
pio/                     PIO programs for SBUS and DSHOT
docs/                    Dedicated hardware wiring guides
```

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

See [HARDWARE.md](HARDWARE.md) for the complete pinout, plus the dedicated
[SBUS receiver](docs/SBUS_RECEIVER.md) and
[motors/ESC](docs/MOTORS_AND_ESC.md) wiring guides.
