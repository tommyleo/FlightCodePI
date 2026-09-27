# FlightCodePI

Current release: **1.4.0**.

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
- 16-channel SBUS or ELRS/CRSF receiver on GP0;
- digital OSD via MSP DisplayPort at 115200 baud on UART1 TX / GP4;
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

## Building on Windows

### Requirements

Use a 64-bit Windows PC with:

- Git for Windows;
- Visual Studio Code;
- the official **Raspberry Pi Pico** extension for Visual Studio Code;
- Windows PowerShell 5.1 or PowerShell 7.

The extension installs the Pico SDK, CMake, Ninja, the Arm toolchain, picotool
and OpenOCD under `%USERPROFILE%\.pico-sdk`. This project currently selects Pico
SDK 2.3.0 and the `15_2_Rel1` toolchain in `CMakeLists.txt`. Allow the extension
to finish downloading those components before the first build. An internet
connection is required for the initial installation.

### Get and open the source

From PowerShell:

```powershell
git clone https://github.com/tommyleo/FlightCodePI.git
Set-Location .\FlightCodePI
code .
```

When Visual Studio Code asks whether to configure the imported Pico project,
accept it and select `pico2` or `pico2_w`. The repository already contains the
extension settings and build tasks used by the project.

### Build from Visual Studio Code

Open the Command Palette with `Ctrl+Shift+P` and run the Pico extension's
compile command, or use **Terminal > Run Build Task > Compile Project**. The
default CMake board is `pico2`.

To build and immediately load the connected board, run **Run Project (USB)**.
The first USB load can require this sequence:

1. disconnect the board;
2. hold **BOOTSEL**;
3. reconnect USB;
4. release **BOOTSEL** when the `RPI-RP2` drive appears;
5. run **Run Project (USB)** again.

### Build from PowerShell

Open a new integrated terminal in Visual Studio Code so the extension-provided
environment is available, then configure the required board.

For Raspberry Pi Pico 2:

```powershell
cmake -S . -B build-pico2 -G Ninja `
  -DPICO_BOARD=pico2 `
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-pico2
```

For Raspberry Pi Pico 2 W:

```powershell
cmake -S . -B build-pico2-w -G Ninja `
  -DPICO_BOARD=pico2_w `
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-pico2-w
```

To use an ordinary PowerShell window instead, initialize the paths installed
by the extension first:

```powershell
$env:PICO_SDK_PATH = "$env:USERPROFILE\.pico-sdk\sdk\2.3.0"
$env:PICO_TOOLCHAIN_PATH = "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1"
$env:Path = "$env:USERPROFILE\.pico-sdk\cmake\v4.3.4\bin;" +
            "$env:USERPROFILE\.pico-sdk\ninja\v1.13.2;" +
            "$env:USERPROFILE\.pico-sdk\picotool\2.3.0\picotool;" +
            "$env:PICO_TOOLCHAIN_PATH\bin;$env:Path"
```

Then run either pair of CMake commands shown above. These environment changes
apply only to the current PowerShell process.

### Build outputs and flashing

A successful build generates the following files in the selected build
directory:

```text
FlightCodePI.uf2
FlightCodePI.elf
FlightCodePI.bin
FlightCodePI.hex
```

The simplest manual flashing method is to start the board in BOOTSEL mode and
copy `FlightCodePI.uf2` to the `RPI-RP2` USB drive. Alternatively, use **Run
Project (USB)** or picotool:

```powershell
picotool load .\build-pico2\FlightCodePI.uf2 -fx
```

Use the output from `build-pico2-w` when flashing a Pico 2 W.

### Troubleshooting

- **CMake cannot find the Pico SDK**: verify that `PICO_SDK_PATH` points to the
  directory containing `pico_sdk_init.cmake`.
- **CMake, Ninja or the compiler is not found**: let the Pico extension finish
  installing its tools, then open a new Visual Studio Code terminal.
- **The wrong board remains selected**: use a different build directory for
  `pico2` and `pico2_w`, as in the commands above, or remove only the stale
  build directory and configure it again.
- **picotool cannot find the board**: reconnect it while holding BOOTSEL and
  confirm that the `RPI-RP2` drive appears in Windows Explorer.
- **PowerShell blocks a project script**: allow scripts only for the current
  process with `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`.

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

## AM32 ESC configurator

FlightCodePI exposes a Betaflight-compatible MSP/4-way passthrough to
[AM32 Configurator](https://am32.ca/configurator). Remove the propellers,
connect USB, open the AM32 configurator in Chrome or Edge, select the
FlightCodePI serial port, and only then power the ESCs. The passthrough is
rejected while the flight controller is armed. It supports AM32 ARM
bootloader discovery, settings read/write and firmware updates over each
motor signal wire; legacy Atmel and Silabs ESC bootloaders are not supported.

AM32 bootloader discovery uses the current 21-byte BLHeli probe: twelve
leading `0x00` bytes followed by `0D 42 4C 48 65 6C 69 F4 7D`.  Do not use
the legacy 17-byte/eight-zero probe: current AM32 bootloaders do not answer it,
and the configurator reports `cmd_DeviceInitFlash: ACK_D_GENERAL_ERROR`.
The STM32 implementation has been verified on a CLRacingF4 with a SEQURE
4-in-1 F421 ESC running AM32 2.17 (bootloader v13), with all four ESC channels
detected through their motor signal wires. FlightCodePI uses the same 4-way
and AM32 bootloader protocol implementation.

See [HARDWARE.md](HARDWARE.md) for the complete pinout, plus the dedicated
[SBUS receiver](docs/SBUS_RECEIVER.md) and
[motors/ESC](docs/MOTORS_AND_ESC.md) wiring guides.

## Direct throttle and storage formats

Throttle is applied directly to the controller and mixer on each update.
Log metadata uses version 5 with integer PID/feedforward values; exports contain a single throttle channel.
Settings saved by the preceding firmware version are not migrated; configure
and save the aircraft settings again after updating. Existing onboard logs
from the preceding format are not loaded.
