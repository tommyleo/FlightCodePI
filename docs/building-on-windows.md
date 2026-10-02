# Building on Windows

## Requirements

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

## Get and open the source

From PowerShell:

```powershell
git clone https://github.com/tommyleo/FlightCodePI.git
Set-Location .\FlightCodePI
code .
```

When Visual Studio Code asks whether to configure the imported Pico project,
accept it and select `pico2` or `pico2_w`. The repository already contains the
extension settings and build tasks used by the project.

## Build from Visual Studio Code

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

## Build from PowerShell

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

## Build outputs and flashing

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

## Troubleshooting

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

[Back to the main README](../README.md)
