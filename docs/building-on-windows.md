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

Use **Terminal > Run Build Task > Compile Project**. It runs
`build-firmware.ps1`, builds both boards in Release mode and updates the fixed
output directory described below. **Run Project (USB)** first builds, asks
whether the connected board is Pico 2 or Pico 2 W, then loads the matching UF2.
The Pico extension's own compile action also exports the selected board's UF2
to the same directory through the CMake post-build step.

## Build from PowerShell

From the repository root, run:

```powershell
.\build-firmware.ps1
```

The script locates CMake, Ninja, SDK 2.3.0 and toolchain `15_2_Rel1` in
`%USERPROFILE%\.pico-sdk`, so an ordinary PowerShell terminal is sufficient.
It stops immediately if configuration, compilation or export fails.

To compile only one board:

```powershell
.\build-firmware.ps1 -Board pico2
.\build-firmware.ps1 -Board pico2_w
```

## Fixed firmware location

| Board | File to flash |
|---|---|
| Pico 2 | `output/firmware/FlightCodePI-PICO2.uf2` |
| Pico 2 W | `output/firmware/FlightCodePI-PICO2_W.uf2` |

Filenames stay the same across versions. Each file has a `.sha256` companion;
the latest hardware wiring guide is copied to this directory as well. Generated
firmware is ignored by Git. Previous release packages have been removed. Use only these fixed files
when testing the current source. A failed build does not replace the last
successful firmware, so only flash after the build reports success.

The fixed build caches are `build-release` (Pico 2) and
`build-pico2-w-release` (Pico 2 W). ELF/BIN and debug files stay there. The
`build` directory remains available to host tests and the Pico extension.
A CMake post-build hook exports the selected board's UF2 even for direct
CMake or extension builds; the wrapper also exports when no recompilation
is needed.

## Flashing

Disconnect the board, hold BOOTSEL, reconnect USB and release BOOTSEL. Copy
the matching UF2 from `output/firmware` to the bootloader's USB drive, or use
**Run Project (USB)**. For a Pico 2, an explicit picotool command is:

```powershell
picotool load .\output\firmware\FlightCodePI-PICO2.uf2 -fx
```

For Pico 2 W, use `FlightCodePI-PICO2_W.uf2` instead. Do not interchange them.

## Troubleshooting

- **CMake cannot find the Pico SDK**: verify that `PICO_SDK_PATH` points to the
  directory containing `pico_sdk_init.cmake`.
- **CMake, Ninja or the compiler is not found**: let the Pico extension finish
  installing its tools, then open a new Visual Studio Code terminal.
- **The wrong board remains selected**: use a different build directory for
  `pico2` and `pico2_w`, as used by the script, or remove only the stale
  build directory and configure it again.
- **picotool cannot find the board**: reconnect it while holding BOOTSEL and
  confirm that the `RPI-RP2` drive appears in Windows Explorer.
- **PowerShell blocks a project script**: allow scripts only for the current
  process with `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`.

[Back to the main README](../README.md)
