# Firmware build workflow

- Build firmware with `./build-firmware.ps1` (both boards), or `-Board pico2` / `-Board pico2_w` for one target.
- The only current files to offer for flashing are `output/firmware/FlightCodePI-PICO2.uf2` and `output/firmware/FlightCodePI-PICO2_W.uf2`. Confirm the build succeeded before offering them.
- Reuse `build-release` and `build-pico2-w-release`. Do not create new ad hoc build or versioned output directories. Do not keep historical firmware packages locally unless the user requests them.
- Keep the fixed output workflow documented in README.md and docs/building-on-windows.md when changing build tooling.
