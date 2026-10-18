FlightCodePI 1.6.3

- Add complete persistent configuration backup and restore for Pico 2 and Pico 2 W through FlightCode Configurator 1.6.3, including saved OSD settings.
- Validate firmware family, board, settings schema, ordered chunks and CRC before applying a restore; reject unsafe operations while armed or testing motors.
- Preserve existing persistent settings layouts, schema versions and storage addresses.

The Configurator saves and reads back the restored settings, then reboots the board automatically. Backups are for the same board model and compatible firmware family/layout and cannot be loaded into Betaflight.

Includes separate UF2 images for Pico 2 and Pico 2 W. Both Release targets compile; shared settings backup protocol tests pass. UF2 block integrity and version strings verified. No device was flashed as part of this release. SHA256SUMS.txt covers both firmware assets.
