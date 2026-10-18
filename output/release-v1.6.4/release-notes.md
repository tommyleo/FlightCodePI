FlightCodePI 1.6.4 — local development build, not published to GitHub.

New pinout for Pico 2 and Pico 2 W: receiver RX GP0 / TX GP1; VTX TX GP2 / RX GP3 (hardware UART0 auxiliary function, still named UART1 in Configurator); motors M1–M4 GP4–GP7; buzzer GP8; VBAT GP26 through the existing 11:1 divider. SPI IMU pins unchanged. Optional I2C IMU moves to GP12 SDA / GP13 SCL to avoid motor conflicts.

Rewire before flashing; see hardware-connections.md for physical pin numbers and migration. Complete settings backups remain compatible, but do not restore the old physical pin assignment.

Both board builds and ELRS/CRSF host tests pass. No physical pin validation or device flash was performed.
