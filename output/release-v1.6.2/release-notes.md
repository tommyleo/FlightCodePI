FlightCodePI 1.6.2 improves startup arming safety and receiver defaults.

- Check ARM on the first valid receiver signal and latch the arming lockout until restart if ARM is already active.
- Show an English OSD warning asking to power off and on again with ARM off, even with the normal OSD layout disabled.
- Keep the onboard status LED steadily lit during the lockout on Pico 2 and Pico 2 W, including without OSD.
- Default to AETR (roll, pitch, throttle, yaw) on a fresh configuration or factory reset. Preserve existing saved mappings.

Includes separate UF2 images for Pico 2 and Pico 2 W.

Validation: Both Pico 2 and Pico 2 W Release firmware targets compile. Boot ARM interlock tests pass. Physical startup and LED/OSD validation remains to be done.

SHA256SUMS.txt covers the release assets.

Rebuilt 2026-10-18 with complete settings backup/restore support for the shared Configurator. Persistent settings layouts, versions and storage addresses are unchanged.
Validation: clean builds of every included target; complete settings protocol, H7 startup/ADC and FlightCode/FlightCodePI ARM interlock host tests passed. Every release image was checked for format integrity and reserved-settings range protection; SHA256SUMS.txt was regenerated. User will perform the next device flash and power-cycle verification.
