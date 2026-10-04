# HDZero MSP + DisplayPort

Select **HDZero V3 · MSP + DisplayPort** and a free UART in the VTX tab.
Connect **FC TX → VTX RX**, **FC RX → VTX TX**, and ground. Both signal wires
are needed for RF control. OSD alone can still work with only FC TX connected.
On FlightCodePI use UART1: **GP4 TX / GP5 RX**.

Save and reboot with the VTX powered. FlightCode replies to HDZero's MSP
handshake and supplies the selected band, channel, frequency and power while
continuing the centered 30 × 16 OSD on the same 115200-baud UART. Band/channel
select the frequency from HDZero's native table: R1–R8, E1, F1/F2/F4 and L1–L8.
Low-band availability depends on the VTX being unlocked. The supported power
choices are **25 and 200 mW**, matching the Configurator's Race V3 profile.
There is no arbitrary-frequency, Pit Mode or higher-power selection in this
implementation. DJI / Walksnail RF control is not supported by this mode.

While armed, RF settings are frozen. Changing the UART or protocol requires
rebooting; changing channel/power on an already active MSP UART is transmitted
while disarmed. Turning the OSD off does not disable MSP VTX control.

The VTX page updates the link status once per second without replacing local
edits. **Settings sent** means the VTX is communicating and the MSP settings
have been queued, not that its RF channel/power has been read back. With only
USB power, an unpowered VTX reports **No response**. Verify the actual channel
and power in the goggles during bench testing.

The digital UART uses the BTFL compatibility identifier because HDZero gates
MSP RF configuration on known FC identifiers. This does not change FlightCode's
USB identity. Remote table uploads, VTX-side configuration writes, EEPROM writes
and stick-operated HDZero CMS are not supported; settings remain controlled by
the FlightCode Configurator. MSPv1 and native MSPv2 request framing are checked,
including checksums, payload bounds and partial-frame timeouts. Receive work
and transmission are bounded and do not wait for a powered VTX.

Software tests cover handshake and frame contents, valid/invalid native
channels, power indices, armed settings, corrupt/oversized/partial requests,
queue retries, link timeout and clock wrap. Physical HDZero validation is
still required.

Protocol references: [HDZero firmware](https://github.com/hd-zero/hdzero-vtx/blob/main/src/msp_displayport.c)
and [Betaflight MSP VTX](https://github.com/betaflight/betaflight/blob/master/src/main/io/vtx_msp.c).
