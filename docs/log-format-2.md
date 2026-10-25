# Flight logging in 1.6.4

Firmware and Configurator version: **1.6.4**. JSON/sample schema: **2.0**.
Metadata schema: **6** (128 bytes). Previous log storage headers/catalogs are
invalidated; record a new flight after upgrading. Saved flight settings are not reset.

## Internal logger

500 Hz clock-based sampling; 64-byte records. RAM budget is unchanged from the
old 40-byte, 200 Hz logger. PI retains 1597 records (about 3.19 seconds); STM32
internal targets retain floor(old_capacity * 40 / 64), about 2.56-3.36 seconds.
Only the newest window is retained, followed by a timestamped stop marker.
`overwrittenRecords` is a saturating cumulative count of evicted ring records,
not an external-backend failure count. Flash writes happen after disarming.

## Signals and timing

`gyroRaw`: aligned and bias-corrected gyro before software gyro LPF.
`gyro`: actual software-filtered gyro supplied to the controller.
`dTermUnfiltered`: derivative of the filtered gyro before D LPF, with the same
gain, TPA and dynamic boost as filtered D. It is not a derivative of raw gyro.
`dTermFiltered` / `dTerm`: actual D contribution, at 0.01 percent resolution.
`pid`: measured final axis PID before mixer/authority scaling, at 0.5 percent.
Gyro quantization is 0.1 deg/s. D values saturate at int16 bounds
(-327.68..327.67 percent); PID/P/I/FF at -64..63.5 percent.
Flag 0x40 / `signalClipped` marks quantization saturation in a recorded signal.

`timestampUs` comes from the MCU microsecond clock. Configurator accumulates
unsigned deltas, preserving gaps and clock rollover. Exported `t` starts at zero
for the first retained record; it is never generated from the sample index.
The stop marker copies the last measured signals and sets motors to zero; it
is an event, not a new IMU measurement. Exclude it from spectra/statistics.

## External Blackbox (FlightCode)

1 kHz clock-based sampling and existing 60-byte block layout, format byte 0x20.
`droppedRecords` counts persistent-backend queue drops (saturates at 65535).
Internal and external record counters have distinct names and meanings.

## Analysis limits

No dedicated anti-alias filter is applied to logger decimation. Nyquist is
250 Hz internally and 500 Hz externally; higher-frequency vibration may fold
into these bands. A peak alone is insufficient to choose a notch frequency.
Real gyro/D pairs are logged independently; no compatibility copies or
reconstructed PID values are exported. Missing/old metadata is rejected.
`receiverDiagnosticsAtDownload` is live receiver state, not flight evidence.
