# Throttle rise limit

The PID page exposes **Throttle rise limit / Full-scale rise time**, 0–1000 ms.
Default 0 disables it, including when migrating existing settings (v9 to v10).
Apply changes in RAM; Save to flash persists them. Changes are rejected while armed.

At 300 ms, a 0–100% step takes 300 ms and a 20–70% step takes 150 ms.
Larger values soften acceleration more and delay requested thrust. Falling demand
below the applied collective takes effect immediately. Idle is not ramped.
PID corrections remain outside the limiter; airmode can still raise individual
motors or the collective to preserve attitude. TPA and airmode activation use the
limited collective. Arm checks continue to use the raw receiver throttle.
Disarm, receiver loss and controller reset clear the limiter state.

Protocol capability: THROTTLE_RAMP. GET_THROTTLE_RAMP returns
`@CFG THROTTLE_RAMP <milliseconds> <saved>`. SET_THROTTLE_RAMP accepts a finite
value from 0 to 1000; SAVE_SETTINGS persists it. Older firmware leaves the UI disabled.

Blackbox throttle remains the raw receiver demand. Metadata v3 captures the rise
time at flight start and exports it as `flightConfiguration.throttleRiseMs` and
`throttleRiseMs` (milliseconds). Zero means disabled; null means not recorded in
a legacy log. The limited collective is not recorded as a sample channel.
Legacy v2 metadata and sample offsets remain readable after the firmware update.

Validate motor behavior without propellers before flight. This limiter is experimental;
bench/software checks do not establish flight performance.
