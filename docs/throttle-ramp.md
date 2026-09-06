# Throttle rise limit

Throttle rise limiting is fixed at a 5 ms full-scale rise time. It is not exposed
as a setting or protocol capability. The legacy persisted field remains reserved
so existing settings layouts can still be loaded, but its value is ignored.

At 5 ms, a 0–100% step takes 5 ms and a 20–70% step takes 2.5 ms. Falling demand
below the applied collective takes effect immediately. Idle is not ramped.
PID corrections remain outside the limiter; airmode can still raise individual
motors or the collective to preserve attitude. TPA and airmode activation use the
limited collective. Arm checks continue to use the raw receiver throttle.
Disarm, receiver loss and controller reset clear the limiter state.

Blackbox throttle remains the raw receiver demand. Metadata v3 captures the rise
time as 5 ms and exports it as `flightConfiguration.throttleRiseMs` and
`throttleRiseMs`. Null means not recorded in a legacy log. The limited collective
is not recorded as a sample channel.
Legacy v2 metadata and sample offsets remain readable after the firmware update.

Validate motor behavior without propellers before flight; bench/software checks do
not establish flight performance.
