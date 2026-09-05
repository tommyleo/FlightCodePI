#pragma once

/* Percent per second is 100000 / rise_ms. Only the collective demand
 * is limited; the mixer may still change any motor instantly for attitude.
 * The caller resets state on disarm/failsafe and supplies elapsed seconds. */
static inline float throttle_ramp_update(float *state, float target,
                                          float rise_ms, float dt)
{
    if (rise_ms <= 0.0f || target <= *state) {
        *state = target;
    } else {
        const float next = *state + 100000.0f * dt / rise_ms;
        *state = next < target ? next : target;
    }
    return *state;
}
