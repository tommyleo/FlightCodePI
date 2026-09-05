#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/control/throttle_ramp.h"

static void close_to(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.03f);
}

int main(void)
{
    const int rates[] = {2000, 8000, 16000};
    for (int r = 0; r < 3; ++r) {
        float state = 20.0f;
        const float dt = 1.0f / rates[r];
        for (int i = 0; i < rates[r] / 10; ++i)
            throttle_ramp_update(&state, 70.0f, 400.0f, dt);
        close_to(state, 45.0f);
        for (int i = 0; i < rates[r] / 10; ++i)
            throttle_ramp_update(&state, 70.0f, 400.0f, dt);
        close_to(state, 70.0f);
        for (int i = 0; i < 100; ++i)
            throttle_ramp_update(&state, 70.0f, 400.0f, dt);
        close_to(state, 70.0f); /* no overshoot */
        close_to(throttle_ramp_update(&state, 10.0f, 400.0f, dt), 10.0f);
        close_to(throttle_ramp_update(&state, 0.0f, 400.0f, dt), 0.0f);
        close_to(throttle_ramp_update(&state, 100.0f, 0.0f, dt), 100.0f);
        close_to(throttle_ramp_update(&state, 12.0f, 0.0f, dt), 12.0f);
        state = 0.0f; /* re-arm must not inherit previous collective */
        assert(throttle_ramp_update(&state, 80.0f, 400.0f, dt) < 0.13f);
    }
    float state = 0.0f;
    /* Mixed elapsed intervals still give 25 points in 100 ms. */
    for (int i = 0; i < 100; ++i) {
        throttle_ramp_update(&state, 100.0f, 400.0f, 0.0003f);
        throttle_ramp_update(&state, 100.0f, 400.0f, 0.0007f);
    }
    close_to(state, 25.0f);
    puts("Throttle ramp behavior tests passed");
    return 0;
}
