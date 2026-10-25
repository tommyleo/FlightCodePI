#include <assert.h>
#include <stdio.h>
#include "../src/control/rate_controller.c"

static flight_settings_t settings;
const flight_settings_t *flight_settings_get(void) { return &settings; }

int main(void)
{
    settings.main_loop_hz = 16000;
    settings.roll = (pid_settings_t){110,180,105};
    settings.pitch = (pid_settings_t){110,200,105};
    settings.yaw = (pid_settings_t){150,260,0};
    settings.gyro_lpf_hz = 92;
    settings.dterm_lpf_hz = 50;
    settings.roll_rate_dps = settings.pitch_rate_dps = 420;
    settings.yaw_rate_dps = 320;
    rate_controller_init();
    imu_sample_t sample = {.valid=true,.accel_z_g=1,.sample_time_us=63};
    rate_controller_output_t output = {0};
    for (unsigned i=0;i<GYRO_CALIBRATION_SAMPLES;i++) {
        rate_controller_update(&sample,false,0,0,0,0,&output);
        sample.sample_time_us+=63;
    }
    assert(rate_controller_is_calibrated());
    rate_controller_update(&sample,true,20,0,0,0,&output);
    sample.sample_time_us+=63;
    sample.gyro_x_dps=100; sample.gyro_y_dps=-200; sample.gyro_z_dps=300;
    assert(rate_controller_update(&sample,true,20,0,0,0,&output));
    assert(output.gyro_raw_dps[0] == 100*GYRO_ROLL_SIGN);
    assert(output.gyro_raw_dps[1] == -200*GYRO_PITCH_SIGN);
    assert(output.gyro_raw_dps[2] == 300*GYRO_YAW_SIGN);
    assert(fabsf(output.roll_rate_dps) < fabsf(output.gyro_raw_dps[0]));
    assert(fabsf(output.d_term_percent[0]) < fabsf(output.d_unfiltered_percent[0]));
    assert(fabsf(output.d_term_percent[1]) < fabsf(output.d_unfiltered_percent[1]));
    assert(output.d_term_percent[2] == 0 && output.d_unfiltered_percent[2] == 0);
    puts("Controller exports independent raw/filtered gyro and D signals with matching axis signs");
}
