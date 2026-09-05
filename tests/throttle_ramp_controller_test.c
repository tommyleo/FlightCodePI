#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "flight_settings.h"
#include "rate_controller.h"

static flight_settings_t settings;
static imu_sample_t imu;
static rate_controller_output_t output;
const flight_settings_t *flight_settings_get(void) { return &settings; }
static void tick(bool armed, uint8_t gas, int8_t roll)
{
    imu.sample_time_us += 125;
    rate_controller_update(&imu, armed, gas, roll, 0, 0, &output);
}
static float mean_motor(void)
{
    return (output.motor_percent[0]+output.motor_percent[1]+
            output.motor_percent[2]+output.motor_percent[3])/4.0f;
}
int main(void)
{
    settings.motor_idle_percent=5.0f;
    settings.throttle_rise_ms=400.0f;
    settings.gyro_lpf_hz=100.0f; settings.dterm_lpf_hz=60.0f;
    settings.roll_rate_dps=400.0f; settings.roll.kp=0.1f;
    imu.valid=true; imu.accel_z_g=1.0f;
    rate_controller_init();
    for(int i=0;i<8100;i++)tick(false,0,0);
    assert(rate_controller_is_calibrated());
    for(int i=0;i<800;i++)tick(true,80,0);
    assert(fabsf(mean_motor()-(5.0f+25.0f*0.95f))<0.03f);
    /* Attitude correction is immediate even with an unfinished ramp. */
    tick(true,80,10);
    assert(output.motor_percent[2]-output.motor_percent[0]>7.9f);
    tick(true,10,0);
    assert(fabsf(mean_motor()-14.5f)<0.01f);
    tick(false,80,0);
    assert(mean_motor()==0.0f);
    tick(true,80,0);
    assert(mean_motor()<5.04f); /* no retained collective on re-arm */
    rate_controller_reset(); /* main loop also uses this for failsafe */
    tick(true,80,0);
    assert(mean_motor()<5.04f);
    settings.throttle_rise_ms=0.0f;
    tick(true,80,0);
    assert(fabsf(mean_motor()-81.0f)<0.01f);
    puts("Controller ramp, PID authority and disarm tests passed");
}
