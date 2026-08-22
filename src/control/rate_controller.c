#include "rate_controller.h"

#include <math.h>
#include <string.h>

#include "flight_control_config.h"
#include "flight_settings.h"

#define CALIBRATION_MAX_VARIATION_DPS 3.0f
#define CALIBRATION_MAX_ABSOLUTE_RATE_DPS 10.0f
#define CALIBRATION_MAX_CONSECUTIVE_OUTLIERS 16u
#define GYRO_BIAS_TRACK_MAX_RATE_DPS 5.0f
#define GYRO_BIAS_TRACK_MIN_ACCEL_G 0.75f
#define GYRO_BIAS_TRACK_MAX_ACCEL_G 1.25f
#define GYRO_BIAS_TRACK_SETTLE_S 1.0f
#define GYRO_BIAS_TRACK_TIME_CONSTANT_S 2.0f
#define YAW_ITERM_RELAX_LPF_HZ 8.0f
#define YAW_ITERM_RELAX_THRESHOLD_DPS 20.0f
#define AIRMODE_ACTIVATION_THROTTLE_PERCENT 10.0f

typedef struct {
    float integral;
    float previous_rate;
    float dterm;
    float relaxed_setpoint;
} pid_state_t;

typedef struct {
    float value;
    bool initialized;
} pt1_filter_t;

static pid_state_t roll_pid;
static pid_state_t pitch_pid;
static pid_state_t yaw_pid;
static pt1_filter_t gyro_filter[3];
static float gyro_bias_x;
static float gyro_bias_y;
static float gyro_bias_z;
static float calibration_sum_x;
static float calibration_sum_y;
static float calibration_sum_z;
static float calibration_reference_x;
static float calibration_reference_y;
static float calibration_reference_z;
static uint16_t calibration_samples;
static uint16_t calibration_outliers;
static uint32_t previous_sample_time_us;
static float stationary_time_s;
static bool calibrated;
static bool mixer_saturated;
static bool airmode_active;

static float clamp_float(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

static float pt1(pt1_filter_t *filter, float input, float cutoff_hz, float dt)
{
    if (!filter->initialized) {
        filter->value = input;
        filter->initialized = true;
        return input;
    }
    const float rc = 1.0f / (2.0f * FLIGHT_PI_F * cutoff_hz);
    filter->value += (dt / (rc + dt)) * (input - filter->value);
    return filter->value;
}

static float rate_setpoint(int8_t percent, float maximum, float expo)
{
    const float x = clamp_float((float)percent / 100.0f, -1.0f, 1.0f);
    return ((1.0f - expo) * x + expo * x * x * x) * maximum;
}

static float pid_update(pid_state_t *state,
                        const pid_axis_t *gains,
                        float setpoint,
                        float measured_rate,
                        float feedforward,
                         float dt,
                         float output_limit,
                         float tpa_factor,
                         float dterm_lpf_hz, bool integral_enabled,
                          bool relax_integral,
                          float *p_out, float *i_out,
                          float *d_out, float *ff_out)
{
    const float error = setpoint - measured_rate;
    float integral_factor = 1.0f;
    if (relax_integral) {
        const float relax_rc =
            1.0f / (2.0f * FLIGHT_PI_F *
                    YAW_ITERM_RELAX_LPF_HZ);
        state->relaxed_setpoint +=
            (dt / (relax_rc + dt)) *
            (setpoint - state->relaxed_setpoint);
        const float setpoint_highpass =
            fabsf(setpoint - state->relaxed_setpoint);
        integral_factor = clamp_float(
            1.0f - setpoint_highpass / YAW_ITERM_RELAX_THRESHOLD_DPS,
            0.0f, 1.0f);
    }
    if (!integral_enabled) {
        /* Do not store corrections while armed on the ground. */
        state->integral = 0.0f;
    } else if (!mixer_saturated || state->integral * error < 0.0f) {
        /* Never slow down unwinding an I term that opposes the error. */
        if (state->integral * error < 0.0f) integral_factor = 1.0f;
        state->integral = clamp_float(
            state->integral +
                gains->ki * error * dt * integral_factor,
            -PID_INTEGRAL_LIMIT_PERCENT,
            PID_INTEGRAL_LIMIT_PERCENT);
    }

    const float derivative =
        -(measured_rate - state->previous_rate) / dt;
    state->previous_rate = measured_rate;
    const float d_rc =
        1.0f / (2.0f * FLIGHT_PI_F * dterm_lpf_hz);
    state->dterm += (dt / (d_rc + dt)) * (derivative - state->dterm);

    *p_out = gains->kp * tpa_factor * error;
    *i_out = state->integral;
    *d_out = gains->kd * tpa_factor * state->dterm;
    *ff_out = feedforward * setpoint;
    return clamp_float(*p_out + *i_out + *d_out + *ff_out,
                       -output_limit,
                       output_limit);
}

void rate_controller_reset(void)
{
    memset(&roll_pid, 0, sizeof(roll_pid));
    memset(&pitch_pid, 0, sizeof(pitch_pid));
    memset(&yaw_pid, 0, sizeof(yaw_pid));
    memset(gyro_filter, 0, sizeof(gyro_filter));
    mixer_saturated = false;
    airmode_active = false;
}

void rate_controller_start_calibration(void)
{
    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0f;
    calibration_sum_x = calibration_sum_y = calibration_sum_z = 0.0f;
    calibration_reference_x =
        calibration_reference_y = calibration_reference_z = 0.0f;
    calibration_samples = 0u;
    calibration_outliers = 0u;
    previous_sample_time_us = 0u;
    stationary_time_s = 0.0f;
    calibrated = false;
    rate_controller_reset();
}

static void track_stationary_gyro_bias(const imu_sample_t *imu, float dt)
{
    const float corrected_x = imu->gyro_x_dps - gyro_bias_x;
    const float corrected_y = imu->gyro_y_dps - gyro_bias_y;
    const float corrected_z = imu->gyro_z_dps - gyro_bias_z;
    const float accel_norm = sqrtf(imu->accel_x_g * imu->accel_x_g +
                                   imu->accel_y_g * imu->accel_y_g +
                                   imu->accel_z_g * imu->accel_z_g);
    const bool stationary =
        fabsf(corrected_x) < GYRO_BIAS_TRACK_MAX_RATE_DPS &&
        fabsf(corrected_y) < GYRO_BIAS_TRACK_MAX_RATE_DPS &&
        fabsf(corrected_z) < GYRO_BIAS_TRACK_MAX_RATE_DPS &&
        accel_norm > GYRO_BIAS_TRACK_MIN_ACCEL_G &&
        accel_norm < GYRO_BIAS_TRACK_MAX_ACCEL_G;

    if (!stationary) {
        stationary_time_s = 0.0f;
        return;
    }

    stationary_time_s += dt;
    if (stationary_time_s < GYRO_BIAS_TRACK_SETTLE_S) {
        return;
    }

    const float alpha = dt / (GYRO_BIAS_TRACK_TIME_CONSTANT_S + dt);
    gyro_bias_x += alpha * (imu->gyro_x_dps - gyro_bias_x);
    gyro_bias_y += alpha * (imu->gyro_y_dps - gyro_bias_y);
    gyro_bias_z += alpha * (imu->gyro_z_dps - gyro_bias_z);
}

void rate_controller_init(void)
{
    rate_controller_start_calibration();
}

static void update_calibration(const imu_sample_t *imu)
{
    if (calibration_samples == 0u) {
        calibration_reference_x = imu->gyro_x_dps;
        calibration_reference_y = imu->gyro_y_dps;
        calibration_reference_z = imu->gyro_z_dps;
    } else if (fabsf(imu->gyro_x_dps) >=
                   CALIBRATION_MAX_ABSOLUTE_RATE_DPS ||
               fabsf(imu->gyro_y_dps) >=
                   CALIBRATION_MAX_ABSOLUTE_RATE_DPS ||
               fabsf(imu->gyro_z_dps) >=
                   CALIBRATION_MAX_ABSOLUTE_RATE_DPS ||
               fabsf(imu->gyro_x_dps - calibration_reference_x) >=
                   CALIBRATION_MAX_VARIATION_DPS ||
               fabsf(imu->gyro_y_dps - calibration_reference_y) >=
                   CALIBRATION_MAX_VARIATION_DPS ||
               fabsf(imu->gyro_z_dps - calibration_reference_z) >=
                   CALIBRATION_MAX_VARIATION_DPS) {
        if (++calibration_outliers < CALIBRATION_MAX_CONSECUTIVE_OUTLIERS) {
            return;
        }
        calibration_sum_x = calibration_sum_y = calibration_sum_z = 0.0f;
        calibration_samples = 0u;
        calibration_outliers = 0u;
        return;
    }

    calibration_outliers = 0u;
    calibration_sum_x += imu->gyro_x_dps;
    calibration_sum_y += imu->gyro_y_dps;
    calibration_sum_z += imu->gyro_z_dps;
    ++calibration_samples;
    if (calibration_samples >= GYRO_CALIBRATION_SAMPLES) {
        gyro_bias_x = calibration_sum_x / (float)calibration_samples;
        gyro_bias_y = calibration_sum_y / (float)calibration_samples;
        gyro_bias_z = calibration_sum_z / (float)calibration_samples;
        calibrated = true;
        rate_controller_reset();
    }
}

bool rate_controller_update(const imu_sample_t *imu,
                            bool armed,
                            uint8_t throttle_percent,
                            int8_t roll_percent,
                            int8_t pitch_percent,
                            int8_t yaw_percent,
                            rate_controller_output_t *output)
{
    if (imu == NULL || output == NULL || !imu->valid) {
        return false;
    }
    if (imu->sample_time_us == previous_sample_time_us) {
        return armed && calibrated;
    }

    float dt = 0.001f;
    if (previous_sample_time_us != 0u) {
        dt = (float)(imu->sample_time_us - previous_sample_time_us) /
             1000000.0f;
        dt = clamp_float(dt, 0.00005f, 0.005f);
    }
    previous_sample_time_us = imu->sample_time_us;

    if (!calibrated) {
        memset(output, 0, sizeof(*output));
        if (!armed) {
            update_calibration(imu);
        }
        return false;
    }

    const flight_settings_t *settings = flight_settings_get();
    if (!armed) {
        track_stationary_gyro_bias(imu, dt);
    } else {
        stationary_time_s = 0.0f;
    }
    output->roll_rate_dps =
        pt1(&gyro_filter[0], imu->gyro_x_dps - gyro_bias_x,
            settings->gyro_lpf_hz, dt) * GYRO_ROLL_SIGN;
    output->pitch_rate_dps =
        pt1(&gyro_filter[1], imu->gyro_y_dps - gyro_bias_y,
            settings->gyro_lpf_hz, dt) * GYRO_PITCH_SIGN;
    output->yaw_rate_dps =
        pt1(&gyro_filter[2], imu->gyro_z_dps - gyro_bias_z,
            settings->gyro_lpf_hz, dt) * GYRO_YAW_SIGN;
    output->roll_setpoint_dps =
        rate_setpoint(roll_percent, settings->roll_rate_dps,
                      settings->rate_expo);
    /* Receiver pitch high is nose-down; body pitch positive is nose-up. */
    output->pitch_setpoint_dps =
        -rate_setpoint(pitch_percent, settings->pitch_rate_dps,
                       settings->rate_expo);
    output->yaw_setpoint_dps =
        rate_setpoint(yaw_percent, settings->yaw_rate_dps,
                      settings->rate_expo);

    if (!armed) {
        rate_controller_reset();
        memset(output->motor_percent, 0, sizeof(output->motor_percent));
        output->roll_pid_percent = 0.0f;
        output->pitch_pid_percent = 0.0f;
        output->yaw_pid_percent = 0.0f;
        output->mixer_saturated = false;
        return false;
    }

    const float throttle = clamp_float((float)throttle_percent, 0.0f, 100.0f);
    float tpa_factor = 1.0f;
    if (settings->tpa_attenuation > 0.0f &&
        throttle > settings->tpa_breakpoint_percent &&
        settings->tpa_breakpoint_percent < 100.0f) {
        tpa_factor = 1.0f - settings->tpa_attenuation *
            (throttle - settings->tpa_breakpoint_percent) /
            (100.0f - settings->tpa_breakpoint_percent);
    }

    /*
     * Airmode is latched for the rest of the armed session. Before its first
     * activation the craft may be handled or the sticks moved while the
     * motors have little authority, so the I term must remain empty.
     * Activate before evaluating the PID loops so the first airborne
     * correction always starts from a known zero integral.
     */
    if (!airmode_active &&
        throttle >= AIRMODE_ACTIVATION_THROTTLE_PERCENT) {
        airmode_active = true;
    }

    output->roll_pid_percent =
        pid_update(&roll_pid, &settings->roll,
                   output->roll_setpoint_dps, output->roll_rate_dps,
                   settings->roll_feedforward, dt,
                   PID_ROLL_PITCH_OUTPUT_LIMIT_PERCENT, tpa_factor,
                   settings->dterm_lpf_hz, airmode_active, false,
                   &output->p_term_percent[0],
                   &output->i_term_percent[0], &output->d_term_percent[0],
                   &output->ff_term_percent[0]);
    output->pitch_pid_percent =
        pid_update(&pitch_pid, &settings->pitch,
                   output->pitch_setpoint_dps, output->pitch_rate_dps,
                   settings->pitch_feedforward, dt,
                   PID_ROLL_PITCH_OUTPUT_LIMIT_PERCENT, tpa_factor,
                   settings->dterm_lpf_hz, airmode_active, false,
                   &output->p_term_percent[1],
                   &output->i_term_percent[1], &output->d_term_percent[1],
                   &output->ff_term_percent[1]);
    output->yaw_pid_percent =
        pid_update(&yaw_pid, &settings->yaw,
                   output->yaw_setpoint_dps, output->yaw_rate_dps,
                   settings->yaw_feedforward, dt,
                   PID_YAW_OUTPUT_LIMIT_PERCENT, tpa_factor,
                  settings->dterm_lpf_hz, airmode_active, true,
                  &output->p_term_percent[2], &output->i_term_percent[2],
                  &output->d_term_percent[2],
                  &output->ff_term_percent[2]);

    const float mixer_yaw =
        settings->motor_direction_reversed != 0u
            ? -output->yaw_pid_percent
            : output->yaw_pid_percent;
    const float pid_authority = airmode_active ? 1.0f :
        clamp_float(throttle / AIRMODE_ACTIVATION_THROTTLE_PERCENT,
                    0.0f, 1.0f);
    /* Quad X: M1 rear-right, M2 front-right, M3 rear-left, M4 front-left. */
    float correction[4] = {
        (-output->roll_pid_percent - output->pitch_pid_percent - mixer_yaw) *
            pid_authority,
        (-output->roll_pid_percent + output->pitch_pid_percent + mixer_yaw) *
            pid_authority,
        (output->roll_pid_percent - output->pitch_pid_percent + mixer_yaw) *
            pid_authority,
        (output->roll_pid_percent + output->pitch_pid_percent - mixer_yaw) *
            pid_authority,
    };
    float minimum = correction[0];
    float maximum = correction[0];
    for (uint8_t i = 1u; i < 4u; ++i) {
        minimum = fminf(minimum, correction[i]);
        maximum = fmaxf(maximum, correction[i]);
    }

    const float available = 100.0f - settings->motor_idle_percent;
    const float requested_base =
        settings->motor_idle_percent + throttle * available / 100.0f;
    float scale = 1.0f;
    float base = requested_base;
    if (airmode_active) {
        const float span = maximum - minimum;
        if (span > available) scale = available / span;
        minimum *= scale;
        maximum *= scale;
        base = clamp_float(requested_base,
                           settings->motor_idle_percent - minimum,
                           100.0f - maximum);
    } else {
        /*
         * Before takeoff the throttle command owns the collective output.
         * Keep its base fixed and reduce all axis corrections by the same
         * factor so none of them can pull a motor below idle or raise the
         * collective through airmode-style base shifting.
         */
        if (minimum < 0.0f) {
            scale = fminf(scale,
                          (requested_base - settings->motor_idle_percent) /
                              -minimum);
        }
        if (maximum > 0.0f) {
            scale = fminf(scale,
                          (100.0f - requested_base) / maximum);
        }
        scale = clamp_float(scale, 0.0f, 1.0f);
    }
    /* Scaled corrections have exhausted the authority available in the
     * current mixer mode and should stop I-term accumulation. */
    mixer_saturated = scale < 0.999f;
    output->mixer_saturated = mixer_saturated;
    for (uint8_t i = 0u; i < 4u; ++i) {
        output->motor_percent[i] =
            clamp_float(base + correction[i] * scale, 0.0f, 100.0f);
    }
    return true;
}

bool rate_controller_is_calibrated(void)
{
    return calibrated;
}

uint16_t rate_controller_get_calibration_samples(void)
{
    return calibration_samples;
}

void rate_controller_get_corrected_imu(const imu_sample_t *raw,
                                       imu_sample_t *corrected)
{
    *corrected = *raw;
    if (calibrated) {
        corrected->gyro_x_dps =
            (corrected->gyro_x_dps - gyro_bias_x) * GYRO_ROLL_SIGN;
        corrected->gyro_y_dps =
            (corrected->gyro_y_dps - gyro_bias_y) * GYRO_PITCH_SIGN;
        corrected->gyro_z_dps =
            (corrected->gyro_z_dps - gyro_bias_z) * GYRO_YAW_SIGN;
    }
}
