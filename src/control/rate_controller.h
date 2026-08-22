#ifndef FLIGHTCODEPI_RATE_CONTROLLER_H
#define FLIGHTCODEPI_RATE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

typedef struct {
    float motor_percent[4];
    float roll_setpoint_dps;
    float pitch_setpoint_dps;
    float yaw_setpoint_dps;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float roll_pid_percent;
    float pitch_pid_percent;
    float yaw_pid_percent;
    float p_term_percent[3];
    float i_term_percent[3];
    float d_term_percent[3];
    float ff_term_percent[3];
    bool mixer_saturated;
} rate_controller_output_t;

void rate_controller_init(void);
bool rate_controller_update(const imu_sample_t *imu,
                            bool armed,
                            uint8_t throttle_percent,
                            int8_t roll_percent,
                            int8_t pitch_percent,
                            int8_t yaw_percent,
                            rate_controller_output_t *output);
void rate_controller_reset(void);
void rate_controller_start_calibration(void);
bool rate_controller_is_calibrated(void);
uint16_t rate_controller_get_calibration_samples(void);
void rate_controller_get_corrected_imu(const imu_sample_t *raw,
                                       imu_sample_t *corrected);

#endif
