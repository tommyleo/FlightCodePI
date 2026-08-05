#ifndef FLIGHTCODEPI_FLIGHT_SETTINGS_H
#define FLIGHTCODEPI_FLIGHT_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_axis_t;

typedef struct {
    pid_axis_t roll;
    pid_axis_t pitch;
    pid_axis_t yaw;
    uint32_t dshot_rate_kbps;
    float board_roll_deg;
    float board_pitch_deg;
    float board_yaw_deg;
    uint32_t motor_direction_reversed;
    float motor_idle_percent;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float rate_expo;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
    float tpa_attenuation;
    float tpa_breakpoint_percent;
    uint32_t receiver_channel_order;
    uint32_t arm_channel;
    uint32_t arm_min_us;
    uint32_t arm_max_us;
    uint32_t beep_channel;
    uint32_t beep_min_us;
    uint32_t beep_max_us;
    float gyro_lpf_hz;
    float dterm_lpf_hz;
} flight_settings_t;

#define RECEIVER_ORDER_TAER1234 0u
#define RECEIVER_ORDER_AETR1234 1u

void flight_settings_init(void);
const flight_settings_t *flight_settings_get(void);
bool flight_settings_set(const flight_settings_t *settings);
void flight_settings_reset_defaults(void);
void flight_settings_reset_tuning_defaults(flight_settings_t *settings);
bool flight_settings_save(void);
bool flight_settings_are_saved(void);

#endif
