#ifndef FLIGHTCODEPI_IMU_H
#define FLIGHTCODEPI_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
    uint32_t sample_time_us;
    bool valid;
} imu_sample_t;

bool imu_init(uint32_t scheduler_rate_hz);
bool imu_update(bool gyro_only);
const imu_sample_t *imu_get_latest_sample(void);
bool imu_is_available(void);
const char *imu_get_name(void);
uint32_t imu_get_update_rate_hz(bool gyro_only,
                                uint32_t scheduler_rate_hz);
uint32_t imu_get_gyro_rate_hz(void);

#endif
