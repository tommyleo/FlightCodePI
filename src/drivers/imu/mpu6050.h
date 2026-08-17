#ifndef FLIGHTCODEPI_MPU6050_H
#define FLIGHTCODEPI_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "imu.h"

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    bool initialized;
} mpu6050_t;

bool mpu6050_init(mpu6050_t *device,
                  i2c_inst_t *i2c,
                  uint8_t address,
                  unsigned int sda_gpio,
                  unsigned int scl_gpio,
                  uint32_t baud_hz);
bool mpu6050_read(mpu6050_t *device, imu_sample_t *sample);
uint32_t mpu6050_get_gyro_rate_hz(void);

#endif
