#ifndef FLIGHTCODEPI_MPU6500_H
#define FLIGHTCODEPI_MPU6500_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/spi.h"
#include "imu.h"

typedef struct {
    spi_inst_t *spi;
    unsigned int cs_gpio;
    uint8_t who_am_i;
    bool initialized;
} mpu6500_t;

bool mpu6500_init(mpu6500_t *device,
                  spi_inst_t *spi,
                  unsigned int miso_gpio,
                  unsigned int cs_gpio,
                  unsigned int sck_gpio,
                  unsigned int mosi_gpio,
                  uint32_t init_baud_hz,
                  uint32_t baud_hz);
bool mpu6500_read(mpu6500_t *device, imu_sample_t *sample);
const char *mpu6500_get_name(const mpu6500_t *device);

#endif
