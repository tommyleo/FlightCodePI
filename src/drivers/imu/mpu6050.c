#include "mpu6050.h"

#include "pico/stdlib.h"

#define MPU6050_REG_SMPLRT_DIV 0x19u
#define MPU6050_REG_CONFIG 0x1au
#define MPU6050_REG_GYRO_CONFIG 0x1bu
#define MPU6050_REG_ACCEL_CONFIG 0x1cu
#define MPU6050_REG_ACCEL_XOUT_H 0x3bu
#define MPU6050_REG_PWR_MGMT_1 0x6bu
#define MPU6050_REG_WHO_AM_I 0x75u

#define MPU6050_WHO_AM_I_VALUE 0x68u
#define MPU6050_ACCEL_SCALE_4G 8192.0f
#define MPU6050_GYRO_SCALE_500DPS 65.5f

static bool mpu6050_write_register(mpu6050_t *device,
                                   uint8_t reg,
                                   uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_write_blocking(device->i2c,
                              device->address,
                              data,
                              sizeof(data),
                              false) == (int)sizeof(data);
}

static bool mpu6050_read_registers(mpu6050_t *device,
                                   uint8_t reg,
                                   uint8_t *data,
                                   size_t length)
{
    if (i2c_write_blocking(device->i2c,
                           device->address,
                           &reg,
                           1u,
                           true) != 1) {
        return false;
    }

    return i2c_read_blocking(device->i2c,
                             device->address,
                             data,
                             length,
                             false) == (int)length;
}

static int16_t read_be_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8u) | data[1]);
}

bool mpu6050_init(mpu6050_t *device,
                  i2c_inst_t *i2c,
                  uint8_t address,
                  unsigned int sda_gpio,
                  unsigned int scl_gpio,
                  uint32_t baud_hz)
{
    device->i2c = i2c;
    device->address = address;
    device->initialized = false;

    i2c_init(i2c, baud_hz);
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);

    sleep_ms(10u);

    uint8_t who_am_i = 0u;
    if (!mpu6050_read_registers(device,
                                MPU6050_REG_WHO_AM_I,
                                &who_am_i,
                                1u) ||
        (who_am_i & 0x7eu) != MPU6050_WHO_AM_I_VALUE) {
        return false;
    }

    if (!mpu6050_write_register(device, MPU6050_REG_PWR_MGMT_1, 0x80u)) {
        return false;
    }
    sleep_ms(100u);

    // PLL asse X, 1 kHz, DLPF ~42 Hz, accelerometro +/-4 g, giroscopio +/-500 dps.
    if (!mpu6050_write_register(device, MPU6050_REG_PWR_MGMT_1, 0x01u) ||
        !mpu6050_write_register(device, MPU6050_REG_SMPLRT_DIV, 0x00u) ||
        !mpu6050_write_register(device, MPU6050_REG_CONFIG, 0x03u) ||
        !mpu6050_write_register(device, MPU6050_REG_GYRO_CONFIG, 0x08u) ||
        !mpu6050_write_register(device, MPU6050_REG_ACCEL_CONFIG, 0x08u)) {
        return false;
    }

    device->initialized = true;
    return true;
}

bool mpu6050_read(mpu6050_t *device, imu_sample_t *sample)
{
    if (!device->initialized) {
        return false;
    }

    uint8_t data[14];
    if (!mpu6050_read_registers(device,
                                MPU6050_REG_ACCEL_XOUT_H,
                                data,
                                sizeof(data))) {
        sample->valid = false;
        return false;
    }

    sample->accel_x_g = (float)read_be_i16(&data[0]) / MPU6050_ACCEL_SCALE_4G;
    sample->accel_y_g = (float)read_be_i16(&data[2]) / MPU6050_ACCEL_SCALE_4G;
    sample->accel_z_g = (float)read_be_i16(&data[4]) / MPU6050_ACCEL_SCALE_4G;
    sample->temperature_c = ((float)read_be_i16(&data[6]) / 340.0f) + 36.53f;
    sample->gyro_x_dps = (float)read_be_i16(&data[8]) / MPU6050_GYRO_SCALE_500DPS;
    sample->gyro_y_dps = (float)read_be_i16(&data[10]) / MPU6050_GYRO_SCALE_500DPS;
    sample->gyro_z_dps = (float)read_be_i16(&data[12]) / MPU6050_GYRO_SCALE_500DPS;
    sample->sample_time_us = time_us_32();
    sample->valid = true;
    return true;
}
uint32_t mpu6050_get_gyro_rate_hz(void)
{
    return 1000u;
}
