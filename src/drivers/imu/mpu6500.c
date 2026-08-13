#include "mpu6500.h"

#include "pico/stdlib.h"

#define MPU6500_REG_SMPLRT_DIV 0x19u
#define MPU6500_REG_CONFIG 0x1au
#define MPU6500_REG_GYRO_CONFIG 0x1bu
#define MPU6500_REG_ACCEL_CONFIG 0x1cu
#define MPU6500_REG_ACCEL_CONFIG_2 0x1du
#define MPU6500_REG_ACCEL_XOUT_H 0x3bu
#define MPU6500_REG_GYRO_XOUT_H 0x43u
#define MPU6500_REG_USER_CTRL 0x6au
#define MPU6500_REG_PWR_MGMT_1 0x6bu
#define MPU6500_REG_PWR_MGMT_2 0x6cu
#define MPU6500_REG_WHO_AM_I 0x75u

#define MPU6500_SPI_READ 0x80u
#define MPU6500_USER_CTRL_I2C_IF_DIS 0x10u
#define MPU6500_ACCEL_SCALE_8G 4096.0f
#define MPU6500_GYRO_SCALE_2000DPS 16.4f

static void select_device(const mpu6500_t *device)
{
    gpio_put(device->cs_gpio, false);
}

static void deselect_device(const mpu6500_t *device)
{
    gpio_put(device->cs_gpio, true);
}

static bool write_register(mpu6500_t *device, uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {(uint8_t)(reg & ~MPU6500_SPI_READ), value};
    select_device(device);
    const int written = spi_write_blocking(device->spi, data, sizeof(data));
    deselect_device(device);
    sleep_us(2u);
    return written == (int)sizeof(data);
}

static bool read_registers(mpu6500_t *device,
                           uint8_t reg,
                           uint8_t *data,
                           size_t length)
{
    const uint8_t address = reg | MPU6500_SPI_READ;
    select_device(device);
    const int address_written =
        spi_write_blocking(device->spi, &address, 1u);
    const int bytes_read =
        address_written == 1
            ? spi_read_blocking(device->spi, 0u, data, length)
            : 0;
    deselect_device(device);
    return address_written == 1 && bytes_read == (int)length;
}

static int16_t read_be_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8u) | data[1]);
}

static bool supported_identity(uint8_t identity)
{
    return identity == 0x70u ||  // MPU6500
           identity == 0x71u ||  // MPU9250
           identity == 0x73u;    // MPU9255
}

bool mpu6500_init(mpu6500_t *device,
                  spi_inst_t *spi,
                  unsigned int miso_gpio,
                  unsigned int cs_gpio,
                  unsigned int sck_gpio,
                  unsigned int mosi_gpio,
                  uint32_t init_baud_hz,
                  uint32_t baud_hz)
{
    *device = (mpu6500_t){
        .spi = spi,
        .cs_gpio = cs_gpio,
    };

    gpio_init(cs_gpio);
    gpio_set_dir(cs_gpio, GPIO_OUT);
    gpio_put(cs_gpio, true);

    spi_init(spi, init_baud_hz);
    spi_set_format(spi, 8u, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(miso_gpio, GPIO_FUNC_SPI);
    gpio_set_function(sck_gpio, GPIO_FUNC_SPI);
    gpio_set_function(mosi_gpio, GPIO_FUNC_SPI);
    sleep_ms(10u);

    uint8_t identity = 0u;
    if (!read_registers(device, MPU6500_REG_WHO_AM_I, &identity, 1u) ||
        !supported_identity(identity)) {
        return false;
    }
    device->who_am_i = identity;

    if (!write_register(device, MPU6500_REG_PWR_MGMT_1, 0x80u)) {
        return false;
    }
    sleep_ms(100u);

    if (!write_register(device, MPU6500_REG_PWR_MGMT_1, 0x01u) ||
        !write_register(device, MPU6500_REG_PWR_MGMT_2, 0x00u) ||
        !write_register(device,
                        MPU6500_REG_USER_CTRL,
                        MPU6500_USER_CTRL_I2C_IF_DIS) ||
        !write_register(device, MPU6500_REG_SMPLRT_DIV, 0x00u) ||
        // DLPF_CFG=0: gyro a 8 kHz; scale +/-2000 dps e accel +/-8 g.
        !write_register(device, MPU6500_REG_CONFIG, 0x00u) ||
        !write_register(device, MPU6500_REG_GYRO_CONFIG, 0x18u) ||
        !write_register(device, MPU6500_REG_ACCEL_CONFIG, 0x10u) ||
        !write_register(device, MPU6500_REG_ACCEL_CONFIG_2, 0x03u)) {
        return false;
    }

    spi_set_baudrate(spi, baud_hz);
    device->initialized = true;
    return true;
}

bool mpu6500_read(mpu6500_t *device, imu_sample_t *sample)
{
    if (!device->initialized) {
        return false;
    }

    uint8_t data[14];
    if (!read_registers(device,
                        MPU6500_REG_ACCEL_XOUT_H,
                        data,
                        sizeof(data))) {
        sample->valid = false;
        return false;
    }

    sample->accel_x_g = (float)read_be_i16(&data[0]) / MPU6500_ACCEL_SCALE_8G;
    sample->accel_y_g = (float)read_be_i16(&data[2]) / MPU6500_ACCEL_SCALE_8G;
    sample->accel_z_g = (float)read_be_i16(&data[4]) / MPU6500_ACCEL_SCALE_8G;
    sample->temperature_c =
        ((float)read_be_i16(&data[6]) / 333.87f) + 21.0f;
    sample->gyro_x_dps =
        (float)read_be_i16(&data[8]) / MPU6500_GYRO_SCALE_2000DPS;
    sample->gyro_y_dps =
        (float)read_be_i16(&data[10]) / MPU6500_GYRO_SCALE_2000DPS;
    sample->gyro_z_dps =
        (float)read_be_i16(&data[12]) / MPU6500_GYRO_SCALE_2000DPS;
    sample->sample_time_us = time_us_32();
    sample->valid = true;
    return true;
}

bool mpu6500_set_gyro_only(mpu6500_t *device, bool enabled)
{
    if (!device->initialized) {
        return false;
    }
    /* FCHOICE_B=01 selects the 32 kHz gyro path; standby all accel axes. */
    if (!write_register(device, MPU6500_REG_GYRO_CONFIG,
                        enabled ? 0x19u : 0x18u)) {
        return false;
    }
    return write_register(device, MPU6500_REG_PWR_MGMT_2,
                          enabled ? 0x38u : 0x00u);
}

bool mpu6500_read_gyro(mpu6500_t *device, imu_sample_t *sample)
{
    uint8_t data[6];
    if (!device->initialized ||
        !read_registers(device, MPU6500_REG_GYRO_XOUT_H,
                        data, sizeof(data))) {
        sample->valid = false;
        return false;
    }
    sample->gyro_x_dps =
        (float)read_be_i16(&data[0]) / MPU6500_GYRO_SCALE_2000DPS;
    sample->gyro_y_dps =
        (float)read_be_i16(&data[2]) / MPU6500_GYRO_SCALE_2000DPS;
    sample->gyro_z_dps =
        (float)read_be_i16(&data[4]) / MPU6500_GYRO_SCALE_2000DPS;
    sample->sample_time_us = time_us_32();
    sample->valid = true;
    return true;
}

const char *mpu6500_get_name(const mpu6500_t *device)
{
    if (device->who_am_i == 0x71u) {
        return "MPU9250 SPI";
    }
    if (device->who_am_i == 0x73u) {
        return "MPU9255 SPI";
    }
    return "MPU6500 SPI";
}
