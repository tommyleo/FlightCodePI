#include "imu.h"

#include <math.h>

#include "flight_settings.h"
#include "flight_control_config.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "imu_config.h"
#include "mpu6050.h"
#include "mpu6500.h"
#include "pico/stdlib.h"

static imu_sample_t latest_sample;
static bool imu_available;
static bool gyro_only_active;
static uint32_t configured_scheduler_rate_hz;

static void apply_sensor_mounting(float *x, float *y, float *z)
{
    /*
     * The MPU module is mounted with its X/Y axes exchanged and its Z axis
     * reversed relative to the flight-controller frame:
     *
     *     FC X (roll)  = sensor Y
     *     FC Y (pitch) = sensor X
     *     FC Z (yaw)   = -sensor Z
     *
     * Keep this fixed mounting transform separate from the user-configurable
     * board alignment, whose zero setting must describe the standard build.
     */
    const float sensor_x = *x;
    *x = *y;
    *y = sensor_x;
    *z = -*z;
}

static float alignment[3][3];
static float alignment_angles[3];
static bool alignment_initialized;

static void update_alignment(void)
{
    const flight_settings_t *settings = flight_settings_get();
    const float roll = settings->board_roll_deg;
    const float pitch = settings->board_pitch_deg;
    const float yaw = settings->board_yaw_deg;
    if (alignment_initialized && alignment_angles[0] == roll &&
        alignment_angles[1] == pitch && alignment_angles[2] == yaw) return;
    const float to_rad = FLIGHT_PI_F / 180.0f;
    const float cr = cosf(roll * to_rad), sr = sinf(roll * to_rad);
    const float cp = cosf(pitch * to_rad), sp = sinf(pitch * to_rad);
    const float cy = cosf(yaw * to_rad), sy = sinf(yaw * to_rad);
    alignment[0][0] = cy * cp;
    alignment[0][1] = cy * sp * sr - sy * cr;
    alignment[0][2] = cy * sp * cr + sy * sr;
    alignment[1][0] = sy * cp;
    alignment[1][1] = sy * sp * sr + cy * cr;
    alignment[1][2] = sy * sp * cr - cy * sr;
    alignment[2][0] = -sp;
    alignment[2][1] = cp * sr;
    alignment[2][2] = cp * cr;
    alignment_angles[0] = roll;
    alignment_angles[1] = pitch;
    alignment_angles[2] = yaw;
    alignment_initialized = true;
}

static void rotate_vector(float *x, float *y, float *z)
{
    const float in_x = *x, in_y = *y, in_z = *z;
    *x = alignment[0][0] * in_x + alignment[0][1] * in_y + alignment[0][2] * in_z;
    *y = alignment[1][0] * in_x + alignment[1][1] * in_y + alignment[1][2] * in_z;
    *z = alignment[2][0] * in_x + alignment[2][1] * in_y + alignment[2][2] * in_z;
}

static void apply_board_alignment(imu_sample_t *sample)
{
    apply_sensor_mounting(&sample->accel_x_g,
                          &sample->accel_y_g,
                          &sample->accel_z_g);
    /* MPU acceleration is specific force; expose the gravity vector. */
    sample->accel_x_g = -sample->accel_x_g;
    sample->accel_y_g = -sample->accel_y_g;
    sample->accel_z_g = -sample->accel_z_g;
    apply_sensor_mounting(&sample->gyro_x_dps,
                          &sample->gyro_y_dps,
                          &sample->gyro_z_dps);
    rotate_vector(&sample->accel_x_g,
                  &sample->accel_y_g,
                  &sample->accel_z_g);
    rotate_vector(&sample->gyro_x_dps,
                  &sample->gyro_y_dps,
                  &sample->gyro_z_dps);
}

static void apply_gyro_alignment(imu_sample_t *sample)
{
    apply_sensor_mounting(&sample->gyro_x_dps,
                          &sample->gyro_y_dps,
                          &sample->gyro_z_dps);
    rotate_vector(&sample->gyro_x_dps,
                  &sample->gyro_y_dps,
                  &sample->gyro_z_dps);
}

#if IMU_BACKEND == IMU_BACKEND_MPU6050_I2C
static mpu6050_t imu_device;
#elif IMU_BACKEND == IMU_BACKEND_MPU6500_SPI
static mpu6500_t imu_device;
#else
#error "Backend IMU non supportato"
#endif

bool imu_init(uint32_t scheduler_rate_hz)
{
    latest_sample = (imu_sample_t){0};
    configured_scheduler_rate_hz = scheduler_rate_hz;

#if IMU_BACKEND == IMU_BACKEND_MPU6050_I2C
    i2c_inst_t *i2c = IMU_I2C_INDEX == 0u ? i2c0 : i2c1;
    imu_available = mpu6050_init(&imu_device,
                                i2c,
                                IMU_I2C_ADDRESS,
                                IMU_I2C_SDA_GPIO,
                                IMU_I2C_SCL_GPIO,
                                IMU_I2C_BAUD_HZ);
#elif IMU_BACKEND == IMU_BACKEND_MPU6500_SPI
    spi_inst_t *spi = IMU_SPI_INDEX == 0u ? spi0 : spi1;
    imu_available = mpu6500_init(&imu_device,
                                spi,
                                IMU_SPI_MISO_GPIO,
                                IMU_SPI_CS_GPIO,
                                IMU_SPI_SCK_GPIO,
                                IMU_SPI_MOSI_GPIO,
                                IMU_SPI_INIT_BAUD_HZ,
                                IMU_SPI_BAUD_HZ);
#endif

    gyro_only_active = false;
    return imu_available;
}

bool imu_update(bool gyro_only)
{
    if (!imu_available) {
        return false;
    }

#if IMU_BACKEND == IMU_BACKEND_MPU6050_I2C
    (void)gyro_only;
    const bool updated = mpu6050_read(&imu_device, &latest_sample);
#elif IMU_BACKEND == IMU_BACKEND_MPU6500_SPI
    if (gyro_only != gyro_only_active) {
        if (!mpu6500_set_gyro_only(&imu_device, gyro_only,
                                   configured_scheduler_rate_hz)) {
            latest_sample.valid = false;
            return false;
        }
        gyro_only_active = gyro_only;
    }
    const bool updated = gyro_only
        ? mpu6500_read_gyro(&imu_device, &latest_sample)
        : mpu6500_read(&imu_device, &latest_sample);
#endif
    if (updated) {
        update_alignment();
        if (gyro_only) {
            apply_gyro_alignment(&latest_sample);
        } else {
            apply_board_alignment(&latest_sample);
        }
    }
    return updated;
}

const imu_sample_t *imu_get_latest_sample(void)
{
    return &latest_sample;
}

bool imu_is_available(void)
{
    return imu_available;
}

const char *imu_get_name(void)
{
#if IMU_BACKEND == IMU_BACKEND_MPU6050_I2C
    return "MPU6050 I2C";
#elif IMU_BACKEND == IMU_BACKEND_MPU6500_SPI
    return mpu6500_get_name(&imu_device);
#else
    return "Unknown";
#endif
}

uint32_t imu_get_update_rate_hz(bool gyro_only,
                                uint32_t scheduler_rate_hz)
{
#if IMU_BACKEND == IMU_BACKEND_MPU6050_I2C
    (void)gyro_only;
    const uint32_t sensor_rate_hz = mpu6050_get_gyro_rate_hz();
#elif IMU_BACKEND == IMU_BACKEND_MPU6500_SPI
    const uint32_t sensor_rate_hz =
        mpu6500_get_gyro_rate_hz(gyro_only, scheduler_rate_hz);
#endif
    return sensor_rate_hz < scheduler_rate_hz
        ? sensor_rate_hz
        : scheduler_rate_hz;
}

uint32_t imu_get_gyro_rate_hz(void)
{
    return imu_get_update_rate_hz(true, configured_scheduler_rate_hz);
}
