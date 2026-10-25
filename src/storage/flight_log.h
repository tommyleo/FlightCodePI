#ifndef FLIGHTCODEPI_FLIGHT_LOG_H
#define FLIGHTCODEPI_FLIGHT_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define FLIGHT_LOG_RATE_HZ 500u
#define FLIGHT_LOG_FORMAT_VERSION_MAJOR 2u
#define FLIGHT_LOG_FORMAT_VERSION_MINOR 0u
#define FLIGHT_LOG_FLAG_MIXER_SATURATED 0x01u
#define FLIGHT_LOG_FLAG_STOP_DISARM 0x02u
#define FLIGHT_LOG_FLAG_STOP_RX_LOSS 0x04u
#define FLIGHT_LOG_FLAG_STOP_IMU 0x08u
#define FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE 0x10u
#define FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT 0x20u
#define FLIGHT_LOG_FLAG_SIGNAL_CLIPPED 0x40U


typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;       /* real acquisition time, modulo 2^32 us */
    int16_t gyro[3];             /* filtered, 0.1 deg/s */
    int16_t gyro_raw[3];         /* aligned, bias removed, before gyro LPF */
    int16_t setpoint[3];         /* 0.1 deg/s */
    int16_t d_unfiltered[3];     /* 0.01 percent, same gain/TPA/boost as D */
    int16_t d_filtered[3];       /* 0.01 percent */
    uint8_t motor[4];           /* 0..255 = 0..100 percent */
    uint8_t throttle;           /* 0..200 = 0..100 percent */
    uint8_t flags;
    uint16_t main_loop_us, gyro_loop_us;
    uint16_t battery_centivolts, cell_centivolts;
    uint8_t battery_cells;
    int8_t p_term[3], i_term[3], ff_term[3], pid[3]; /* 0.5 percent */
    uint16_t overwritten_records; /* cumulative ring overwrites, saturating */
    uint8_t reserved;
} flight_log_record_t;

_Static_assert(sizeof(flight_log_record_t) == 64U,
               "internal log v2 record must remain 64 bytes");

#define FLIGHT_LOG_METADATA_VERSION 6u
typedef struct __attribute__((packed)) {
    uint32_t version, main_loop_hz, gyro_rate_hz, log_rate_hz;
    uint32_t pids[9];
    float rates[4];
    uint32_t feedforward[3];
    float tpa[2], filters[2], alignment[3];
    float motor_idle_percent;
    uint32_t motor_protocol, motor_direction_reversed, receiver_protocol;
    uint16_t initial_battery_centivolts;
    uint8_t initial_battery_cells, reserved;
} flight_log_metadata_t;

_Static_assert(sizeof(flight_log_metadata_t) == 128U,
               "metadata v6 must remain 128 bytes");
static inline bool flight_log_metadata_decode(flight_log_metadata_t *out,
                                               const void *stored)
{
    uint32_t version;
    memcpy(&version, stored, sizeof(version));
    if (version != FLIGHT_LOG_METADATA_VERSION) return false;
    memcpy(out, stored, sizeof(*out));
    return true;
}


void flight_log_init(void);
void flight_log_set_inhibited(bool inhibited);
void flight_log_set_battery_voltage(float voltage);
void flight_log_start(void);
void flight_log_stop(uint8_t stop_flag);
bool flight_log_is_recording(void);
uint32_t flight_log_count(void);
bool flight_log_get(uint32_t index, flight_log_record_t *record);
bool flight_log_get_metadata(flight_log_metadata_t *metadata);
void flight_log_persist_if_ready(void);
void flight_log_record(const float gyro_raw[3], const float gyro[3],
                       const float setpoint[3],
                       const float p_term[3],
                       const float i_term[3], const float d_raw[3], const float d_term[3],
                       const float ff_term[3],
                       const float motors[4],
                       float throttle_percent,
                       bool mixer_saturated,
                       uint16_t main_loop_us, uint16_t gyro_loop_us,
                       const float pid_output[3]);

#endif
