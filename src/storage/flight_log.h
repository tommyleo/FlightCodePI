#ifndef FLIGHTCODEPI_FLIGHT_LOG_H
#define FLIGHTCODEPI_FLIGHT_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define FLIGHT_LOG_RATE_HZ 200u
#define FLIGHT_LOG_FLAG_MIXER_SATURATED 0x01u
#define FLIGHT_LOG_FLAG_STOP_DISARM 0x02u
#define FLIGHT_LOG_FLAG_STOP_RX_LOSS 0x04u
#define FLIGHT_LOG_FLAG_STOP_IMU 0x08u
#define FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE 0x10u
#define FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT 0x20u

typedef struct __attribute__((packed)) {
    int16_t gyro[3];
    int16_t setpoint[3];
    uint8_t motor[4];
    uint8_t throttle;
    uint8_t flags;
    uint16_t main_loop_us;
    uint16_t gyro_loop_us;
    uint16_t battery_centivolts;
    uint16_t cell_centivolts;
    uint8_t battery_cells;
    int8_t p_term[3];
    int8_t i_term[3];
    int8_t d_term[3];
    int8_t ff_term[3];
    uint8_t reserved;
} flight_log_record_t;

_Static_assert(sizeof(flight_log_record_t) == 40u,
               "flight log record format must remain 40 bytes");

#define FLIGHT_LOG_METADATA_VERSION 3u
typedef struct __attribute__((packed)) {
    uint32_t version, main_loop_hz, gyro_rate_hz, log_rate_hz;
    float pids[9], rates[4], feedforward[3], tpa[2], filters[2], alignment[3];
    float motor_idle_percent;
    uint32_t motor_protocol, motor_direction_reversed, receiver_protocol;
    uint16_t initial_battery_centivolts;
    uint8_t initial_battery_cells, reserved;
    float throttle_rise_ms; /* full-scale rise time in ms; v3+ */
} flight_log_metadata_t;

/* Version 2 ended before throttle_rise_ms. Never interpret trailing padding
 * or the first legacy sample as a saved ramp. Keep the original version. */
_Static_assert(offsetof(flight_log_metadata_t, throttle_rise_ms) == 128U,
               "legacy metadata prefix must remain 128 bytes");
_Static_assert(sizeof(flight_log_metadata_t) == 132U,
               "metadata v3 must remain 132 bytes");
static inline bool flight_log_metadata_decode(flight_log_metadata_t *out,
                                               const void *stored)
{
    uint32_t version;
    memcpy(&version, stored, sizeof(version));
    if (version != 2U && version != FLIGHT_LOG_METADATA_VERSION) return false;
    memset(out, 0, sizeof(*out));
    memcpy(out, stored, version == 2U
        ? offsetof(flight_log_metadata_t, throttle_rise_ms) : sizeof(*out));
    if (version == 2U) out->throttle_rise_ms = -1.0f;
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
void flight_log_record(const float gyro[3],
                       const float setpoint[3],
                       const float p_term[3],
                       const float i_term[3], const float d_term[3],
                       const float ff_term[3],
                       const float motors[4],
                       float throttle_percent,
                       bool mixer_saturated,
                       uint16_t main_loop_us, uint16_t gyro_loop_us);

#endif
