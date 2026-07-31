#ifndef FLIGHTCODEPI_FLIGHT_LOG_H
#define FLIGHTCODEPI_FLIGHT_LOG_H

#include <stdbool.h>
#include <stdint.h>

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
    int8_t pid[3];
    uint8_t motor[4];
    uint8_t throttle;
    uint8_t flags;
    uint16_t loop_us;
    uint8_t reserved;
} flight_log_record_t;

void flight_log_init(void);
void flight_log_set_inhibited(bool inhibited);
void flight_log_start(void);
void flight_log_stop(uint8_t stop_flag);
bool flight_log_is_recording(void);
uint32_t flight_log_count(void);
bool flight_log_get(uint32_t index, flight_log_record_t *record);
void flight_log_persist_if_ready(void);
void flight_log_record(const float gyro[3],
                       const float setpoint[3],
                       const float pid[3],
                       const float motors[4],
                       float throttle_percent,
                       bool mixer_saturated,
                       uint16_t loop_us);

#endif
