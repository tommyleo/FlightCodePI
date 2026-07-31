#ifndef FLIGHTCODEPI_CONFIG_PROTOCOL_H
#define FLIGHTCODEPI_CONFIG_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"
#include "rate_controller.h"
#include "sbus_receiver.h"

void config_protocol_init(void);
void config_protocol_update(const sbus_frame_t *receiver, bool armed);
bool config_protocol_is_client_active(void);
bool config_protocol_motor_output_suppressed(void);
bool config_protocol_pid_simulation_enabled(void);
bool config_protocol_get_motor_test(const sbus_frame_t *receiver,
                                    uint8_t motor_percent[4]);
void config_protocol_send_telemetry(const sbus_frame_t *receiver,
                                    const imu_sample_t *imu,
                                    bool armed,
                                    float loop_frequency_hz,
                                    uint32_t max_loop_period_us,
                                    const rate_controller_output_t *control);

#endif
