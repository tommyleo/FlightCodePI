#pragma once

#include <stdint.h>

#include "pico/stdlib.h"
#include "esc_controller.h"
#include "sbus_receiver.h"

#define MAIN_ESC_COUNT 4U

typedef struct {
    uint32_t phase;
    uint32_t rate_hz;
} loop_task_t;

typedef struct {
    uint32_t loop_hz;
    absolute_time_t next_loop;
    uint32_t timing_remainder;
    loop_task_t service_task;
    loop_task_t telemetry_task;
    loop_task_t esc_task;
    loop_task_t imu_task;
    uint32_t loop_measurement_start_us;
    uint32_t loop_measurement_count;
    float loop_frequency_hz;
    uint32_t maximum_loop_period_us;
    uint32_t previous_loop_start_us;
    esc_controller_t escs[MAIN_ESC_COUNT];
    sbus_frame_t receiver;
} main_loop_state_t;
