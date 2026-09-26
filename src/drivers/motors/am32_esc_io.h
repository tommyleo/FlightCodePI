#pragma once

#include <stdbool.h>
#include <stdint.h>

uint8_t am32_esc_count(void);
void am32_esc_begin(void);
void am32_esc_end(void);
void am32_esc_input(uint8_t index);
void am32_esc_output(uint8_t index);
bool am32_esc_read(uint8_t index);
void am32_esc_write(uint8_t index, bool high);
uint32_t am32_esc_micros(void);
uint32_t am32_esc_timing_now(void);
void am32_esc_wait_until(uint32_t started, uint32_t offset_us);
uint32_t am32_esc_critical_enter(void);
void am32_esc_critical_exit(uint32_t state);
