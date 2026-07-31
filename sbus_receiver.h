#ifndef FLIGHTCODEPI_SBUS_RECEIVER_H
#define FLIGHTCODEPI_SBUS_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#define SBUS_CHANNEL_COUNT 16u

typedef struct {
    uint16_t channel_raw[SBUS_CHANNEL_COUNT];
    uint16_t channel_us[SBUS_CHANNEL_COUNT];
    bool signal_valid;
    bool frame_lost;
    bool failsafe;
    uint32_t frame_age_us;
} sbus_frame_t;

typedef struct {
    uint32_t serial_words;
    uint32_t parity_errors;
    uint32_t stop_errors;
    uint32_t start_bytes;
    uint32_t valid_frames;
    bool idle_level;
} sbus_diagnostics_t;

void sbus_receiver_init(unsigned int gpio);
bool sbus_receiver_read(sbus_frame_t *frame);
void sbus_receiver_get_diagnostics(sbus_diagnostics_t *diagnostics);

#endif
