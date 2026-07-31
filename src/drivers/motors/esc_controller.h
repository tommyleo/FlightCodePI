#ifndef FLIGHTCODEPI_ESC_CONTROLLER_H
#define FLIGHTCODEPI_ESC_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#define DSHOT_THROTTLE_MIN 48u
#define DSHOT_THROTTLE_MAX 2047u

typedef struct {
    unsigned int gpio;
    unsigned int pio_index;
    unsigned int state_machine;
    uint16_t last_command;
} esc_controller_t;

bool esc_controller_set_dshot_rate(unsigned int rate_kbps);
unsigned int esc_controller_get_dshot_rate(void);
void esc_controller_init(esc_controller_t *esc, unsigned int gpio);
void esc_controller_stop(esc_controller_t *esc);
void esc_controller_set_throttle(esc_controller_t *esc, uint16_t throttle);
void esc_controller_set_throttle_percent(esc_controller_t *esc, uint8_t percent);
void esc_controller_set_throttle_percent_float(esc_controller_t *esc, float percent);
void esc_controller_set_throttle_us(esc_controller_t *esc, uint16_t throttle_us);
uint16_t esc_controller_get_last_command(const esc_controller_t *esc);

#endif
