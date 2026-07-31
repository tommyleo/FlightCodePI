#include "esc_controller.h"

#include <stdbool.h>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "dshot.pio.h"

#define DSHOT_CYCLES_PER_BIT 8u
#define ESC_CONTROLLER_MAX_COUNT 4u
#define RECEIVER_THROTTLE_MIN_US 1000u
#define RECEIVER_THROTTLE_MAX_US 2000u

static bool dshot_program_loaded;
static uint dshot_program_offset;
static unsigned int dshot_rate_kbps = 300u;
static esc_controller_t *registered_escs[ESC_CONTROLLER_MAX_COUNT];
static uint8_t registered_esc_count;

static float dshot_clock_divider(void)
{
    return (float)clock_get_hz(clk_sys) /
           (float)(dshot_rate_kbps * 1000u * DSHOT_CYCLES_PER_BIT);
}

bool esc_controller_set_dshot_rate(unsigned int rate_kbps)
{
    if (rate_kbps != 150u && rate_kbps != 300u && rate_kbps != 600u) {
        return false;
    }

    dshot_rate_kbps = rate_kbps;
    for (uint8_t i = 0u; i < registered_esc_count; ++i) {
        esc_controller_t *esc = registered_escs[i];
        PIO pio = esc->pio_index == 0u ? pio0 : pio1;
        pio_sm_set_clkdiv(pio, esc->state_machine, dshot_clock_divider());
        pio_sm_clkdiv_restart(pio, esc->state_machine);
    }
    return true;
}

unsigned int esc_controller_get_dshot_rate(void)
{
    return dshot_rate_kbps;
}

static uint16_t dshot_create_packet(uint16_t value, bool telemetry)
{
    const uint16_t payload = (uint16_t)((value << 1u) | (telemetry ? 1u : 0u));
    uint16_t checksum_data = payload;
    uint16_t checksum = 0u;

    for (uint8_t i = 0u; i < 3u; ++i) {
        checksum ^= checksum_data;
        checksum_data >>= 4u;
    }

    return (uint16_t)((payload << 4u) | (checksum & 0x0fu));
}

static void dshot_send(esc_controller_t *esc, uint16_t value)
{
    PIO pio = esc->pio_index == 0u ? pio0 : pio1;
    const uint16_t packet = dshot_create_packet(value, false);
    pio_sm_put_blocking(pio, esc->state_machine, (uint32_t)packet << 16u);
    esc->last_command = value;
}

void esc_controller_init(esc_controller_t *esc, unsigned int gpio)
{
    // PIO0 e' condiviso con SBUS. Tutti i quattro ESC usano le quattro
    // state machine di PIO1 e una sola copia del programma DShot.
    PIO pio = pio1;
    if (!dshot_program_loaded) {
        dshot_program_offset = pio_add_program(pio, &dshot_program);
        dshot_program_loaded = true;
    }

    const uint sm = pio_claim_unused_sm(pio, true);
    pio_sm_config config = dshot_program_get_default_config(dshot_program_offset);

    sm_config_set_sideset_pins(&config, gpio);
    sm_config_set_out_shift(&config, false, true, 16u);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    sm_config_set_clkdiv(&config, dshot_clock_divider());

    pio_gpio_init(pio, gpio);
    pio_sm_set_consecutive_pindirs(pio, sm, gpio, 1u, true);
    pio_sm_init(pio, sm, dshot_program_offset, &config);
    pio_sm_set_enabled(pio, sm, true);

    esc->gpio = gpio;
    esc->pio_index = 1u;
    esc->state_machine = sm;
    if (registered_esc_count < ESC_CONTROLLER_MAX_COUNT) {
        registered_escs[registered_esc_count++] = esc;
    }
    esc_controller_stop(esc);
}

void esc_controller_stop(esc_controller_t *esc)
{
    dshot_send(esc, 0u);
}

void esc_controller_set_throttle(esc_controller_t *esc, uint16_t throttle)
{
    if (throttle < DSHOT_THROTTLE_MIN) {
        esc_controller_stop(esc);
        return;
    }

    if (throttle > DSHOT_THROTTLE_MAX) {
        throttle = DSHOT_THROTTLE_MAX;
    }

    dshot_send(esc, throttle);
}

void esc_controller_set_throttle_percent(esc_controller_t *esc, uint8_t percent)
{
    esc_controller_set_throttle_percent_float(esc, (float)percent);
}

void esc_controller_set_throttle_percent_float(esc_controller_t *esc, float percent)
{
    if (percent <= 0.0f) {
        esc_controller_stop(esc);
        return;
    }

    if (percent > 100.0f) {
        percent = 100.0f;
    }

    const uint32_t dshot_range = DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN;
    const uint16_t throttle = (uint16_t)(
        DSHOT_THROTTLE_MIN + (percent * (float)dshot_range) / 100.0f);

    esc_controller_set_throttle(esc, throttle);
}

void esc_controller_set_throttle_us(esc_controller_t *esc, uint16_t throttle_us)
{
    if (throttle_us <= RECEIVER_THROTTLE_MIN_US) {
        esc_controller_stop(esc);
        return;
    }

    if (throttle_us > RECEIVER_THROTTLE_MAX_US) {
        throttle_us = RECEIVER_THROTTLE_MAX_US;
    }

    const uint32_t input_range = RECEIVER_THROTTLE_MAX_US - RECEIVER_THROTTLE_MIN_US;
    const uint8_t percent = (uint8_t)(
        ((uint32_t)(throttle_us - RECEIVER_THROTTLE_MIN_US) * 100u) / input_range);

    esc_controller_set_throttle_percent(esc, percent);
}

uint16_t esc_controller_get_last_command(const esc_controller_t *esc)
{
    return esc->last_command;
}
