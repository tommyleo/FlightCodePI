#include "esc_controller.h"

#include <stdbool.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "dshot.pio.h"

#define DSHOT_CYCLES_PER_BIT 8u
#define ESC_CONTROLLER_MAX_COUNT 4u
#define RECEIVER_THROTTLE_MIN_US 1000u
#define RECEIVER_THROTTLE_MAX_US 2000u
#define DSHOT_STARTUP_TIME_US 500000u
#define DSHOT_STARTUP_FRAME_INTERVAL_MS 1u
#define DSHOT_INTERFRAME_GUARD_US 2u

static bool dshot_program_loaded;
static uint dshot_program_offset;
static unsigned int dshot_rate_kbps = 300u;
static esc_controller_t *registered_escs[ESC_CONTROLLER_MAX_COUNT];
static uint8_t registered_esc_count;
static uint32_t dshot_next_frame_us;

static float dshot_clock_divider(void)
{
    return (float)clock_get_hz(clk_sys) /
           (float)(dshot_rate_kbps * 1000u * DSHOT_CYCLES_PER_BIT);
}

static uint32_t dshot_frame_duration_us(void)
{
    const uint32_t frame_time_us =
        (16000u + dshot_rate_kbps - 1u) / dshot_rate_kbps;
    return frame_time_us + DSHOT_INTERFRAME_GUARD_US;
}

static bool dshot_frame_active(void)
{
    return (int32_t)(time_us_32() - dshot_next_frame_us) < 0;
}

static uint16_t dshot_sanitize_value(uint16_t value)
{
    if (value < DSHOT_THROTTLE_MIN) {
        return 0u;
    }
    return value > DSHOT_THROTTLE_MAX ? DSHOT_THROTTLE_MAX : value;
}

bool esc_controller_set_dshot_rate(unsigned int rate_kbps)
{
    if (rate_kbps != 150u && rate_kbps != 300u && rate_kbps != 600u) {
        return false;
    }

    dshot_rate_kbps = rate_kbps;
    while (registered_esc_count > 0u && dshot_frame_active()) {
        tight_loop_contents();
    }
    uint32_t pio0_sm_mask = 0u;
    uint32_t pio1_sm_mask = 0u;
    for (uint8_t i = 0u; i < registered_esc_count; ++i) {
        esc_controller_t *esc = registered_escs[i];
        const uint32_t sm_mask = 1u << esc->state_machine;
        if (esc->pio_index == 0u) {
            pio0_sm_mask |= sm_mask;
        } else {
            pio1_sm_mask |= sm_mask;
        }
    }

    pio_set_sm_mask_enabled(pio0, pio0_sm_mask, false);
    pio_set_sm_mask_enabled(pio1, pio1_sm_mask, false);
    for (uint8_t i = 0u; i < registered_esc_count; ++i) {
        esc_controller_t *esc = registered_escs[i];
        PIO pio = esc->pio_index == 0u ? pio0 : pio1;
        pio_sm_set_clkdiv(pio, esc->state_machine, dshot_clock_divider());
        pio_sm_clkdiv_restart(pio, esc->state_machine);
    }
    pio_enable_sm_mask_in_sync(pio0, pio0_sm_mask);
    pio_enable_sm_mask_in_sync(pio1, pio1_sm_mask);
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
    if (dshot_frame_active()) {
        return;
    }
    PIO pio = esc->pio_index == 0u ? pio0 : pio1;
    value = dshot_sanitize_value(value);
    const uint16_t packet = dshot_create_packet(value, false);
    pio_sm_put_blocking(pio, esc->state_machine, (uint32_t)packet << 16u);
    esc->last_command = value;
    dshot_next_frame_us = time_us_32() + dshot_frame_duration_us();
}

static void dshot_send_all(esc_controller_t escs[],
                           const uint16_t values[],
                           uint8_t count)
{
    if (count > ESC_CONTROLLER_MAX_COUNT) {
        count = ESC_CONTROLLER_MAX_COUNT;
    }
    if (count == 0u || dshot_frame_active()) {
        return;
    }

    uint32_t pio0_sm_mask = 0u;
    uint32_t pio1_sm_mask = 0u;
    uint32_t packets[ESC_CONTROLLER_MAX_COUNT];

    for (uint8_t i = 0u; i < count; ++i) {
        const uint16_t value = dshot_sanitize_value(values[i]);
        packets[i] = (uint32_t)dshot_create_packet(value, false) << 16u;
        const uint32_t sm_mask = 1u << escs[i].state_machine;
        if (escs[i].pio_index == 0u) {
            pio0_sm_mask |= sm_mask;
        } else {
            pio1_sm_mask |= sm_mask;
        }
    }

    // Ferma tutti i canali prima di accodare i pacchetti: nessun ESC puo'
    // iniziare un frame mentre gli altri sono ancora in preparazione.
    pio_set_sm_mask_enabled(pio0, pio0_sm_mask, false);
    pio_set_sm_mask_enabled(pio1, pio1_sm_mask, false);
    for (uint8_t i = 0u; i < count; ++i) {
        PIO pio = escs[i].pio_index == 0u ? pio0 : pio1;
        pio_sm_put(pio, escs[i].state_machine, packets[i]);
        escs[i].last_command = dshot_sanitize_value(values[i]);
    }

    // Ogni PIO abilita le proprie state machine e riallinea i divisori di
    // clock con una sola scrittura.
    pio_enable_sm_mask_in_sync(pio0, pio0_sm_mask);
    pio_enable_sm_mask_in_sync(pio1, pio1_sm_mask);
    dshot_next_frame_us = time_us_32() + dshot_frame_duration_us();
}

static uint16_t dshot_throttle_from_percent(float percent)
{
    if (percent <= 0.0f) {
        return 0u;
    }

    if (percent > 100.0f) {
        percent = 100.0f;
    }

    const uint32_t dshot_range = DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN;
    return (uint16_t)(
        DSHOT_THROTTLE_MIN + (percent * (float)dshot_range) / 100.0f);
}

void esc_controller_preinit(const unsigned int gpios[], uint8_t count)
{
    // Impone un livello noto prima delle inizializzazioni piu' lente. Quando
    // gli ESC ricevono alimentazione non vedono impulsi casuali sui segnali.
    for (uint8_t i = 0u; i < count; ++i) {
        gpio_init(gpios[i]);
        gpio_put(gpios[i], false);
        gpio_set_dir(gpios[i], GPIO_OUT);
        gpio_pull_down(gpios[i]);
    }
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
    gpio_pull_down(gpio);
    pio_sm_set_consecutive_pindirs(pio, sm, gpio, 1u, true);
    pio_sm_init(pio, sm, dshot_program_offset, &config);
    pio_sm_set_pins_with_mask(pio, sm, 0u, 1u << gpio);

    esc->gpio = gpio;
    esc->pio_index = 1u;
    esc->state_machine = sm;
    if (registered_esc_count < ESC_CONTROLLER_MAX_COUNT) {
        registered_escs[registered_esc_count++] = esc;
    }
    esc->last_command = 0u;
}

void esc_controller_startup_sequence(esc_controller_t escs[], uint8_t count)
{
    const uint32_t start_time_us = time_us_32();
    do {
        esc_controller_stop_all(escs, count);
        sleep_ms(DSHOT_STARTUP_FRAME_INTERVAL_MS);
    } while ((uint32_t)(time_us_32() - start_time_us) < DSHOT_STARTUP_TIME_US);
}

void esc_controller_stop(esc_controller_t *esc)
{
    dshot_send(esc, 0u);
}

void esc_controller_stop_all(esc_controller_t escs[], uint8_t count)
{
    const uint16_t stop_values[ESC_CONTROLLER_MAX_COUNT] = {0u};
    if (count > ESC_CONTROLLER_MAX_COUNT) {
        count = ESC_CONTROLLER_MAX_COUNT;
    }
    dshot_send_all(escs, stop_values, count);
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
    esc_controller_set_throttle(esc, dshot_throttle_from_percent(percent));
}

void esc_controller_set_throttle_percent_all(esc_controller_t escs[],
                                               const float percent[],
                                               uint8_t count)
{
    uint16_t throttle[ESC_CONTROLLER_MAX_COUNT];
    if (count > ESC_CONTROLLER_MAX_COUNT) {
        count = ESC_CONTROLLER_MAX_COUNT;
    }
    for (uint8_t i = 0u; i < count; ++i) {
        throttle[i] = dshot_throttle_from_percent(percent[i]);
    }
    dshot_send_all(escs, throttle, count);
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
