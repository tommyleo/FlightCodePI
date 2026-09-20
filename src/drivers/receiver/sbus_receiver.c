#include "sbus_receiver.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "crsf_rx.pio.h"
#include "sbus_rx.pio.h"
#include "flight_settings.h"

#define SBUS_BAUD              100000.0f
#define SBUS_FRAME_SIZE             25u
#define SBUS_START_BYTE            0x0fu
#define SBUS_FLAG_FRAME_LOST       0x04u
#define SBUS_FLAG_FAILSAFE         0x08u
#define SBUS_SIGNAL_TIMEOUT_US   100000u
#define SBUS_BYTE_GAP_US            500u
#define SBUS_FAILSAFE_CONFIRM_US  100000u
#define CRSF_BAUD                 420000.0f
#define CRSF_RC_CHANNELS_PACKED       0x16u
#define CRSF_MAX_FRAME_SIZE              64u

static PIO sbus_pio = pio0;
static unsigned int sbus_sm;
static unsigned int receiver_gpio;
static unsigned int sbus_offset;
static unsigned int crsf_offset;
static uint32_t receiver_protocol;
static uint8_t rx_buffer[CRSF_MAX_FRAME_SIZE];
static uint8_t rx_index;
static sbus_frame_t latest_frame;
static uint32_t latest_frame_us;
static uint32_t previous_byte_us;
static bool have_frame;
static sbus_diagnostics_t diagnostics;
static bool failsafe_pending;
static uint32_t failsafe_started_us;

static uint8_t crsf_crc8(const uint8_t *bytes, uint8_t length)
{
    uint8_t crc = 0u;
    while (length-- > 0u) {
        crc ^= *bytes++;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x80u) != 0u
                      ? (uint8_t)((crc << 1u) ^ 0xd5u)
                      : (uint8_t)(crc << 1u);
        }
    }
    return crc;
}

static uint16_t sbus_raw_to_us(uint16_t raw)
{
    // Mappatura FrSky standard: 172..1811 circa -> 988..2012 us.
    return (uint16_t)(880u + (((uint32_t)raw * 5u + 4u) / 8u));
}

static void decode_frame(void)
{
    const uint32_t now_us = time_us_32();
    const uint8_t flags = rx_buffer[23];
    const bool frame_failsafe =
        (flags & SBUS_FLAG_FAILSAFE) != 0u;

    if (!have_frame) {
        /*
         * Begin operational diagnostics with the first complete SBUS frame.
         * Bytes and serial errors observed while the receiver and PIO input
         * are starting up are acquisition noise, not link-quality failures.
         */
        diagnostics.serial_words = SBUS_FRAME_SIZE;
        diagnostics.parity_errors = 0u;
        diagnostics.stop_errors = 0u;
        diagnostics.start_bytes = 1u;
    }

    latest_frame.frame_lost =
        (flags & SBUS_FLAG_FRAME_LOST) != 0u;
    latest_frame_us = now_us;
    have_frame = true;

    if (frame_failsafe) {
        if (!failsafe_pending) {
            failsafe_pending = true;
            failsafe_started_us = now_us;
        }
        latest_frame.failsafe =
            (uint32_t)(now_us - failsafe_started_us) >=
            SBUS_FAILSAFE_CONFIRM_US;
        return;
    }

    failsafe_pending = false;
    latest_frame.failsafe = false;
    const uint8_t *data = &rx_buffer[1];
    uint32_t accumulator = 0u;
    uint8_t accumulator_bits = 0u;
    uint8_t data_index = 0u;

    for (uint8_t channel = 0u; channel < SBUS_CHANNEL_COUNT; ++channel) {
        while (accumulator_bits < 11u) {
            accumulator |= ((uint32_t)data[data_index++]) << accumulator_bits;
            accumulator_bits += 8u;
        }

        const uint16_t raw = (uint16_t)(accumulator & 0x07ffu);
        latest_frame.channel_raw[channel] = raw;
        latest_frame.channel_us[channel] = sbus_raw_to_us(raw);
        accumulator >>= 11u;
        accumulator_bits -= 11u;
    }

}

static void accept_byte(uint8_t byte)
{
    if (receiver_protocol == RECEIVER_PROTOCOL_CRSF) {
        if (rx_index == 0u && byte != 0xc8u && byte != 0xeeu) {
            return;
        }
        rx_buffer[rx_index++] = byte;
        if (rx_index == 2u &&
            (rx_buffer[1] < 2u ||
             rx_buffer[1] > CRSF_MAX_FRAME_SIZE - 2u)) {
            diagnostics.stop_errors++;
            rx_index = 0u;
        } else if (rx_index >= 2u &&
                   rx_index == (uint8_t)(rx_buffer[1] + 2u)) {
            const uint8_t length = rx_buffer[1];
            if (length == 24u && rx_buffer[2] == CRSF_RC_CHANNELS_PACKED &&
                crsf_crc8(&rx_buffer[2], (uint8_t)(length - 1u)) ==
                    rx_buffer[length + 1u]) {
                const uint8_t *payload = &rx_buffer[3];
                uint32_t accumulator = 0u;
                uint8_t bits = 0u;
                uint8_t offset = 0u;
                for (uint8_t channel = 0u;
                     channel < SBUS_CHANNEL_COUNT; ++channel) {
                    while (bits < 11u) {
                        accumulator |=
                            (uint32_t)payload[offset++] << bits;
                        bits += 8u;
                    }
                    uint16_t raw = (uint16_t)(accumulator & 0x07ffu);
                    if (raw < 172u) raw = 172u;
                    if (raw > 1811u) raw = 1811u;
                    latest_frame.channel_raw[channel] = raw;
                    latest_frame.channel_us[channel] = (uint16_t)(
                        988u + ((uint32_t)(raw - 172u) * 1024u + 819u) /
                                   1639u);
                    accumulator >>= 11u;
                    bits -= 11u;
                }
                latest_frame.frame_lost = false;
                latest_frame.failsafe = false;
                latest_frame_us = time_us_32();
                have_frame = true;
                diagnostics.valid_frames++;
            } else {
                diagnostics.stop_errors++;
            }
            rx_index = 0u;
        }
        return;
    }

    if (rx_index == 0u) {
        if (byte == SBUS_START_BYTE) {
            rx_buffer[rx_index++] = byte;
            diagnostics.start_bytes++;
        }
        return;
    }

    rx_buffer[rx_index++] = byte;

    if (rx_index == SBUS_FRAME_SIZE) {
        const uint8_t footer = rx_buffer[24];
        const bool valid_footer =
            (footer == 0x00u) || ((footer & 0x0fu) == 0x04u);

        if (valid_footer) {
            decode_frame();
            diagnostics.valid_frames++;
        }
        rx_index = 0u;
    }
}

static void accept_serial_word(uint32_t serial_word)
{
    diagnostics.serial_words++;
    const uint32_t now_us = time_us_32();

    if ((now_us - previous_byte_us) > SBUS_BYTE_GAP_US) {
        rx_index = 0u;
    }
    previous_byte_us = now_us;

    // The eight sampled data bits are aligned in bits 24..31.
    const uint8_t physical_byte =
        (uint8_t)(serial_word >> 24u);
    const uint8_t byte = receiver_protocol == RECEIVER_PROTOCOL_SBUS
                             ? (uint8_t)~physical_byte
                             : physical_byte;

    accept_byte(byte);
}

static void configure_receiver_pio(void)
{
    pio_sm_set_enabled(sbus_pio, sbus_sm, false);
    pio_sm_clear_fifos(sbus_pio, sbus_sm);
    pio_sm_restart(sbus_pio, sbus_sm);
    if (receiver_protocol == RECEIVER_PROTOCOL_CRSF) {
        crsf_rx_program_init(sbus_pio, sbus_sm, crsf_offset,
                             receiver_gpio, CRSF_BAUD);
    } else {
        sbus_rx_program_init(sbus_pio, sbus_sm, sbus_offset,
                             receiver_gpio, SBUS_BAUD);
    }
}

void sbus_receiver_init(unsigned int gpio, uint32_t protocol)
{
    memset(&latest_frame, 0, sizeof(latest_frame));
    memset(&diagnostics, 0, sizeof(diagnostics));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0u;
    latest_frame_us = 0u;
    previous_byte_us = 0u;
    have_frame = false;
    failsafe_pending = false;
    failsafe_started_us = 0u;

    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_disable_pulls(gpio);
    sleep_us(100u);
    diagnostics.idle_level = gpio_get(gpio);

    receiver_gpio = gpio;
    receiver_protocol = protocol == RECEIVER_PROTOCOL_CRSF
                            ? RECEIVER_PROTOCOL_CRSF
                            : RECEIVER_PROTOCOL_SBUS;
    sbus_offset = pio_add_program(sbus_pio, &sbus_rx_program);
    crsf_offset = pio_add_program(sbus_pio, &crsf_rx_program);
    sbus_sm = pio_claim_unused_sm(sbus_pio, true);
    configure_receiver_pio();
}

bool sbus_receiver_set_protocol(uint32_t protocol)
{
    if (protocol > RECEIVER_PROTOCOL_CRSF) {
        return false;
    }
    if (receiver_protocol == protocol) {
        return true;
    }
    receiver_protocol = protocol;
    memset(&latest_frame, 0, sizeof(latest_frame));
    memset(&diagnostics, 0, sizeof(diagnostics));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0u;
    latest_frame_us = 0u;
    previous_byte_us = 0u;
    have_frame = false;
    failsafe_pending = false;
    configure_receiver_pio();
    return true;
}

bool sbus_receiver_read(sbus_frame_t *frame)
{
    if (frame == NULL) {
        return false;
    }

    while (!pio_sm_is_rx_fifo_empty(sbus_pio, sbus_sm)) {
        const uint32_t serial_word = pio_sm_get(sbus_pio, sbus_sm);
        accept_serial_word(serial_word);
    }

    *frame = latest_frame;
    frame->frame_age_us = time_us_32() - latest_frame_us;
    frame->signal_valid =
        have_frame &&
        !frame->failsafe &&
        (frame->frame_age_us <= SBUS_SIGNAL_TIMEOUT_US);

    return frame->signal_valid;
}

void sbus_receiver_get_diagnostics(sbus_diagnostics_t *result)
{
    if (result != NULL) {
        *result = diagnostics;
    }
}
