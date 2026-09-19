#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

/* Bounded fixed-decimal telemetry writer. No printf float/double conversion
 * in the periodic control-loop path; control calculations are untouched. */
typedef struct {
    char *data;
    size_t capacity, length;
    bool valid;
} telemetry_text_t;

static inline void telemetry_char(telemetry_text_t *text, char value)
{
    if (text->length + 1U >= text->capacity) {
        text->valid = false;
        return;
    }
    text->data[text->length++] = value;
    text->data[text->length] = '\0';
}

static inline void telemetry_literal(telemetry_text_t *text, const char *value)
{
    while (*value) telemetry_char(text, *value++);
}

static inline void telemetry_digits(telemetry_text_t *text, uint32_t value)
{
    char digits[10];
    unsigned count = 0U;
    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) telemetry_char(text, digits[--count]);
}

static inline void telemetry_uint(telemetry_text_t *text, uint32_t value)
{
    telemetry_char(text, ' ');
    telemetry_digits(text, value);
}

static inline void telemetry_fixed(telemetry_text_t *text, float value,
                                    unsigned decimals)
{
    static const uint32_t scales[] = {1U, 10U, 100U, 1000U};
    if (decimals > 3U || !isfinite(value) || fabsf(value) > 1000000.0f) {
        text->valid = false;
        return;
    }
    telemetry_char(text, ' ');
    if (signbit(value)) telemetry_char(text, '-');
    const uint32_t scale = scales[decimals];
    const uint32_t scaled = (uint32_t)lroundf(fabsf(value) * (float)scale);
    telemetry_digits(text, scaled / scale);
    if (decimals != 0U) {
        telemetry_char(text, '.');
        uint32_t fraction = scaled % scale;
        for (uint32_t divisor = scale / 10U; divisor != 0U; divisor /= 10U) {
            telemetry_char(text, (char)('0' + fraction / divisor));
            fraction %= divisor;
        }
    }
}
