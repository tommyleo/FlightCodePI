#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef size_t (*am32_host_write_fn)(const uint8_t *data, size_t length);

void am32_passthrough_init(am32_host_write_fn writer);
bool am32_passthrough_consume(uint8_t byte, bool armed);
bool am32_passthrough_active(void);
