#pragma once

#include <stdint.h>

/* CRSF receiver bind command (ExpressLRS >= 3.4).
 * Extended destination EC, origin C8, receiver command 10, bind 01.
 * Command CRC uses polynomial BA; frame CRC uses D5. */
static const uint8_t crsf_bind_frame[] = {
    0xc8, 0x07, 0x32, 0xec, 0xc8, 0x10, 0x01, 0x9e, 0xe8
};
