#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sbus_receiver.h"
#define RECEIVER_PROTOCOL_SBUS 0u
#define RECEIVER_PROTOCOL_CRSF 1u
#define CRSF_RC_CHANNELS_PACKED 0x16u
#define CRSF_MAX_FRAME_SIZE 64u
#define SBUS_START_BYTE 0x0fu
#define SBUS_FRAME_SIZE 25u
static uint32_t receiver_protocol = RECEIVER_PROTOCOL_CRSF;
static uint8_t rx_buffer[64], rx_index;
static sbus_frame_t latest_frame;
static uint32_t latest_frame_us;
static bool have_frame;
static sbus_diagnostics_t diagnostics;
static uint32_t time_us_32(void) { return 1234u; }
static void decode_frame(void) { assert(false); }
#include "crsf_crc_under_test.inc"
#include "crsf_accept_under_test.inc"
static void send_frame(uint8_t type, uint8_t length, bool corrupt)
{
    uint8_t packet[64] = {0xc8, length, type};
    packet[length + 1u] = crsf_crc8(&packet[2], length - 1u);
    if (corrupt) packet[length + 1u] ^= 1;
    for (unsigned i = 0; i < length + 2u; ++i) accept_byte(packet[i]);
    assert(rx_index == 0);
}
int main(void)
{
    latest_frame.failsafe = true;
    for (unsigned i = 0; i < 5; ++i) send_frame(0x14, 12, false);
    assert(diagnostics.stop_errors == 0 && diagnostics.valid_frames == 0);
    assert(!have_frame && latest_frame_us == 0 && latest_frame.failsafe);
    send_frame(0x14, 12, true);
    assert(diagnostics.stop_errors == 1);
    send_frame(0x16, 23, false);
    assert(diagnostics.stop_errors == 2 && !have_frame);
    diagnostics.parity_errors = 373;
    send_frame(0x16, 24, false);
    assert(diagnostics.valid_frames == 1 && have_frame);
    assert(latest_frame_us == 1234 && !latest_frame.failsafe);
    assert(latest_frame.channel_us[0] == 988);
    assert(diagnostics.stop_errors == 0 && diagnostics.parity_errors == 0);
    send_frame(0x14, 12, true);
    latest_frame.failsafe = true;
    send_frame(0x16, 24, false);
    assert(diagnostics.stop_errors == 1 && diagnostics.valid_frames == 2);
    assert(!latest_frame.failsafe);
    send_frame(0x29, 5, false);
    assert(diagnostics.stop_errors == 1 && diagnostics.valid_frames == 2);
    puts("CRSF PI diagnostics: PASS");
}
