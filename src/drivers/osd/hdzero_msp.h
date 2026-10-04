#pragma once

/* HDZero MSP peer, independent of the UART and the OSD renderer.
 * Wire layouts: hd-zero/hdzero-vtx src/msp_displayport.c and Betaflight
 * src/main/io/vtx_msp.c. Power is a one-based index, never milliwatts.
 * A successful send establishes MSP communication, not RF readback. */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef bool (*hdzero_msp_send_fn)(bool v2, char direction, uint16_t command,
                                  const uint8_t *payload, uint8_t length);
typedef struct {
    uint8_t state, position, checksum, header[5], payload[64];
    uint16_t command, length;
    bool v2, configured, settings_sent, linked;
    uint8_t config[15];
    uint32_t started_us, last_request_us, last_byte_us, last_push_us;
} hdzero_msp_t;

static uint8_t hdzero_crc(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (unsigned i = 0; i < 8; ++i)
        crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0xD5U : 0U));
    return crc;
}

static uint16_t hdzero_frequency(uint32_t band, uint32_t channel)
{
    static const uint16_t table[6][8] = {
        {0}, {0}, {5705,0,0,0,0,0,0,0},
        {5740,5760,0,5800,0,0,0,0},
        {5658,5695,5732,5769,5806,5843,5880,5917},
        {5362,5399,5436,5473,5510,5547,5584,5621},
    };
    return band < 6U && channel < 8U ? table[band][channel] : 0U;
}

static void hdzero_configure(hdzero_msp_t *peer, uint32_t band,
                             uint32_t channel, uint32_t power, bool armed)
{
    /* Freeze RF settings while armed, including replies to VTX polls. */
    if (armed && peer->configured) return;
    const uint16_t frequency = hdzero_frequency(band, channel);
    const uint8_t index = power == 25U ? 1U : power == 200U ? 2U : 0U;
    const uint8_t config[15] = {5U, (uint8_t)(band + 1U),
        (uint8_t)(channel + 1U), index, 0U,
        (uint8_t)frequency, (uint8_t)(frequency >> 8), 1U,
        0U, 0U, 0U, 0U, 6U, 8U, 2U};
    if (memcmp(peer->config, config, sizeof(config)) != 0)
        peer->settings_sent = false;
    memcpy(peer->config, config, sizeof(config));
    peer->configured = frequency != 0U && index != 0U;
}

static void hdzero_request(hdzero_msp_t *peer, bool armed, uint32_t now,
                           hdzero_msp_send_fn send)
{
    uint8_t reply[16] = {0};
    uint8_t size = 0;
    bool supported = peer->length == 0U;
    if (supported) switch (peer->command) {
    case 1U: reply[0]=0U; reply[1]=1U; reply[2]=46U; size=3U; break;
    /* HDZero gates RF control on this Betaflight compatibility signature. */
    case 2U: memcpy(reply, "BTFL", 4U); size=4U; break;
    case 3U: reply[0]=4U; reply[1]=4U; size=3U; break;
    case 88U:
        supported = peer->configured;
        memcpy(reply, peer->config, 15U); size=15U; break;
    case 101U: reply[6]=armed ? 1U : 0U; size=11U; break;
    case 105U:
        /* No stick-operated CMS is exposed by this link. Keep neutral sticks
         * and throttle low rather than opening VTX menus in flight. */
        for (unsigned i=0; i<8U; ++i) {
            const uint16_t value = i == 3U ? 1000U : 1500U;
            reply[2U*i]=(uint8_t)value; reply[2U*i+1U]=(uint8_t)(value >> 8);
        }
        size=16U; break;
    case 119U: reply[0]=0U; size=1U; break; /* ARM is status bit zero. */
    case 189U: reply[0]=30U; reply[1]=16U; size=2U; break;
    default: supported=false; break;
    }
    /* VTX table upload, remote EEPROM writes and SET_VTX_CONFIG are not
     * accepted: the configurator owns the desired settings and native table.
     * In particular, startup uploads must not replace the saved channel. */
    if (!supported) {
        (void)send(peer->v2, '!', peer->command, NULL, 0U);
        return;
    }
    peer->last_request_us = now;
    peer->linked = true;
    if (send(peer->v2, '>', peer->command, reply, size) && peer->command == 88U) {
        peer->settings_sent = true;
        peer->last_push_us = now;
    }
}

static void hdzero_receive(hdzero_msp_t *peer, uint8_t byte, bool armed,
                           uint32_t now, hdzero_msp_send_fn send)
{
    if ((uint32_t)(now - peer->last_byte_us) > 100000U) peer->state=0U;
    peer->last_byte_us=now;
    switch (peer->state) {
    case 0U: if (byte == '$') peer->state=1U; break;
    case 1U:
        peer->v2=byte == 'X';
        peer->state=(byte == 'M' || byte == 'X') ? 2U : byte == '$' ? 1U : 0U;
        break;
    case 2U: peer->state=byte == '<' ? 3U : 0U; peer->position=0U;
        peer->checksum=0U; break;
    case 3U:
        peer->header[peer->position++]=byte;
        peer->checksum=peer->v2 ? hdzero_crc(peer->checksum,byte) : peer->checksum ^ byte;
        if (peer->position == (peer->v2 ? 5U : 2U)) {
            peer->command=peer->v2 ? (uint16_t)(peer->header[1] | peer->header[2]<<8) : peer->header[1];
            peer->length=peer->v2 ? (uint16_t)(peer->header[3] | peer->header[4]<<8) : peer->header[0];
            peer->position=0U;
            peer->state=peer->length > sizeof(peer->payload) ? 6U : peer->length ? 4U : 5U;
        }
        break;
    case 4U:
        peer->payload[peer->position++]=byte;
        peer->checksum=peer->v2 ? hdzero_crc(peer->checksum,byte) : peer->checksum ^ byte;
        if (peer->position == peer->length) peer->state=5U;
        break;
    case 5U:
        peer->state=0U;
        if (byte == peer->checksum) hdzero_request(peer,armed,now,send);
        break;
    case 6U: /* Discard the entire oversized frame, including its checksum. */
        if (peer->length != 0U) --peer->length;
        else peer->state=0U;
        break;
    default: peer->state=0U; break;
    }
}

static void hdzero_service(hdzero_msp_t *peer, uint32_t now,
                           hdzero_msp_send_fn send)
{
    if (peer->configured && peer->linked &&
        (uint32_t)(now-peer->last_request_us) < 2000000U &&
        (uint32_t)(now-peer->last_push_us) >= 500000U) {
        if (send(false,'>',88U,peer->config,15U)) {
            peer->settings_sent=true;
            peer->last_push_us=now;
        }
    }
}

static const char *hdzero_status(const hdzero_msp_t *peer, uint32_t now)
{
    if (!peer->configured) return "INVALID_SETTINGS";
    if (peer->linked &&
        (uint32_t)(now-peer->last_request_us) < 2000000U)
        return peer->settings_sent ? "MSP_SETTINGS_SENT" : "MSP_CONNECTED";
    return peer->linked || (uint32_t)(now-peer->started_us) >= 5000000U
        ? "NO_RESPONSE" : "INITIALIZING";
}
