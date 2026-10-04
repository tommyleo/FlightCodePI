#include "msp_displayport.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flight_settings.h"
#include "pico/time.h"
#include "hdzero_msp.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#define DISPLAYPORT_UART uart1
#define DISPLAYPORT_TX_GPIO 4u
#define DISPLAYPORT_RX_GPIO 5u
#define DISPLAYPORT_BAUD 115200u
#define MSP_DISPLAYPORT 182u
#define TX_BUFFER_SIZE 512u
#define SCREEN_COLUMNS 30u
#define HEARTBEAT_PERIOD_US 500000u

static hdzero_msp_t vtx_peer;
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint16_t tx_head, tx_tail;
static uint32_t last_heartbeat_us, flight_started_us, flight_duration_us;
static bool available, previous_armed, timer_started, release_sent;

static bool enqueue_packet(bool v2, char direction, uint16_t command, const uint8_t *payload, uint8_t length)
{
    const uint16_t used = (uint16_t)((tx_head - tx_tail) & (TX_BUFFER_SIZE - 1u));
    if (TX_BUFFER_SIZE - 1u - used < (uint16_t)length + (v2 ? 9u : 6u)) return false;
    uint8_t checksum = 0u;
    const uint8_t prefix[] = {'$', v2 ? 'X' : 'M', (uint8_t)direction};
    for (size_t i = 0; i < sizeof(prefix); ++i) {
        tx_buffer[tx_head] = prefix[i];
        tx_head = (uint16_t)((tx_head + 1u) & (TX_BUFFER_SIZE - 1u));
    }
    const uint8_t header[] = {0u, (uint8_t)command, (uint8_t)(command >> 8), length, 0u};
    const uint8_t legacy[] = {length, (uint8_t)command};
    const uint8_t *bytes=v2 ? header : legacy;
    for (unsigned i=0; i<(v2 ? sizeof(header) : sizeof(legacy)); ++i) {
        tx_buffer[tx_head]=bytes[i];
        tx_head=(uint16_t)((tx_head+1u)&(TX_BUFFER_SIZE-1u));
        checksum=v2 ? hdzero_crc(checksum,bytes[i]) : checksum ^ bytes[i];
    }
    for (uint8_t i = 0; i < length; ++i) {
        tx_buffer[tx_head] = payload[i];
        tx_head = (uint16_t)((tx_head + 1u) & (TX_BUFFER_SIZE - 1u));
        checksum=v2 ? hdzero_crc(checksum,payload[i]) : checksum ^ payload[i];
    }
    tx_buffer[tx_head] = checksum;
    tx_head = (uint16_t)((tx_head + 1u) & (TX_BUFFER_SIZE - 1u));
    return true;
}

static bool enqueue_frame(const uint8_t *payload, uint8_t length)
{ return enqueue_packet(false,'>',MSP_DISPLAYPORT,payload,length); }

static void enqueue_simple(uint8_t command)
{
    (void)enqueue_frame(&command, 1u);
}

static void enqueue_string(uint32_t position, const char *text)
{
    if (position >= 480u || !text || !text[0]) return;
    uint8_t payload[33];
    size_t length = strlen(text);
    const uint8_t column = (uint8_t)(position % SCREEN_COLUMNS);
    if (length > SCREEN_COLUMNS - column) length = SCREEN_COLUMNS - column;
    if (length > 29u) length = 29u;
    payload[0] = 3u;
    payload[1] = (uint8_t)(position / SCREEN_COLUMNS);
    payload[2] = column;
    payload[3] = 0u;
    memcpy(payload + 4u, text, length);
    (void)enqueue_frame(payload, (uint8_t)(length + 4u));
}

void msp_displayport_init(void)
{
    const flight_settings_t *settings = flight_settings_get();
    if (available) uart_deinit(DISPLAYPORT_UART);
    available = false;
    tx_head = tx_tail = 0u;
    if (settings->vtx_protocol != VTX_PROTOCOL_HDZERO_MSP ||
        settings->vtx_uart != 1u) return;
    uart_init(DISPLAYPORT_UART, DISPLAYPORT_BAUD);
    gpio_set_function(DISPLAYPORT_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(DISPLAYPORT_RX_GPIO, GPIO_FUNC_UART);
    uart_set_format(DISPLAYPORT_UART, 8u, 1u, UART_PARITY_NONE);
    uart_set_hw_flow(DISPLAYPORT_UART, false, false);
    memset(&vtx_peer,0,sizeof(vtx_peer));
    vtx_peer.started_us=time_us_32();
    hdzero_configure(&vtx_peer,settings->vtx_band,settings->vtx_channel,
                     settings->vtx_power_mw,false);
    available = true;
    previous_armed = timer_started = release_sent = false;
    last_heartbeat_us = 0u;
    const uint8_t options[] = {5u, 2u};
    (void)enqueue_frame(options, sizeof(options));
    enqueue_simple(0u);
}

void msp_displayport_process(bool armed)
{
    if (!msp_displayport_is_available()) return;
    const uint32_t now=time_us_32();
    const flight_settings_t *settings=flight_settings_get();
    hdzero_configure(&vtx_peer,settings->vtx_band,settings->vtx_channel,
                     settings->vtx_power_mw,armed);
    for (unsigned i=0; i<8u && uart_is_readable(DISPLAYPORT_UART); ++i)
        hdzero_receive(&vtx_peer,(uint8_t)uart_getc(DISPLAYPORT_UART),armed,now,enqueue_packet);
    hdzero_service(&vtx_peer,now,enqueue_packet);
    if (tx_tail == tx_head || !uart_is_writable(DISPLAYPORT_UART))
        return;
    uart_putc_raw(DISPLAYPORT_UART, tx_buffer[tx_tail]);
    tx_tail = (uint16_t)((tx_tail + 1u) & (TX_BUFFER_SIZE - 1u));
}

void msp_displayport_update(float voltage, bool armed, uint32_t now_us)
{
    if (!msp_displayport_is_available()) return;
    const flight_settings_t *settings = flight_settings_get();
    if ((uint32_t)(now_us - last_heartbeat_us) >= HEARTBEAT_PERIOD_US) {
        enqueue_simple(0u);
        last_heartbeat_us = now_us;
    }
    if (!settings->osd_enabled) {
        if (!release_sent) { enqueue_simple(1u); release_sent = true; }
        return;
    }
    release_sent = false;
    if (armed && !previous_armed) {
        flight_started_us = now_us;
        flight_duration_us = 0u;
        timer_started = true;
    } else if (armed && timer_started) {
        flight_duration_us = now_us - flight_started_us;
    }
    previous_armed = armed;
    char total_voltage[16] = "", cell_voltage[16] = "", timer[16], vtx[16];
    if (voltage >= 1.0f && voltage < 100.0f) {
        uint8_t cells = (uint8_t)ceilf(voltage / 4.25f);
        if (!cells) cells = 1u;
        (void)snprintf(total_voltage, sizeof(total_voltage), "BAT %.2fV", (double)voltage);
        (void)snprintf(cell_voltage, sizeof(cell_voltage), "CELL %.2fV", (double)(voltage / cells));
    }
    const uint32_t seconds = flight_duration_us / 1000000u;
    (void)snprintf(timer, sizeof(timer), "%02lu:%02lu",
                   (unsigned long)(seconds / 60u), (unsigned long)(seconds % 60u));
    static const char bands[] = "ABEFRL";
    (void)snprintf(vtx, sizeof(vtx), "%c:%lu:%lu", bands[settings->vtx_band],
                   (unsigned long)(settings->vtx_channel + 1u),
                   (unsigned long)settings->vtx_power_mw);
    const char *elements[OSD_ELEMENT_COUNT] = {
        total_voltage, cell_voltage, timer, "FLIGHTCODE", settings->osd_pilot_name,
    };
    enqueue_simple(2u);
    for (uint8_t i = 0u; i < OSD_ELEMENT_COUNT; ++i)
        if (settings->osd_element_enabled_mask & (1u << i))
            enqueue_string(settings->osd_element_positions[i], elements[i]);
    if (settings->vtx_osd_enabled)
        enqueue_string(settings->vtx_osd_position, vtx);
    enqueue_simple(4u);
}

bool msp_displayport_is_available(void)
{
    return available && flight_settings_get()->vtx_protocol == VTX_PROTOCOL_HDZERO_MSP &&
        flight_settings_get()->vtx_uart == 1u;
}

const char *msp_displayport_status_name(void)
{
    return available ? "MSP_DISPLAYPORT_READY" : "MSP_DISPLAYPORT_NOT_CONFIGURED";
}

const char *msp_displayport_vtx_status_name(void)
{ return msp_displayport_is_available() ? hdzero_status(&vtx_peer,time_us_32()) : "NOT_CONFIGURED"; }
