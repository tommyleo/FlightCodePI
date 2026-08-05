#include "flight_log.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define FLIGHT_LOG_CAPACITY 4096u
#define CONTROL_LOOP_HZ 8000u
#define LOG_DECIMATION (CONTROL_LOOP_HZ / FLIGHT_LOG_RATE_HZ)
#define LOG_FLASH_MAGIC 0x50494c47u
#define LOG_FLASH_VERSION 3u
#define LOG_FLASH_SECTORS 25u
#define LOG_FLASH_SIZE (LOG_FLASH_SECTORS * FLASH_SECTOR_SIZE)
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define LOG_FLASH_OFFSET (SETTINGS_FLASH_OFFSET - LOG_FLASH_SIZE)
#define LOG_PERSIST_DELAY_US 200000u
#define LOG_MIN_FLIGHT_THROTTLE_PERCENT 10.0f

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    uint32_t rate_hz;
    uint32_t record_size;
    uint32_t checksum;
} log_header_t;

static flight_log_record_t records[FLIGHT_LOG_CAPACITY];
static uint32_t write_index;
static uint32_t record_count;
static uint16_t decimation_count;
static bool recording;
static bool inhibited;
static bool using_flash;
static bool persist_pending;
static uint32_t persist_requested_us;
static bool flight_qualified;
static uint32_t preserved_flash_count;
static uint16_t battery_centivolts;
static uint16_t cell_centivolts;
static uint8_t battery_cells;

static uint32_t hash_bytes(uint32_t hash, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0u; i < length; ++i) {
        hash = (hash ^ bytes[i]) * 16777619u;
    }
    return hash;
}

static const log_header_t *flash_header(void)
{
    return (const log_header_t *)(XIP_BASE + LOG_FLASH_OFFSET);
}

static const flight_log_record_t *flash_records(void)
{
    return (const flight_log_record_t *)
        (XIP_BASE + LOG_FLASH_OFFSET + sizeof(log_header_t));
}

static const flight_log_record_t *ram_record(uint32_t index)
{
    const uint32_t oldest =
        record_count == FLIGHT_LOG_CAPACITY ? write_index : 0u;
    return &records[(oldest + index) % FLIGHT_LOG_CAPACITY];
}

static int16_t scaled_i16(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t)lroundf(scaled);
}

static int8_t scaled_pid(float value)
{
    const float scaled = value * 2.0f;
    if (scaled > 127.0f) return 127;
    if (scaled < -128.0f) return -128;
    return (int8_t)lroundf(scaled);
}

void flight_log_init(void)
{
    write_index = 0u;
    record_count = 0u;
    decimation_count = 0u;
    recording = false;
    inhibited = false;
    using_flash = false;
    persist_pending = false;
    flight_qualified = false;
    preserved_flash_count = 0u;
    battery_centivolts = 0u;
    cell_centivolts = 0u;
    battery_cells = 0u;

    const log_header_t *header = flash_header();
    const size_t stored_size =
        sizeof(log_header_t) +
        (size_t)header->count * sizeof(flight_log_record_t);
    if (header->magic == LOG_FLASH_MAGIC &&
        header->version == LOG_FLASH_VERSION &&
        header->count > 0u &&
        header->count <= FLIGHT_LOG_CAPACITY &&
        header->rate_hz == FLIGHT_LOG_RATE_HZ &&
        header->record_size == sizeof(flight_log_record_t) &&
        stored_size <= LOG_FLASH_SIZE &&
        hash_bytes(2166136261u, flash_records(),
                   header->count * sizeof(flight_log_record_t)) ==
            header->checksum) {
        record_count = header->count;
        using_flash = true;
    }
}

void flight_log_set_inhibited(bool value)
{
    inhibited = value;
    if (value) recording = false;
}

void flight_log_set_battery_voltage(float voltage)
{
    if (voltage < 1.0f || voltage >= 100.0f) {
        battery_centivolts = 0u;
        cell_centivolts = 0u;
        battery_cells = 0u;
        return;
    }
    uint8_t candidate_cells = (uint8_t)ceilf(voltage / 4.25f);
    if (candidate_cells > 8u) candidate_cells = 8u;
    if (candidate_cells > battery_cells) battery_cells = candidate_cells;
    battery_centivolts = (uint16_t)lroundf(voltage * 100.0f);
    cell_centivolts = battery_cells > 0u
        ? (uint16_t)lroundf(voltage * 100.0f / (float)battery_cells)
        : 0u;
}

void flight_log_start(void)
{
    if (inhibited) return;
    preserved_flash_count = using_flash ? record_count : 0u;
    using_flash = false;
    persist_pending = false;
    write_index = 0u;
    record_count = 0u;
    decimation_count = 0u;
    flight_qualified = false;
    recording = true;
}

void flight_log_stop(uint8_t stop_flag)
{
    if (recording && !flight_qualified) {
        recording = false;
        persist_pending = false;
        write_index = 0u;
        record_count = preserved_flash_count;
        using_flash = preserved_flash_count > 0u;
        return;
    }
    if (recording && record_count > 0u) {
        flight_log_record_t *marker = &records[write_index];
        *marker = *ram_record(record_count - 1u);
        memset(marker->motor, 0, sizeof(marker->motor));
        marker->flags =
            (marker->flags & FLIGHT_LOG_FLAG_MIXER_SATURATED) | stop_flag;
        write_index = (write_index + 1u) % FLIGHT_LOG_CAPACITY;
        if (record_count < FLIGHT_LOG_CAPACITY) ++record_count;
        persist_pending = true;
        persist_requested_us = time_us_32();
    }
    recording = false;
}

bool flight_log_is_recording(void) { return recording; }
uint32_t flight_log_count(void) { return record_count; }

bool flight_log_get(uint32_t index, flight_log_record_t *record)
{
    if (record == NULL || recording || index >= record_count) {
        return false;
    }
    *record = using_flash ? flash_records()[index] : *ram_record(index);
    return true;
}

void flight_log_persist_if_ready(void)
{
    if (!persist_pending || recording ||
        time_us_32() - persist_requested_us < LOG_PERSIST_DELAY_US) {
        return;
    }

    log_header_t header = {
        .magic = LOG_FLASH_MAGIC,
        .version = LOG_FLASH_VERSION,
        .count = record_count,
        .rate_hz = FLIGHT_LOG_RATE_HZ,
        .record_size = sizeof(flight_log_record_t),
        .checksum = 2166136261u,
    };
    for (uint32_t i = 0u; i < record_count; ++i) {
        header.checksum =
            hash_bytes(header.checksum, ram_record(i), sizeof(*ram_record(i)));
    }

    static uint8_t page[FLASH_PAGE_SIZE];
    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(LOG_FLASH_OFFSET, LOG_FLASH_SIZE);

    size_t flash_position = 0u;
    size_t source_position = 0u;
    const size_t total_size =
        sizeof(header) + record_count * sizeof(flight_log_record_t);
    while (flash_position < total_size) {
        memset(page, 0xff, sizeof(page));
        size_t used = 0u;
        while (used < sizeof(page) && source_position < total_size) {
            if (source_position < sizeof(header)) {
                page[used++] =
                    ((const uint8_t *)&header)[source_position++];
            } else {
                const size_t record_data_position =
                    source_position - sizeof(header);
                const uint32_t record_index =
                    (uint32_t)(record_data_position /
                               sizeof(flight_log_record_t));
                const size_t byte_index =
                    record_data_position % sizeof(flight_log_record_t);
                page[used++] =
                    ((const uint8_t *)ram_record(record_index))[byte_index];
                ++source_position;
            }
        }
        flash_range_program(LOG_FLASH_OFFSET + (uint32_t)flash_position,
                            page,
                            FLASH_PAGE_SIZE);
        flash_position += FLASH_PAGE_SIZE;
    }
    restore_interrupts(interrupts);
    persist_pending = false;
    using_flash = true;
}

void flight_log_record(const float gyro[3],
                       const float setpoint[3],
                       const float pid[3], const float p_term[3],
                       const float i_term[3], const float d_term[3],
                       const float motors[4],
                       float throttle_percent,
                       bool saturated,
                       uint16_t loop_us)
{
    if (!recording || inhibited) return;
    if (throttle_percent > LOG_MIN_FLIGHT_THROTTLE_PERCENT) {
        flight_qualified = true;
    }
    if (++decimation_count < LOG_DECIMATION) return;
    decimation_count = 0u;

    flight_log_record_t *item = &records[write_index];
    for (uint8_t i = 0u; i < 3u; ++i) {
        item->gyro[i] = scaled_i16(gyro[i], 10.0f);
        item->setpoint[i] = scaled_i16(setpoint[i], 10.0f);
        item->pid[i] = scaled_pid(pid[i]);
        item->p_term[i] = scaled_pid(p_term[i]);
        item->i_term[i] = scaled_pid(i_term[i]);
        item->d_term[i] = scaled_pid(d_term[i]);
    }
    for (uint8_t i = 0u; i < 4u; ++i) {
        const float percent =
            motors[i] < 0.0f ? 0.0f : (motors[i] > 100.0f ? 100.0f : motors[i]);
        item->motor[i] = (uint8_t)lroundf(percent * 2.55f);
    }
    throttle_percent =
        throttle_percent < 0.0f ? 0.0f :
        (throttle_percent > 100.0f ? 100.0f : throttle_percent);
    item->throttle = (uint8_t)lroundf(throttle_percent * 2.0f);
    item->flags = saturated ? FLIGHT_LOG_FLAG_MIXER_SATURATED : 0u;
    item->loop_us = loop_us;
    item->battery_centivolts = battery_centivolts;
    item->cell_centivolts = cell_centivolts;
    item->battery_cells = battery_cells;
    memset(item->reserved, 0, sizeof(item->reserved));

    write_index = (write_index + 1u) % FLIGHT_LOG_CAPACITY;
    if (record_count < FLIGHT_LOG_CAPACITY) ++record_count;
}
