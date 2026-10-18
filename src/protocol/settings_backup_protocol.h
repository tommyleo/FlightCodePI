/* Complete settings snapshots. No flash layout or settings version changes. */
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#define SETTINGS_BACKUP_CHUNK 48U
#define SETTINGS_BACKUP_TIMEOUT_US 10000000U
static flight_settings_t backup_snapshot, restore_staging;
static bool backup_snapshot_ready, restore_active;
static size_t restore_received;
static uint32_t restore_crc, restore_activity_us;
static uint32_t settings_backup_crc(const void *data, size_t size)
{
    const uint8_t *bytes = data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}
static int backup_hex_digit(char value)
{
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}
static bool settings_backup_process(const char *command, bool unsafe)
{
    const bool reading = strncmp(command, "GET_SETTINGS_", 13) == 0;
    const bool restoring = strncmp(command, "RESTORE_SETTINGS_", 17) == 0;
    if (!reading && !restoring) return false;
    const uint32_t now = BACKUP_NOW_US();
    if (restore_active && (uint32_t)(now - restore_activity_us) > SETTINGS_BACKUP_TIMEOUT_US)
        restore_active = false;
    if (unsafe) {
        restore_active = false;
        BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_BUSY\n");
        return true;
    }
    if (strcmp(command, "GET_SETTINGS_INFO") == 0) {
        memcpy(&backup_snapshot, flight_settings_get(), sizeof backup_snapshot);
        backup_snapshot_ready = true;
        BACKUP_REPLY("@CFG SETTINGS_INFO %s %s %lu %lu %08lX %u\n",
                     BACKUP_FAMILY, BACKUP_BOARD,
                     (unsigned long)flight_settings_schema_version(),
                     (unsigned long)sizeof backup_snapshot,
                     (unsigned long)settings_backup_crc(&backup_snapshot, sizeof backup_snapshot),
                     flight_settings_are_saved() ? 1U : 0U);
        return true;
    }
    unsigned long offset, length, schema, size, crc;
    char extra;
    if (sscanf(command, "GET_SETTINGS_CHUNK %lu %lu %c", &offset, &length, &extra) == 2) {
        if (!backup_snapshot_ready || length == 0 || length > SETTINGS_BACKUP_CHUNK ||
            offset >= sizeof backup_snapshot || length > sizeof backup_snapshot - offset) {
            BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_RANGE\n"); return true;
        }
        char hex[SETTINGS_BACKUP_CHUNK * 2 + 1];
        const uint8_t *bytes = (const uint8_t *)&backup_snapshot;
        static const char digits[] = "0123456789ABCDEF";
        for (size_t i = 0; i < length; ++i) {
            hex[i * 2] = digits[bytes[offset + i] >> 4];
            hex[i * 2 + 1] = digits[bytes[offset + i] & 15U];
        }
        hex[length * 2] = '\0';
        BACKUP_REPLY("@CFG SETTINGS_CHUNK %lu %s\n", offset, hex);
        return true;
    }
    if (strcmp(command, "RESTORE_SETTINGS_CANCEL") == 0) {
        restore_active = false;
        BACKUP_REPLY("@CFG OK RESTORE_SETTINGS_CANCEL\n"); return true;
    }
    char family[16], board[40];
    if (sscanf(command, "RESTORE_SETTINGS_BEGIN %15s %39s %lu %lu %lx %c",
               family, board, &schema, &size, &crc, &extra) == 5) {
        restore_active = false;
        if (strcmp(family, BACKUP_FAMILY) || strcmp(board, BACKUP_BOARD) ||
            schema != flight_settings_schema_version() || size != sizeof restore_staging) {
            BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_INCOMPATIBLE\n"); return true;
        }
        memset(&restore_staging, 0, sizeof restore_staging);
        restore_received = 0;
        restore_crc = (uint32_t)crc;
        restore_activity_us = now;
        restore_active = true;
        BACKUP_REPLY("@CFG OK RESTORE_SETTINGS_BEGIN\n"); return true;
    }
    char hex[SETTINGS_BACKUP_CHUNK * 2 + 1];
    if (sscanf(command, "RESTORE_SETTINGS_CHUNK %lu %96s %c", &offset, hex, &extra) == 2) {
        const size_t count = strlen(hex);
        if (!restore_active || offset != restore_received || !count || count % 2 ||
            offset >= sizeof restore_staging || count / 2 > sizeof restore_staging - offset) {
            restore_active = false;
            BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_SEQUENCE\n"); return true;
        }
        uint8_t *bytes = (uint8_t *)&restore_staging;
        for (size_t i = 0; i < count; i += 2) {
            int hi = backup_hex_digit(hex[i]), lo = backup_hex_digit(hex[i + 1]);
            if (hi < 0 || lo < 0) {
                restore_active = false;
                BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_HEX\n"); return true;
            }
            bytes[offset + i / 2] = (uint8_t)((hi << 4) | lo);
        }
        restore_received += count / 2;
        restore_activity_us = now;
        BACKUP_REPLY("@CFG OK RESTORE_SETTINGS_CHUNK %lu\n", offset); return true;
    }
    if (strcmp(command, "RESTORE_SETTINGS_COMMIT") == 0) {
        const bool complete = restore_active && restore_received == sizeof restore_staging;
        restore_active = false;
        if (!complete || settings_backup_crc(&restore_staging, sizeof restore_staging) != restore_crc ||
            memchr(restore_staging.osd_pilot_name, '\0', sizeof restore_staging.osd_pilot_name) == NULL) {
            BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_INTEGRITY\n"); return true;
        }
        const flight_settings_t previous = *flight_settings_get();
        if (!flight_settings_set(&restore_staging)) {
            BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_VALUES\n"); return true;
        }
        if (!flight_settings_save()) {
            (void)flight_settings_set(&previous);
            BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_SAVE_FAILED\n"); return true;
        }
        BACKUP_REPLY("@CFG OK RESTORE_SETTINGS_COMMIT REBOOT_REQUIRED\n"); return true;
    }
    if (restoring) restore_active = false;
    BACKUP_REPLY("@CFG ERROR SETTINGS_BACKUP_COMMAND\n");
    return true;
}
