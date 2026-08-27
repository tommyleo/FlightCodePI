#include "flight_settings.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define SETTINGS_MAGIC 0x46465049u
#define SETTINGS_VERSION 8u
#define SETTINGS_LEGACY_VERSION_7 7u
#define SETTINGS_LEGACY_VERSION_6 6u
#define SETTINGS_LEGACY_VERSION_5 5u
#define SETTINGS_LEGACY_VERSION 3u
#define SETTINGS_LEGACY_VERSION_4 4u
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, main_loop_hz)];
    uint32_t checksum;
} legacy_record_v6_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, vbat_multiplier)];
    uint32_t checksum;
} legacy_record_v7_t;

typedef struct {
    pid_axis_t roll;
    pid_axis_t pitch;
    pid_axis_t yaw;
    uint32_t dshot_rate_kbps;
} legacy_settings_v3_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v3_t settings;
    uint32_t checksum;
} legacy_record_v3_t;

typedef struct {
    pid_axis_t roll;
    pid_axis_t pitch;
    pid_axis_t yaw;
    uint32_t dshot_rate_kbps;
    float board_roll_deg;
    float board_pitch_deg;
    float board_yaw_deg;
    uint32_t motor_direction_reversed;
    float motor_idle_percent;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float rate_expo;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
    float tpa_attenuation;
    float tpa_breakpoint_percent;
} legacy_settings_v4_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v4_t settings;
    uint32_t checksum;
} legacy_record_v4_t;

typedef struct {
    pid_axis_t roll;
    pid_axis_t pitch;
    pid_axis_t yaw;
    uint32_t dshot_rate_kbps;
    float board_roll_deg;
    float board_pitch_deg;
    float board_yaw_deg;
    uint32_t motor_direction_reversed;
    float motor_idle_percent;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float rate_expo;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
    float tpa_attenuation;
    float tpa_breakpoint_percent;
    uint32_t receiver_channel_order;
    uint32_t arm_channel;
    uint32_t arm_min_us;
    uint32_t arm_max_us;
    uint32_t beep_channel;
    uint32_t beep_min_us;
    uint32_t beep_max_us;
} legacy_settings_v5_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v5_t settings;
    uint32_t checksum;
} legacy_record_v5_t;

_Static_assert(offsetof(flight_settings_t, gyro_lpf_hz) ==
                   sizeof(legacy_settings_v5_t),
               "Flight settings v5 migration layout changed");

typedef struct {
    uint32_t magic;
    uint32_t version;
    flight_settings_t settings;
    uint32_t checksum;
} settings_record_t;

static flight_settings_t current_settings;
static bool settings_saved;

static uint32_t hash_record(const void *record, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t hash = 2166136261u;
    for (size_t i = 0u; i < length; ++i) {
        hash = (hash ^ bytes[i]) * 16777619u;
    }
    return hash;
}

static bool finite_range(float value, float minimum, float maximum)
{
    return isfinite(value) && value >= minimum && value <= maximum;
}

static bool valid_pid(const pid_axis_t *pid)
{
    return finite_range(pid->kp, 0.0f, 1000.0f) &&
           finite_range(pid->ki, 0.0f, 1000.0f) &&
           finite_range(pid->kd, 0.0f, 1000.0f);
}

static bool valid_settings(const flight_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }
    const bool valid_dshot =
        settings->dshot_rate_kbps == 300u ||
        settings->dshot_rate_kbps == 600u ||
        settings->dshot_rate_kbps == 1200u;
    return valid_pid(&settings->roll) &&
           valid_pid(&settings->pitch) &&
           valid_pid(&settings->yaw) &&
           valid_dshot &&
           finite_range(settings->board_roll_deg, -180.0f, 180.0f) &&
           finite_range(settings->board_pitch_deg, -180.0f, 180.0f) &&
           finite_range(settings->board_yaw_deg, -180.0f, 180.0f) &&
           settings->motor_direction_reversed <= 1u &&
           finite_range(settings->motor_idle_percent, 1.0f, 10.0f) &&
           finite_range(settings->roll_rate_dps, 100.0f, 1200.0f) &&
           finite_range(settings->pitch_rate_dps, 100.0f, 1200.0f) &&
           finite_range(settings->yaw_rate_dps, 100.0f, 1200.0f) &&
           finite_range(settings->rate_expo, 0.0f, 0.9f) &&
           finite_range(settings->roll_feedforward, 0.0f, 1.0f) &&
           finite_range(settings->pitch_feedforward, 0.0f, 1.0f) &&
           finite_range(settings->yaw_feedforward, 0.0f, 1.0f) &&
           finite_range(settings->tpa_attenuation, 0.0f, 1.0f) &&
           finite_range(settings->tpa_breakpoint_percent, 0.0f, 100.0f) &&
           finite_range(settings->gyro_lpf_hz, 50.0f, 250.0f) &&
           finite_range(settings->dterm_lpf_hz, 20.0f, 200.0f) &&
           settings->dterm_lpf_hz <= settings->gyro_lpf_hz &&
           settings->receiver_channel_order <= RECEIVER_ORDER_AETR1234 &&
           settings->arm_channel >= 4u && settings->arm_channel < 16u &&
           settings->beep_channel >= 4u && settings->beep_channel < 16u &&
           settings->arm_min_us >= 900u && settings->arm_max_us <= 2100u &&
           settings->arm_min_us < settings->arm_max_us &&
           settings->beep_min_us >= 900u && settings->beep_max_us <= 2100u &&
           settings->beep_min_us < settings->beep_max_us &&
           finite_range(settings->vbat_multiplier, 0.5f, 1.5f);

}

void flight_settings_reset_tuning_defaults(flight_settings_t *settings)
{
    settings->roll = (pid_axis_t){0.10100f, 0.19000f, 0.00120f};
    settings->pitch = (pid_axis_t){0.09950f, 0.20000f, 0.00100f};
    settings->yaw = (pid_axis_t){0.15000f, 0.25000f, 0.00000f};
    settings->roll_rate_dps = 420.0f;
    settings->pitch_rate_dps = 420.0f;
    settings->yaw_rate_dps = 320.0f;
    settings->rate_expo = 0.30f;
    settings->roll_feedforward = 0.025f;
    settings->pitch_feedforward = 0.025f;
    settings->yaw_feedforward = 0.015f;
    settings->tpa_attenuation = 0.20f;
    settings->tpa_breakpoint_percent = 70.0f;
    settings->gyro_lpf_hz = 100.0f;
    settings->dterm_lpf_hz = 60.0f;
}

void flight_settings_reset_defaults(void)
{
    current_settings = (flight_settings_t){
        .dshot_rate_kbps = 300u,
        .motor_idle_percent = 5.0f,
        .receiver_channel_order = RECEIVER_ORDER_TAER1234,
        .arm_channel = 5u,
        .arm_min_us = 1950u,
        .arm_max_us = 2100u,
        .beep_channel = 4u,
        .beep_min_us = 1950u,
        .beep_max_us = 2100u,
        .main_loop_hz = 16000u,
        .vbat_multiplier = 1.0f,
    };
    flight_settings_reset_tuning_defaults(&current_settings);
    settings_saved = false;
}

void flight_settings_init(void)
{
    const uint8_t *flash =
        (const uint8_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
    const settings_record_t *stored = (const settings_record_t *)flash;
    if (stored->magic == SETTINGS_MAGIC &&
        stored->version == SETTINGS_VERSION &&
        stored->checksum ==
            hash_record(stored, offsetof(settings_record_t, checksum)) &&
        valid_settings(&stored->settings) &&
        (stored->settings.main_loop_hz == 8000u ||
         stored->settings.main_loop_hz == 16000u)) {
        current_settings = stored->settings;
        settings_saved = true;
        return;
    }

    const legacy_record_v7_t *legacy_v7 =
        (const legacy_record_v7_t *)flash;
    if (legacy_v7->magic == SETTINGS_MAGIC &&
        legacy_v7->version == SETTINGS_LEGACY_VERSION_7 &&
        legacy_v7->checksum == hash_record(
            legacy_v7, offsetof(legacy_record_v7_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v7->settings,
               sizeof(legacy_v7->settings));
        settings_saved = false;
        return;
    }

    const legacy_record_v6_t *legacy_v6 =
        (const legacy_record_v6_t *)flash;
    if (legacy_v6->magic == SETTINGS_MAGIC &&
        legacy_v6->version == SETTINGS_LEGACY_VERSION_6 &&
        legacy_v6->checksum == hash_record(
            legacy_v6, offsetof(legacy_record_v6_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v6->settings,
               sizeof(legacy_v6->settings));
        if (current_settings.dshot_rate_kbps != 300u &&
            current_settings.dshot_rate_kbps != 600u &&
            current_settings.dshot_rate_kbps != 1200u) {
            current_settings.dshot_rate_kbps = 300u;
        }
        settings_saved = false;
        return;
    }

    const legacy_record_v5_t *legacy_v5 =
        (const legacy_record_v5_t *)flash;
    if (legacy_v5->magic == SETTINGS_MAGIC &&
        legacy_v5->version == SETTINGS_LEGACY_VERSION_5 &&
        legacy_v5->checksum ==
            hash_record(legacy_v5, offsetof(legacy_record_v5_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v5->settings,
               sizeof(legacy_v5->settings));
        settings_saved = false;
        return;
    }

    const legacy_record_v4_t *legacy_v4 =
        (const legacy_record_v4_t *)flash;
    if (legacy_v4->magic == SETTINGS_MAGIC &&
        legacy_v4->version == SETTINGS_LEGACY_VERSION_4 &&
        legacy_v4->checksum ==
            hash_record(legacy_v4, offsetof(legacy_record_v4_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v4->settings,
               sizeof(legacy_v4->settings));
        settings_saved = false;
        return;
    }

    const legacy_record_v3_t *legacy = (const legacy_record_v3_t *)flash;
    if (legacy->magic == SETTINGS_MAGIC &&
        legacy->version == SETTINGS_LEGACY_VERSION &&
        legacy->checksum ==
            hash_record(legacy, offsetof(legacy_record_v3_t, checksum))) {
        flight_settings_reset_defaults();
        current_settings.roll = legacy->settings.roll;
        current_settings.pitch = legacy->settings.pitch;
        current_settings.yaw = legacy->settings.yaw;
        current_settings.dshot_rate_kbps = legacy->settings.dshot_rate_kbps;
        settings_saved = false;
        return;
    }

    flight_settings_reset_defaults();
}

const flight_settings_t *flight_settings_get(void)
{
    return &current_settings;
}

bool flight_settings_set(const flight_settings_t *settings)
{
    if (!valid_settings(settings) ||
        (settings->main_loop_hz != 8000u &&
         settings->main_loop_hz != 16000u)) {
        return false;
    }
    current_settings = *settings;
    settings_saved = false;
    return true;
}

bool flight_settings_save(void)
{
    if (!valid_settings(&current_settings)) {
        return false;
    }
    settings_record_t record = {
        .magic = SETTINGS_MAGIC,
        .version = SETTINGS_VERSION,
        .settings = current_settings,
    };
    record.checksum =
        hash_record(&record, offsetof(settings_record_t, checksum));

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &record, sizeof(record));

    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);
    settings_saved = true;
    return true;
}

bool flight_settings_are_saved(void)
{
    return settings_saved;
}
