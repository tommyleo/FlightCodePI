#include "config_protocol.h"

#include <stdio.h>
#include <string.h>

#include "battery_voltage.h"
#include "esc_controller.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#define CONFIG_LINE_LENGTH 192u
#define CONFIG_CLIENT_TIMEOUT_US 3000000u
#define MOTOR_TEST_TIMEOUT_US 1000000u
#define DFU_DELAY_US 250000u

static char input_line[CONFIG_LINE_LENGTH];
static size_t input_length;
static bool client_active;
static uint32_t last_client_activity_us;
static bool motor_test_enabled;
static bool pid_simulation_enabled;
static uint8_t motor_test_percent[4];
static uint32_t last_motor_test_us;
static bool dfu_pending;
static uint32_t dfu_at_us;
static bool reboot_pending;
static uint32_t reboot_at_us;

static bool arm_mode_active(const sbus_frame_t *receiver)
{
    const flight_settings_t *settings = flight_settings_get();
    return receiver->signal_valid && settings->arm_channel < SBUS_CHANNEL_COUNT &&
           receiver->channel_us[settings->arm_channel] >= settings->arm_min_us &&
           receiver->channel_us[settings->arm_channel] <= settings->arm_max_us;
}

static void send_pids(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG PIDS %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %u\n",
           settings->roll.kp, settings->roll.ki, settings->roll.kd,
           settings->pitch.kp, settings->pitch.ki, settings->pitch.kd,
           settings->yaw.kp, settings->yaw.ki, settings->yaw.kd,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_motor_protocol(void)
{
    printf("@CFG MOTOR_PROTOCOL DSHOT%u %u\n",
           flight_settings_get()->dshot_rate_kbps,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_main_loop(void)
{
    printf("@CFG MAIN_LOOP %lu %u\n",
           (unsigned long)flight_settings_get()->main_loop_hz,
           flight_settings_are_saved() ? 1u : 0u);
    printf("@CFG GYRO_RATE %lu\n",
           (unsigned long)imu_get_gyro_rate_hz());
}

static void send_vbat_multiplier(void)
{
    printf("@CFG VBAT_MULTIPLIER %.3f %u\n",
           flight_settings_get()->vbat_multiplier,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_board_alignment(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG BOARD_ALIGNMENT %.2f %.2f %.2f %u\n",
           settings->board_roll_deg,
           settings->board_pitch_deg,
           settings->board_yaw_deg,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_motor_direction(void)
{
    printf("@CFG MOTOR_DIRECTION %s %u\n",
           flight_settings_get()->motor_direction_reversed != 0u
               ? "REVERSED" : "NORMAL",
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_motor_idle(void)
{
    printf("@CFG MOTOR_IDLE %.2f %u\n",
           flight_settings_get()->motor_idle_percent,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_rates(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG RATES %.1f %.1f %.1f %.3f %u\n",
           settings->roll_rate_dps,
           settings->pitch_rate_dps,
           settings->yaw_rate_dps,
           settings->rate_expo,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_feedforward(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG FEEDFORWARD %.6f %.6f %.6f %u\n",
           settings->roll_feedforward,
           settings->pitch_feedforward,
           settings->yaw_feedforward,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_tpa(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG TPA %.3f %.1f %u\n",
           settings->tpa_attenuation,
           settings->tpa_breakpoint_percent,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_filters(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG FILTERS %.1f %.1f %.1f %u\n",
           settings->gyro_lpf_hz,
           settings->dterm_lpf_hz,
           settings->dynamic_d_boost_percent,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_receiver_config(void)
{
    const flight_settings_t *settings = flight_settings_get();
    printf("@CFG RECEIVER_CONFIG SBUS %s %lu %lu %lu %lu %lu %lu %u\n",
           settings->receiver_channel_order == RECEIVER_ORDER_AETR1234
               ? "AETR1234" : "TAER1234",
           (unsigned long)(settings->arm_channel + 1u),
           (unsigned long)settings->arm_min_us,
           (unsigned long)settings->arm_max_us,
           (unsigned long)(settings->beep_channel + 1u),
           (unsigned long)settings->beep_min_us,
           (unsigned long)settings->beep_max_us,
           flight_settings_are_saved() ? 1u : 0u);
}

static void send_all_settings(void)
{
    send_pids();
    send_motor_protocol();
    send_board_alignment();
    send_motor_direction();
    send_motor_idle();
    send_rates();
    send_feedforward();
    send_tpa();
    send_filters();
    send_receiver_config();
    send_main_loop();
    send_vbat_multiplier();
}

static void send_flight_log_info(const sbus_frame_t *receiver)
{
    sbus_diagnostics_t diagnostics;
    sbus_receiver_get_diagnostics(&diagnostics);
    const unsigned int loss_reason =
        receiver->failsafe ? 1u :
        (!receiver->signal_valid || receiver->frame_lost ? 2u : 0u);
    printf("@CFG FLIGHT_LOG_INFO %lu %u %u %u %lu %lu %lu %u\n",
           (unsigned long)flight_log_count(),
           FLIGHT_LOG_RATE_HZ,
           flight_log_is_recording() ? 1u : 0u,
           loss_reason,
           (unsigned long)(receiver->frame_age_us / 1000u),
           (unsigned long)diagnostics.valid_frames,
           (unsigned long)(diagnostics.parity_errors +
                           diagnostics.stop_errors),
           0u);
}

static void process_command(const char *command,
                            const sbus_frame_t *receiver,
                            bool armed)
{
    if (strcmp(command, "HELLO") == 0) {
        client_active = true;
        last_client_activity_us = time_us_32();
        printf("@CFG HELLO FlightCode 3 PICO2_W %s\n", FLIGHTCODE_VERSION);
        printf("@CFG CAPABILITIES PIDS MOTOR_TEST TELEMETRY MOTOR_PROTOCOL MAIN_LOOP "
               "BOARD_ALIGNMENT MOTOR_DIRECTION MOTOR_IDLE RATES "
               "FEEDFORWARD TPA FILTERS GYRO_CALIBRATION FLIGHT_LOG PID_SIM DFU REBOOT "
               "TELEMETRY_EXT RECEIVER_CONFIG BATTERY_VOLTAGE VBAT_CALIBRATION\n");
        printf("@CFG RECEIVER_PROTOCOLS SBUS\n");
        printf("@CFG IMU %s %u\n",
               imu_get_name(),
               imu_is_available() ? 1u : 0u);
        send_all_settings();
        return;
    }
    if (strcmp(command, "PING") == 0) {
        client_active = true;
        last_client_activity_us = time_us_32();
        return;
    }
    if (strcmp(command, "BYE") == 0) {
        client_active = false;
        motor_test_enabled = false;
        pid_simulation_enabled = false;
        flight_log_set_inhibited(false);
        return;
    }
    if (strcmp(command, "GET_PIDS") == 0) {
        send_pids();
        return;
    }
    if (strcmp(command, "GET_MOTOR_PROTOCOL") == 0 ||
        strcmp(command, "GET_DSHOT") == 0) {
        send_motor_protocol();
        return;
    }
    if (strcmp(command, "GET_MAIN_LOOP") == 0) {
        send_main_loop();
        return;
    }
    if (strcmp(command, "GET_VBAT_MULTIPLIER") == 0) {
        send_vbat_multiplier();
        return;
    }
    if (strcmp(command, "GET_BOARD_ALIGNMENT") == 0) {
        send_board_alignment();
        return;
    }
    if (strcmp(command, "GET_MOTOR_DIRECTION") == 0) {
        send_motor_direction();
        return;
    }
    if (strcmp(command, "GET_MOTOR_IDLE") == 0) {
        send_motor_idle();
        return;
    }
    if (strcmp(command, "GET_RATES") == 0) {
        send_rates();
        return;
    }
    if (strcmp(command, "GET_FEEDFORWARD") == 0) {
        send_feedforward();
        return;
    }
    if (strcmp(command, "GET_TPA") == 0) {
        send_tpa();
        return;
    }
    if (strcmp(command, "GET_FILTERS") == 0) {
        send_filters();
        return;
    }
    if (strcmp(command, "GET_RECEIVER_CONFIG") == 0) {
        send_receiver_config();
        return;
    }
    if (strcmp(command, "GET_FLIGHT_LOG_INFO") == 0) {
        send_flight_log_info(receiver);
        return;
    }
    if (strcmp(command, "GET_FLIGHT_LOG_METADATA") == 0) {
        flight_log_metadata_t metadata;
        if (!flight_log_get_metadata(&metadata)) {
            printf("@CFG FLIGHT_LOG_METADATA_UNAVAILABLE\n");
            return;
        }
        printf("@CFG FLIGHT_LOG_METADATA_CORE %lu %lu %lu %lu %lu %lu %lu %u %.2f\n",
               (unsigned long)metadata.version,
               (unsigned long)metadata.main_loop_hz,
               (unsigned long)metadata.gyro_rate_hz,
               (unsigned long)metadata.log_rate_hz,
               (unsigned long)metadata.motor_protocol,
               (unsigned long)metadata.motor_direction_reversed,
               (unsigned long)metadata.receiver_protocol,
               metadata.initial_battery_cells,
               metadata.initial_battery_centivolts / 100.0f);
        printf("@CFG FLIGHT_LOG_METADATA_PIDS %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
               metadata.pids[0], metadata.pids[1], metadata.pids[2],
               metadata.pids[3], metadata.pids[4], metadata.pids[5],
               metadata.pids[6], metadata.pids[7], metadata.pids[8]);
        printf("@CFG FLIGHT_LOG_METADATA_TUNING %.2f %.2f %.2f %.4f %.6f %.6f %.6f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.1f\n",
               metadata.rates[0], metadata.rates[1], metadata.rates[2],
               metadata.rates[3], metadata.feedforward[0],
               metadata.feedforward[1], metadata.feedforward[2],
               metadata.tpa[0], metadata.tpa[1], metadata.filters[0],
               metadata.filters[1], metadata.alignment[0],
               metadata.alignment[1], metadata.alignment[2],
               metadata.motor_idle_percent, metadata.reserved / 2.0f);
        printf("@CFG FLIGHT_LOG_METADATA_END\n");
        return;
    }

    unsigned int log_offset;
    unsigned int log_count;
    if (sscanf(command, "GET_FLIGHT_LOG_CHUNK %u %u",
               &log_offset, &log_count) == 2) {
        if (flight_log_is_recording()) {
            printf("@CFG ERROR FLIGHT_LOG_RECORDING\n");
            return;
        }
        if (log_count > 4u) log_count = 4u;
        uint32_t sent = 0u;
        for (; sent < log_count; ++sent) {
            flight_log_record_t item;
            if (!flight_log_get((uint32_t)log_offset + sent, &item)) break;
            printf("@CFG FLIGHT_LOG %lu "
                   "%d %d %d %d %d %d "
                   "%u %u %u %u %u %u %u %u %u %u %u "
                   "%d %d %d %d %d %d %d %d %d %d %d %d\n",
                   (unsigned long)((uint32_t)log_offset + sent),
                   item.gyro[0], item.gyro[1], item.gyro[2],
                   item.setpoint[0], item.setpoint[1], item.setpoint[2],
                   item.motor[0], item.motor[1],
                   item.motor[2], item.motor[3],
                   item.throttle, item.flags, item.main_loop_us,
                   item.gyro_loop_us,
                   item.battery_centivolts, item.cell_centivolts,
                   item.battery_cells,
                   item.p_term[0], item.p_term[1], item.p_term[2],
                   item.i_term[0], item.i_term[1], item.i_term[2],
                   item.d_term[0], item.d_term[1], item.d_term[2],
                   item.ff_term[0], item.ff_term[1], item.ff_term[2]);
        }
        printf("@CFG FLIGHT_LOG_CHUNK_END %lu\n",
               (unsigned long)((uint32_t)log_offset + sent));
        return;
    }
    if (strcmp(command, "PID_SIM_RESET") == 0 &&
        pid_simulation_enabled) {
        rate_controller_reset();
        printf("@CFG OK PID_SIM_RESET\n");
        return;
    }
    if (armed) {
        printf("@CFG ERROR ARMED\n");
        return;
    }
    if (strcmp(command, "CALIBRATE_GYRO") == 0) {
        rate_controller_start_calibration();
        printf("@CFG OK CALIBRATE_GYRO\n");
        return;
    }
    if (strcmp(command, "ENTER_DFU") == 0) {
        motor_test_enabled = false;
        pid_simulation_enabled = false;
        memset(motor_test_percent, 0, sizeof(motor_test_percent));
        dfu_pending = true;
        dfu_at_us = time_us_32() + DFU_DELAY_US;
        printf("@CFG OK ENTER_DFU\n");
        return;
    }
    if (strcmp(command, "REBOOT") == 0) {
        if (armed) {
            printf("@CFG ERROR ARMED\n");
            return;
        }
        reboot_pending = true;
        reboot_at_us = time_us_32() + DFU_DELAY_US;
        printf("@CFG OK REBOOT\n");
        return;
    }

    unsigned int enable;
    if (sscanf(command, "PID_SIM_ENABLE %u", &enable) == 1) {
        pid_simulation_enabled = enable != 0u;
        motor_test_enabled = false;
        memset(motor_test_percent, 0, sizeof(motor_test_percent));
        flight_log_set_inhibited(pid_simulation_enabled);
        printf("@CFG OK PID_SIM_%s\n",
               pid_simulation_enabled ? "ENABLED" : "DISABLED");
        return;
    }
    if (sscanf(command, "MOTOR_TEST_ENABLE %u", &enable) == 1) {
        if (enable == 0u) {
            motor_test_enabled = false;
            memset(motor_test_percent, 0, sizeof(motor_test_percent));
            printf("@CFG OK MOTOR_TEST_DISABLED\n");
        } else if (arm_mode_active(receiver)) {
            printf("@CFG ERROR ARM_SWITCH\n");
        } else {
            pid_simulation_enabled = false;
            flight_log_set_inhibited(false);
            motor_test_enabled = true;
            last_motor_test_us = time_us_32();
            printf("@CFG OK MOTOR_TEST_ENABLED\n");
        }
        return;
    }

    float test[4];
    if (sscanf(command, "MOTOR_TEST %f %f %f %f",
               &test[0], &test[1], &test[2], &test[3]) == 4) {
        if (!motor_test_enabled) {
            printf("@CFG ERROR MOTOR_TEST_DISABLED\n");
            return;
        }
        for (uint8_t i = 0u; i < 4u; ++i) {
            if (test[i] < 0.0f || test[i] > 100.0f) {
                printf("@CFG ERROR MOTOR_TEST_RANGE\n");
                return;
            }
            motor_test_percent[i] = (uint8_t)(test[i] + 0.5f);
        }
        last_motor_test_us = time_us_32();
        return;
    }

    flight_settings_t settings = *flight_settings_get();
    if (sscanf(command, "SET_VBAT_MULTIPLIER %f",
               &settings.vbat_multiplier) == 1) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_VBAT_MULTIPLIER\n"
                   : "@CFG ERROR INVALID_VBAT_MULTIPLIER\n");
        send_vbat_multiplier();
        return;
    }
    char receiver_protocol[8], receiver_order[16];
    unsigned int arm_channel, arm_min, arm_max;
    unsigned int beep_channel, beep_min, beep_max;
    bool receiver_config_match = false;
    if (sscanf(command, "SET_RECEIVER_CONFIG %7s %15s %u %u %u %u %u %u",
               receiver_protocol, receiver_order, &arm_channel, &arm_min,
               &arm_max, &beep_channel, &beep_min, &beep_max) == 8) {
        if (strcmp(receiver_protocol, "SBUS") != 0) {
            printf("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        receiver_config_match = true;
    } else if (sscanf(command, "SET_RECEIVER_CONFIG %15s %u %u %u %u %u %u",
                      receiver_order, &arm_channel, &arm_min, &arm_max,
                      &beep_channel, &beep_min, &beep_max) == 7) {
        receiver_config_match = true;
    }
    if (receiver_config_match) {
        if (strcmp(receiver_order, "TAER1234") == 0) {
            settings.receiver_channel_order = RECEIVER_ORDER_TAER1234;
        } else if (strcmp(receiver_order, "AETR1234") == 0) {
            settings.receiver_channel_order = RECEIVER_ORDER_AETR1234;
        } else if (arm_channel < 5u || arm_channel > SBUS_CHANNEL_COUNT ||
                   beep_channel < 5u || beep_channel > SBUS_CHANNEL_COUNT) {
            printf("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        } else {
            printf("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        if (arm_channel < 5u || arm_channel > SBUS_CHANNEL_COUNT ||
            beep_channel < 5u || beep_channel > SBUS_CHANNEL_COUNT) {
            printf("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        settings.arm_channel = arm_channel - 1u;
        settings.arm_min_us = arm_min;
        settings.arm_max_us = arm_max;
        settings.beep_channel = beep_channel - 1u;
        settings.beep_min_us = beep_min;
        settings.beep_max_us = beep_max;
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_RECEIVER_CONFIG\n"
                   : "@CFG ERROR INVALID_RECEIVER_CONFIG\n");
        send_receiver_config();
        return;
    }
    if (sscanf(command, "SET_TPA %f %f",
               &settings.tpa_attenuation,
               &settings.tpa_breakpoint_percent) == 2) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_TPA\n"
                   : "@CFG ERROR INVALID_TPA\n");
        send_tpa();
        return;
    }
    if (sscanf(command, "SET_FILTERS %f %f %f",
               &settings.gyro_lpf_hz,
               &settings.dterm_lpf_hz,
               &settings.dynamic_d_boost_percent) == 3) {
        const bool applied = flight_settings_set(&settings);
        printf(applied ? "@CFG OK SET_FILTERS\n"
                       : "@CFG ERROR INVALID_FILTERS\n");
        if (applied) {
            rate_controller_reset();
        }
        send_filters();
        return;
    }
    if (sscanf(command, "SET_FEEDFORWARD %f %f %f",
               &settings.roll_feedforward,
               &settings.pitch_feedforward,
               &settings.yaw_feedforward) == 3) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_FEEDFORWARD\n"
                   : "@CFG ERROR INVALID_FEEDFORWARD\n");
        send_feedforward();
        return;
    }
    if (sscanf(command, "SET_RATES %f %f %f %f",
               &settings.roll_rate_dps,
               &settings.pitch_rate_dps,
               &settings.yaw_rate_dps,
               &settings.rate_expo) == 4) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_RATES\n"
                   : "@CFG ERROR INVALID_RATES\n");
        send_rates();
        return;
    }
    if (sscanf(command, "SET_MOTOR_IDLE %f",
               &settings.motor_idle_percent) == 1) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_MOTOR_IDLE\n"
                   : "@CFG ERROR INVALID_MOTOR_IDLE\n");
        send_motor_idle();
        return;
    }

    unsigned int main_loop_hz;
    if (sscanf(command, "SET_MAIN_LOOP %u", &main_loop_hz) == 1) {
        settings = *flight_settings_get();
        settings.main_loop_hz = main_loop_hz;
        if (armed) {
            printf("@CFG ERROR ARMED\n");
        } else if (flight_settings_set(&settings)) {
            printf("@CFG OK SET_MAIN_LOOP REBOOT_REQUIRED\n");
            send_main_loop();
        } else {
            printf("@CFG ERROR INVALID_MAIN_LOOP\n");
        }
        return;
    }
    char name[24];
    if (sscanf(command, "SET_MOTOR_DIRECTION %23s", name) == 1) {
        if (strcmp(name, "NORMAL") == 0) {
            settings.motor_direction_reversed = 0u;
        } else if (strcmp(name, "REVERSED") == 0) {
            settings.motor_direction_reversed = 1u;
        } else {
            printf("@CFG ERROR INVALID_MOTOR_DIRECTION\n");
            return;
        }
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_MOTOR_DIRECTION\n"
                   : "@CFG ERROR SET_MOTOR_DIRECTION\n");
        send_motor_direction();
        return;
    }
    if (sscanf(command, "SET_BOARD_ALIGNMENT %f %f %f",
               &settings.board_roll_deg,
               &settings.board_pitch_deg,
               &settings.board_yaw_deg) == 3) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_BOARD_ALIGNMENT\n"
                   : "@CFG ERROR INVALID_BOARD_ALIGNMENT\n");
        rate_controller_start_calibration();
        send_board_alignment();
        return;
    }
    if (sscanf(command,
               "SET_PIDS %f %f %f %f %f %f %f %f %f",
               &settings.roll.kp, &settings.roll.ki, &settings.roll.kd,
               &settings.pitch.kp, &settings.pitch.ki, &settings.pitch.kd,
               &settings.yaw.kp, &settings.yaw.ki, &settings.yaw.kd) == 9) {
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_PIDS\n"
                   : "@CFG ERROR INVALID_PIDS\n");
        rate_controller_reset();
        send_pids();
        return;
    }
    if (strcmp(command, "SAVE_PIDS") == 0 ||
        strcmp(command, "SAVE_SETTINGS") == 0) {
        printf(flight_settings_save()
                   ? "@CFG OK SAVE_SETTINGS\n"
                   : "@CFG ERROR SAVE_SETTINGS\n");
        send_all_settings();
        return;
    }
    if (strcmp(command, "RESET_PIDS") == 0) {
        flight_settings_reset_tuning_defaults(&settings);
        printf(flight_settings_set(&settings)
                   ? "@CFG OK RESET_PIDS\n"
                   : "@CFG ERROR RESET_PIDS\n");
        rate_controller_reset();
        send_pids();
        send_rates();
        send_feedforward();
        send_tpa();
        send_filters();
        send_main_loop();
        return;
    }
    if (sscanf(command, "SET_MOTOR_PROTOCOL %23s", name) == 1) {
        unsigned int rate;
        if (sscanf(name, "DSHOT%u", &rate) != 1 ||
            !esc_controller_set_dshot_rate(rate)) {
            printf("@CFG ERROR INVALID_MOTOR_PROTOCOL\n");
            return;
        }
        settings.dshot_rate_kbps = rate;
        printf(flight_settings_set(&settings)
                   ? "@CFG OK SET_MOTOR_PROTOCOL\n"
                   : "@CFG ERROR SET_MOTOR_PROTOCOL\n");
        send_motor_protocol();
        return;
    }
    unsigned int rate;
    if (sscanf(command, "SET_DSHOT %u", &rate) == 1) {
        if (!esc_controller_set_dshot_rate(rate)) {
            printf("@CFG ERROR DSHOT_RATE\n");
            return;
        }
        settings.dshot_rate_kbps = rate;
        flight_settings_set(&settings);
        send_motor_protocol();
        return;
    }
    printf("@CFG ERROR UNKNOWN_COMMAND\n");
}

void config_protocol_init(void)
{
    input_length = 0u;
    client_active = false;
    last_client_activity_us = 0u;
    motor_test_enabled = false;
    pid_simulation_enabled = false;
    memset(motor_test_percent, 0, sizeof(motor_test_percent));
    dfu_pending = false;
    reboot_pending = false;
}

void config_protocol_update(const sbus_frame_t *receiver, bool armed)
{
    int character;
    while ((character = getchar_timeout_us(0u)) != PICO_ERROR_TIMEOUT) {
        if (character == '\r') continue;
        if (character == '\n') {
            input_line[input_length] = '\0';
            if (input_length > 0u) {
                process_command(input_line, receiver, armed);
            }
            input_length = 0u;
        } else if (input_length < CONFIG_LINE_LENGTH - 1u) {
            input_line[input_length++] = (char)character;
        } else {
            input_length = 0u;
            printf("@CFG ERROR LINE_TOO_LONG\n");
        }
    }

    if (dfu_pending && (int32_t)(time_us_32() - dfu_at_us) >= 0) {
        reset_usb_boot(0u, 0u);
    }
    if (reboot_pending && (int32_t)(time_us_32() - reboot_at_us) >= 0) {
        watchdog_reboot(0u, 0u, 0u);
    }
    if (client_active &&
        time_us_32() - last_client_activity_us >
            CONFIG_CLIENT_TIMEOUT_US) {
        client_active = false;
        motor_test_enabled = false;
        pid_simulation_enabled = false;
        flight_log_set_inhibited(false);
    }
}

bool config_protocol_is_client_active(void)
{
    return client_active;
}

bool config_protocol_motor_output_suppressed(void)
{
    return pid_simulation_enabled;
}

bool config_protocol_pid_simulation_enabled(void)
{
    return pid_simulation_enabled;
}

bool config_protocol_get_motor_test(const sbus_frame_t *receiver,
                                    uint8_t output[4])
{
    if (!motor_test_enabled ||
        arm_mode_active(receiver) ||
        time_us_32() - last_motor_test_us > MOTOR_TEST_TIMEOUT_US) {
        motor_test_enabled = false;
        memset(motor_test_percent, 0, sizeof(motor_test_percent));
        return false;
    }
    memcpy(output, motor_test_percent, sizeof(motor_test_percent));
    return true;
}

void config_protocol_send_telemetry(const sbus_frame_t *receiver,
                                    const imu_sample_t *imu,
                                    bool armed,
                                    float loop_frequency_hz,
                                    uint32_t max_loop_period_us,
                                    const rate_controller_output_t *control)
{
    if (!client_active) return;

    printf("@CFG BATTERY_VOLTAGE %.2f\n", battery_voltage_get());

    static uint32_t last_sbus_diagnostics_us;
    const uint32_t now_us = time_us_32();
    if ((uint32_t)(now_us - last_sbus_diagnostics_us) >= 500000u) {
        last_sbus_diagnostics_us = now_us;
        sbus_diagnostics_t diagnostics;
        sbus_receiver_get_diagnostics(&diagnostics);
        printf("@CFG SBUS_DIAGNOSTICS %u %lu %lu %lu %u %u %u\n",
               receiver->signal_valid ? 1u : 0u,
               (unsigned long)(receiver->frame_age_us / 1000u),
               (unsigned long)diagnostics.valid_frames,
               (unsigned long)(diagnostics.parity_errors +
                               diagnostics.stop_errors),
               0u, 0u, 0u);
    }

    imu_sample_t corrected;
    rate_controller_get_corrected_imu(imu, &corrected);
    printf("@CFG TELEMETRY %lu %u %u %.1f "
           "%.3f %.3f %.3f %.3f %.3f %.3f",
           (unsigned long)time_us_32(),
           receiver->signal_valid ? 1u : 0u,
           armed ? 1u : 0u,
           loop_frequency_hz,
           corrected.gyro_x_dps,
           corrected.gyro_y_dps,
           corrected.gyro_z_dps,
           corrected.accel_x_g,
           corrected.accel_y_g,
           corrected.accel_z_g);
    for (uint8_t i = 0u; i < SBUS_CHANNEL_COUNT; ++i) {
        printf(" %u", receiver->signal_valid ? receiver->channel_us[i] : 0u);
    }
    for (uint8_t i = 0u; i < 4u; ++i) {
        printf(" %.2f", control->motor_percent[i]);
    }
    printf(" %u %lu %.3f %.3f %.3f %u %.2f\n",
           rate_controller_is_calibrated() ? 1u : 0u,
           (unsigned long)max_loop_period_us,
           imu->gyro_x_dps,
           imu->gyro_y_dps,
           imu->gyro_z_dps,
           rate_controller_get_calibration_samples(),
           imu->temperature_c);
}
