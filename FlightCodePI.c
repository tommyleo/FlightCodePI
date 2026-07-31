#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "config_protocol.h"
#include "esc_controller.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "imu.h"
#include "rate_controller.h"
#include "sbus_receiver.h"

#define SBUS_INPUT_GPIO 0u
#define ESC_1_GPIO 1u
#define ESC_2_GPIO 2u
#define ESC_3_GPIO 3u
#define ESC_4_GPIO 6u
#define BUZZER_GPIO 7u
#define ESC_COUNT 4u
#define FLIGHT_LOOP_HZ 8000u
#define FLIGHT_LOOP_PERIOD_US (1000000u / FLIGHT_LOOP_HZ)
#define THROTTLE_CHANNEL_INDEX 0u  // CH1 della ricevente
#define ROLL_CHANNEL_INDEX 1u      // CH2 della ricevente
#define PITCH_CHANNEL_INDEX 2u     // CH3 della ricevente
#define YAW_CHANNEL_INDEX 3u       // CH4 della ricevente
#define BUZZER_CHANNEL_INDEX 4u    // CH5 della ricevente
#define ARM_CHANNEL_INDEX 5u       // CH6 della ricevente
#define ARM_THRESHOLD_US 2000u
#define BUZZER_THRESHOLD_US 2000u
#define THROTTLE_MIN_US 1000u
#define THROTTLE_MAX_US 2000u
#define ARM_THROTTLE_MAX_PERCENT 5u
#define STICK_CENTER_US 1500
#define STICK_RANGE_US 500
#define STICK_DEADBAND_US 10

static bool escs_armed = false;
static uint8_t escs_throttle_percent = 0u;
static int8_t yaw_percent = 0;
static int8_t roll_percent = 0;
static int8_t pitch_percent = 0;
static rate_controller_output_t rate_output;
static bool arm_switch_was_low;

static void update_buzzer(const sbus_frame_t *receiver)
{
    const bool buzzer_active =
        receiver->signal_valid &&
        receiver->channel_us[BUZZER_CHANNEL_INDEX] > BUZZER_THRESHOLD_US;
    gpio_put(BUZZER_GPIO, buzzer_active);
}

static int8_t stick_us_to_percent(uint16_t value_us)
{
    int32_t centered = (int32_t)value_us - STICK_CENTER_US;

    if (centered >= -STICK_DEADBAND_US && centered <= STICK_DEADBAND_US) {
        return 0;
    }

    if (centered > STICK_RANGE_US) {
        centered = STICK_RANGE_US;
    } else if (centered < -STICK_RANGE_US) {
        centered = -STICK_RANGE_US;
    }

    return (int8_t)((centered * 100) / STICK_RANGE_US);
}

static void stop_all_escs(esc_controller_t escs[ESC_COUNT])
{
    for (uint8_t i = 0u; i < ESC_COUNT; ++i) {
        esc_controller_stop(&escs[i]);
    }
}

static void set_esc_outputs(esc_controller_t escs[ESC_COUNT],
                            const rate_controller_output_t *output)
{
    for (uint8_t i = 0u; i < ESC_COUNT; ++i) {
        esc_controller_set_throttle_percent_float(&escs[i],
                                                  output->motor_percent[i]);
    }
}

static bool apply_configurator_motor_test(const sbus_frame_t *receiver,
                                          esc_controller_t escs[ESC_COUNT])
{
    uint8_t test_percent[ESC_COUNT];
    if (!config_protocol_get_motor_test(receiver, test_percent)) {
        return false;
    }
    escs_armed = false;
    for (uint8_t i = 0u; i < ESC_COUNT; ++i) {
        esc_controller_set_throttle_percent(&escs[i], test_percent[i]);
        rate_output.motor_percent[i] = (float)test_percent[i];
    }
    return true;
}

static void stop_flight(uint8_t reason,
                        esc_controller_t escs[ESC_COUNT])
{
    if (escs_armed) {
        flight_log_stop(reason);
    }
    escs_armed = false;
    escs_throttle_percent = 0u;
    yaw_percent = roll_percent = pitch_percent = 0;
    rate_controller_reset();
    memset(&rate_output, 0, sizeof(rate_output));
    stop_all_escs(escs);
}

static void flight_control_step(const sbus_frame_t *receiver,
                                esc_controller_t escs[ESC_COUNT],
                                uint16_t loop_period_us)
{
    const imu_sample_t *imu = imu_get_latest_sample();
    if (!imu->valid) {
        stop_flight(FLIGHT_LOG_FLAG_STOP_IMU, escs);
        return;
    }
    if (!receiver->signal_valid) {
        uint8_t reason = FLIGHT_LOG_FLAG_STOP_RX_LOSS;
        reason |= receiver->failsafe
            ? FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE
            : FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT;
        stop_flight(reason, escs);
        return;
    }

    yaw_percent =
        stick_us_to_percent(receiver->channel_us[YAW_CHANNEL_INDEX]);
    roll_percent =
        stick_us_to_percent(receiver->channel_us[ROLL_CHANNEL_INDEX]);
    pitch_percent =
        stick_us_to_percent(receiver->channel_us[PITCH_CHANNEL_INDEX]);

    const uint16_t throttle_us = receiver->channel_us[THROTTLE_CHANNEL_INDEX];
    uint8_t throttle_percent = 0u;

    if (throttle_us >= THROTTLE_MAX_US) {
        throttle_percent = 100u;
    } else if (throttle_us > THROTTLE_MIN_US) {
        throttle_percent = (uint8_t)(
            ((uint32_t)(throttle_us - THROTTLE_MIN_US) * 100u) /
            (THROTTLE_MAX_US - THROTTLE_MIN_US));
    }

    escs_throttle_percent = throttle_percent;
    const bool arm_switch =
        receiver->channel_us[ARM_CHANNEL_INDEX] > ARM_THRESHOLD_US;
    if (!arm_switch) {
        arm_switch_was_low = true;
        if (escs_armed) {
            stop_flight(FLIGHT_LOG_FLAG_STOP_DISARM, escs);
        }
    }

    const bool simulation = config_protocol_pid_simulation_enabled();
    const bool control_allowed =
        !config_protocol_is_client_active() || simulation;
    const bool arm_requested = control_allowed && arm_switch;
    if (!arm_requested) {
        if (!control_allowed && escs_armed) {
            stop_flight(FLIGHT_LOG_FLAG_STOP_DISARM, escs);
            arm_switch_was_low = false;
            return;
        }
        rate_controller_update(imu, false, throttle_percent,
                               roll_percent, pitch_percent, yaw_percent,
                               &rate_output);
        stop_all_escs(escs);
        return;
    }

    if (!escs_armed) {
        if (!arm_switch_was_low ||
            throttle_percent > ARM_THROTTLE_MAX_PERCENT ||
            !rate_controller_is_calibrated()) {
            rate_controller_update(imu, false, throttle_percent,
                                   roll_percent, pitch_percent, yaw_percent,
                                   &rate_output);
            stop_all_escs(escs);
            return;
        }
        escs_armed = true;
        if (!simulation) {
            flight_log_start();
        }
    }

    const bool controller_active =
        rate_controller_update(imu,
                               escs_armed,
                               throttle_percent,
                               roll_percent,
                               pitch_percent,
                               yaw_percent,
                               &rate_output);
    if (!controller_active) {
        stop_all_escs(escs);
        return;
    }

    const float gyro[3] = {
        rate_output.roll_rate_dps,
        rate_output.pitch_rate_dps,
        rate_output.yaw_rate_dps,
    };
    const float setpoints[3] = {
        rate_output.roll_setpoint_dps,
        rate_output.pitch_setpoint_dps,
        rate_output.yaw_setpoint_dps,
    };
    const float pid[3] = {
        rate_output.roll_pid_percent,
        rate_output.pitch_pid_percent,
        rate_output.yaw_pid_percent,
    };
    flight_log_record(gyro, setpoints, pid, rate_output.motor_percent,
                      (float)throttle_percent,
                      rate_output.mixer_saturated,
                      loop_period_us);

    if (config_protocol_motor_output_suppressed()) {
        stop_all_escs(escs);
    } else {
        set_esc_outputs(escs, &rate_output);
    }
}

int main(void)
{
    stdio_init_all();
    flight_settings_init();
    esc_controller_set_dshot_rate(
        flight_settings_get()->dshot_rate_kbps);
    config_protocol_init();
    sbus_receiver_init(SBUS_INPUT_GPIO);
    gpio_init(BUZZER_GPIO);
    gpio_set_dir(BUZZER_GPIO, GPIO_OUT);
    gpio_put(BUZZER_GPIO, false);
    imu_init();
    rate_controller_init();
    flight_log_init();
    arm_switch_was_low = false;

    static const unsigned int esc_gpios[ESC_COUNT] = {
        ESC_1_GPIO,
        ESC_2_GPIO,
        ESC_3_GPIO,
        ESC_4_GPIO,
    };
    esc_controller_t escs[ESC_COUNT];
    for (uint8_t i = 0u; i < ESC_COUNT; ++i) {
        esc_controller_init(&escs[i], esc_gpios[i]);
    }

    sbus_frame_t receiver = {0};
    absolute_time_t next_loop =
        delayed_by_us(get_absolute_time(), FLIGHT_LOOP_PERIOD_US);
    absolute_time_t next_telemetry = delayed_by_ms(next_loop, 40u);
    uint32_t loop_measurement_start_us = time_us_32();
    uint32_t loop_measurement_count = 0u;
    float loop_frequency_hz = (float)FLIGHT_LOOP_HZ;
    uint32_t maximum_loop_period_us = 0u;
    uint32_t previous_loop_start_us = time_us_32();

    while (true) {
        busy_wait_until(next_loop);
        const uint32_t loop_start_us = time_us_32();
        const uint32_t loop_period_us = loop_start_us - previous_loop_start_us;
        previous_loop_start_us = loop_start_us;
        if (loop_period_us > maximum_loop_period_us) {
            maximum_loop_period_us = loop_period_us;
        }
        ++loop_measurement_count;
        const uint32_t measurement_period_us =
            loop_start_us - loop_measurement_start_us;
        if (measurement_period_us >= 250000u) {
            loop_frequency_hz =
                ((float)loop_measurement_count * 1000000.0f) /
                (float)measurement_period_us;
            loop_measurement_start_us = loop_start_us;
            loop_measurement_count = 0u;
        }

        next_loop = delayed_by_us(next_loop, FLIGHT_LOOP_PERIOD_US);

        sbus_receiver_read(&receiver);
        const bool simulation_was_enabled =
            config_protocol_pid_simulation_enabled();
        config_protocol_update(&receiver, escs_armed);
        if (simulation_was_enabled &&
            !config_protocol_pid_simulation_enabled() &&
            escs_armed) {
            stop_flight(FLIGHT_LOG_FLAG_STOP_DISARM, escs);
            arm_switch_was_low = false;
        }
        update_buzzer(&receiver);
        imu_update();
        if (!apply_configurator_motor_test(&receiver, escs)) {
            flight_control_step(
                &receiver,
                escs,
                loop_period_us > UINT16_MAX
                    ? UINT16_MAX
                    : (uint16_t)loop_period_us);
        }
        if (!config_protocol_is_client_active()) {
            flight_log_persist_if_ready();
        }

        if (time_reached(next_telemetry)) {
            config_protocol_send_telemetry(&receiver,
                                           imu_get_latest_sample(),
                                           escs_armed,
                                           loop_frequency_hz,
                                           maximum_loop_period_us,
                                           &rate_output);
            next_telemetry = delayed_by_ms(next_telemetry, 40u);
            maximum_loop_period_us = 0u;
        }

        if (time_reached(next_loop)) {
            next_loop = delayed_by_us(get_absolute_time(),
                                      FLIGHT_LOOP_PERIOD_US);
        }
    }
}
