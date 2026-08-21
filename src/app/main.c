#include <stdio.h>
#include <string.h>

#include "main.h"

#include "pico/stdlib.h"
#include "battery_voltage.h"
#include "config_protocol.h"
#include "esc_controller.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "imu.h"
#include "rate_controller.h"
#include "sbus_receiver.h"

#if defined(CYW43_WL_GPIO_LED_PIN)
#include "pico/cyw43_arch.h"
#endif

#define SBUS_INPUT_GPIO 0u
#define ESC_1_GPIO 1u
#define ESC_2_GPIO 2u
#define ESC_3_GPIO 3u
#define ESC_4_GPIO 6u
#define BUZZER_GPIO 7u
#define ESC_COUNT MAIN_ESC_COUNT
#define THROTTLE_MIN_US 1000u
#define THROTTLE_MAX_US 2000u
#define ARM_THROTTLE_MAX_PERCENT 5u
#define STICK_CENTER_US 1500
#define STICK_RANGE_US 500
#define STICK_DEADBAND_US 10
#define STATUS_LED_PATTERN_PERIOD_US 1000000u
#define STATUS_LED_FLASH_US 80000u
#define STATUS_LED_SECOND_FLASH_US 160000u
#define BUZZER_PATTERN_PERIOD_US 500000u
#define BUZZER_BEEP_US 70000u
#define BUZZER_SECOND_BEEP_US 120000u

static bool escs_armed = false;
static uint8_t escs_throttle_percent = 0u;
static int8_t yaw_percent = 0;
static int8_t roll_percent = 0;
static int8_t pitch_percent = 0;
static rate_controller_output_t rate_output;
static bool arm_switch_was_low;
static bool status_led_available;
static bool status_led_on;
static const unsigned int esc_gpios[ESC_COUNT] = {
    ESC_1_GPIO,
    ESC_2_GPIO,
    ESC_3_GPIO,
    ESC_4_GPIO,
};

static bool task_due(loop_task_t *task, uint32_t loop_hz)
{
    task->phase += task->rate_hz;
    if (task->phase < loop_hz) return false;
    task->phase -= loop_hz;
    return true;
}

static bool receiver_mode_active(const sbus_frame_t *receiver,
                                 uint32_t channel,
                                 uint32_t min_us,
                                 uint32_t max_us)
{
    return receiver->signal_valid && channel < SBUS_CHANNEL_COUNT &&
           receiver->channel_us[channel] >= min_us &&
           receiver->channel_us[channel] <= max_us;
}

static void get_primary_channels(uint8_t *throttle, uint8_t *roll,
                                 uint8_t *pitch, uint8_t *yaw)
{
    const bool aetr = flight_settings_get()->receiver_channel_order ==
                      RECEIVER_ORDER_AETR1234;
    *throttle = aetr ? 2u : 0u;
    *roll = aetr ? 0u : 1u;
    *pitch = aetr ? 1u : 2u;
    *yaw = 3u;
}

static void status_led_init(void)
{
#if defined(CYW43_WL_GPIO_LED_PIN)
    status_led_available = cyw43_arch_init() == 0;
    if (status_led_available) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
    }
#elif defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    status_led_available = true;
#else
    status_led_available = false;
#endif
    status_led_on = false;
}

static void status_led_set(bool enabled)
{
    if (!status_led_available || status_led_on == enabled) {
        return;
    }

    status_led_on = enabled;
#if defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, status_led_on);
#elif defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, status_led_on);
#endif
}

static void status_led_update(bool receiver_signal_valid)
{
    const uint32_t phase =
        time_us_32() % STATUS_LED_PATTERN_PERIOD_US;
    const bool first_flash = phase < STATUS_LED_FLASH_US;
    const bool second_flash = receiver_signal_valid &&
        phase >= STATUS_LED_SECOND_FLASH_US &&
        phase < STATUS_LED_SECOND_FLASH_US + STATUS_LED_FLASH_US;
    status_led_set(first_flash || second_flash);
}

static void update_buzzer(const sbus_frame_t *receiver)
{
    static bool was_active;
    static uint32_t started_us;
    const flight_settings_t *settings = flight_settings_get();
    const bool buzzer_active = receiver_mode_active(
        receiver, settings->beep_channel,
        settings->beep_min_us, settings->beep_max_us);
    if (!buzzer_active) {
        was_active = false;
        gpio_put(BUZZER_GPIO, false);
        return;
    }
    const uint32_t now_us = time_us_32();
    if (!was_active) {
        was_active = true;
        started_us = now_us;
    }
    const uint32_t phase =
        (uint32_t)(now_us - started_us) % BUZZER_PATTERN_PERIOD_US;
    const bool sounding = phase < BUZZER_BEEP_US ||
        (phase >= BUZZER_SECOND_BEEP_US &&
         phase < BUZZER_SECOND_BEEP_US + BUZZER_BEEP_US);
    gpio_put(BUZZER_GPIO, sounding);
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
    esc_controller_stop_all(escs, ESC_COUNT);
}

static void set_esc_outputs(esc_controller_t escs[ESC_COUNT],
                            const rate_controller_output_t *output)
{
    esc_controller_set_throttle_percent_all(
        escs, output->motor_percent, ESC_COUNT);
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
        rate_output.motor_percent[i] = (float)test_percent[i];
    }
    esc_controller_set_throttle_percent_all(
        escs, rate_output.motor_percent, ESC_COUNT);
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
                                uint16_t loop_period_us,
                                bool esc_update_due)
{
    const imu_sample_t *imu = imu_get_latest_sample();
    if (!imu->valid) {
        stop_flight(FLIGHT_LOG_FLAG_STOP_IMU, escs);
        return;
    }
    /*
     * Gyro calibration is independent of the receiver.  Keep feeding fresh
     * IMU samples while disarmed so bench calibration also works when no RX
     * is connected or its signal is currently invalid.
     */
    if (!rate_controller_is_calibrated()) {
        rate_controller_update(imu, false, 0u, 0, 0, 0, &rate_output);
        stop_all_escs(escs);
        return;
    }
    if (!receiver->signal_valid) {
        rate_controller_update(imu, false, 0u, 0, 0, 0, &rate_output);
        uint8_t reason = FLIGHT_LOG_FLAG_STOP_RX_LOSS;
        reason |= receiver->failsafe
            ? FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE
            : FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT;
        stop_flight(reason, escs);
        return;
    }

    uint8_t throttle_channel, roll_channel, pitch_channel, yaw_channel;
    get_primary_channels(&throttle_channel, &roll_channel,
                         &pitch_channel, &yaw_channel);
    yaw_percent = stick_us_to_percent(receiver->channel_us[yaw_channel]);
    roll_percent = stick_us_to_percent(receiver->channel_us[roll_channel]);
    pitch_percent = stick_us_to_percent(receiver->channel_us[pitch_channel]);

    const uint16_t throttle_us = receiver->channel_us[throttle_channel];
    uint8_t throttle_percent = 0u;

    if (throttle_us >= THROTTLE_MAX_US) {
        throttle_percent = 100u;
    } else if (throttle_us > THROTTLE_MIN_US) {
        throttle_percent = (uint8_t)(
            ((uint32_t)(throttle_us - THROTTLE_MIN_US) * 100u) /
            (THROTTLE_MAX_US - THROTTLE_MIN_US));
    }

    escs_throttle_percent = throttle_percent;
    const flight_settings_t *settings = flight_settings_get();
    const bool arm_switch = receiver_mode_active(
        receiver, settings->arm_channel,
        settings->arm_min_us, settings->arm_max_us);
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
    flight_log_record(gyro, setpoints, pid, rate_output.p_term_percent,
                      rate_output.i_term_percent,
                      rate_output.d_term_percent, rate_output.motor_percent,
                      (float)throttle_percent,
                      rate_output.mixer_saturated,
                      loop_period_us);

    if (config_protocol_motor_output_suppressed()) {
        stop_all_escs(escs);
    } else if (esc_update_due) {
        set_esc_outputs(escs, &rate_output);
    }
}

static void main_loop_state_init(main_loop_state_t *state)
{
    state->loop_hz = flight_settings_get()->main_loop_hz;
    state->next_loop = get_absolute_time();
    state->service_task = (loop_task_t){0u, 1000u};
    state->telemetry_task = (loop_task_t){0u, 25u};
    state->esc_task = (loop_task_t){
        0u, state->loop_hz < 16000u ? state->loop_hz : 16000u};
    state->imu_task = (loop_task_t){
        0u, imu_get_update_rate_hz(false, state->loop_hz)};
    state->loop_measurement_start_us = time_us_32();
    state->loop_frequency_hz = (float)state->loop_hz;
    state->previous_loop_start_us = time_us_32();
}

static void main_loop_step(main_loop_state_t *state)
{
    busy_wait_until(state->next_loop);
    const bool service_due = task_due(&state->service_task, state->loop_hz);
    const bool esc_update_due = task_due(&state->esc_task, state->loop_hz);
    const uint32_t loop_start_us = time_us_32();
    const uint32_t loop_period_us = loop_start_us - state->previous_loop_start_us;
    state->previous_loop_start_us = loop_start_us;
    if (loop_period_us > state->maximum_loop_period_us) {
        state->maximum_loop_period_us = loop_period_us;
    }
    ++state->loop_measurement_count;
    const uint32_t measurement_period_us =
        loop_start_us - state->loop_measurement_start_us;
    if (measurement_period_us >= 250000u) {
        state->loop_frequency_hz =
            ((float)state->loop_measurement_count * 1000000.0f) /
            (float)measurement_period_us;
        state->loop_measurement_start_us = loop_start_us;
        state->loop_measurement_count = 0u;
    }

    uint32_t loop_delay_us = 1000000u / state->loop_hz;
    state->timing_remainder += 1000000u % state->loop_hz;
    if (state->timing_remainder >= state->loop_hz) {
        state->timing_remainder -= state->loop_hz;
        ++loop_delay_us;
    }
    state->next_loop = delayed_by_us(state->next_loop, loop_delay_us);

    sbus_receiver_read(&state->receiver);
    if (service_due) {
        status_led_update(state->receiver.signal_valid);
    }
    if (service_due) {
        const bool simulation_was_enabled =
            config_protocol_pid_simulation_enabled();
        config_protocol_update(&state->receiver, escs_armed);
        if (simulation_was_enabled &&
            !config_protocol_pid_simulation_enabled() && escs_armed) {
            stop_flight(FLIGHT_LOG_FLAG_STOP_DISARM, state->escs);
            arm_switch_was_low = false;
        }
    }
    if (service_due) {
        update_buzzer(&state->receiver);
        battery_voltage_update();
        flight_log_set_battery_voltage(battery_voltage_get());
    }
    state->imu_task.rate_hz =
        imu_get_update_rate_hz(escs_armed, state->loop_hz);
    const bool imu_due = task_due(&state->imu_task, state->loop_hz);
    if (imu_due) {
        imu_update(escs_armed);
    }
    if (!apply_configurator_motor_test(&state->receiver, state->escs)) {
        flight_control_step(
            &state->receiver,
            state->escs,
            loop_period_us > UINT16_MAX
                ? UINT16_MAX
                : (uint16_t)loop_period_us,
            esc_update_due);
    }
    if (!config_protocol_is_client_active()) {
        flight_log_persist_if_ready();
    }

    if (task_due(&state->telemetry_task, state->loop_hz)) {
        config_protocol_send_telemetry(&state->receiver,
                                       imu_get_latest_sample(),
                                       escs_armed,
                                       state->loop_frequency_hz,
                                       state->maximum_loop_period_us,
                                       &rate_output);
        state->maximum_loop_period_us = 0u;
    }

    if (time_reached(state->next_loop)) {
        state->next_loop = delayed_by_us(get_absolute_time(),
                                         1000000u / state->loop_hz);
    }
}

int main(void)
{
    esc_controller_preinit(esc_gpios, ESC_COUNT);
    stdio_init_all();
    status_led_init();
    flight_settings_init();
    battery_voltage_init();
    esc_controller_set_dshot_rate(
        flight_settings_get()->dshot_rate_kbps);

    main_loop_state_t loop = {0};
    for (uint8_t i = 0u; i < ESC_COUNT; ++i) {
        esc_controller_init(&loop.escs[i], esc_gpios[i]);
    }
    esc_controller_startup_sequence(loop.escs, ESC_COUNT);

    config_protocol_init();
    sbus_receiver_init(SBUS_INPUT_GPIO);
    gpio_init(BUZZER_GPIO);
    gpio_set_dir(BUZZER_GPIO, GPIO_OUT);
    gpio_put(BUZZER_GPIO, false);
    imu_init(flight_settings_get()->main_loop_hz);
    rate_controller_init();
    flight_log_init();
    arm_switch_was_low = false;

    main_loop_state_init(&loop);

    while (true) {
        main_loop_step(&loop);
    }
}
