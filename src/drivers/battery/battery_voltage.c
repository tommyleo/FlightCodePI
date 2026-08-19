#include "battery_voltage.h"

#include "flight_settings.h"
#include "hardware/adc.h"

#define VBAT_GPIO 26u
#define VBAT_ADC_INPUT 0u
#define VBAT_ADC_REFERENCE_V 3.3f
#define VBAT_ADC_MAX 4095.0f
#define VBAT_BETAFLIGHT_DIVIDER 11.0f
#define VBAT_AVERAGE_SAMPLES 8u

static uint32_t adc_total;
static uint8_t adc_samples;
static float filtered_voltage;

void battery_voltage_init(void)
{
    adc_init();
    adc_gpio_init(VBAT_GPIO);
    adc_select_input(VBAT_ADC_INPUT);
    adc_total = 0u;
    adc_samples = 0u;
    filtered_voltage = 0.0f;
}

void battery_voltage_update(void)
{
    adc_select_input(VBAT_ADC_INPUT);
    adc_total += adc_read();
    ++adc_samples;
    if (adc_samples < VBAT_AVERAGE_SAMPLES) return;

    const float average = (float)adc_total / (float)VBAT_AVERAGE_SAMPLES;
    const float measured = average * VBAT_ADC_REFERENCE_V *
        VBAT_BETAFLIGHT_DIVIDER * flight_settings_get()->vbat_multiplier /
        VBAT_ADC_MAX;
    filtered_voltage = filtered_voltage <= 0.0f
        ? measured
        : filtered_voltage * 0.85f + measured * 0.15f;
    adc_total = 0u;
    adc_samples = 0u;
}

float battery_voltage_get(void)
{
    return filtered_voltage;
}
