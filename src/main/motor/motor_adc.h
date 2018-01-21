#pragma once

#define MOTOR_ADC_RESOLUTION  12

struct motor_adc_sample
{
    uint64_t timestamp;

    uint32_t phase_values[3];
    uint32_t input_voltage;
    uint32_t input_current;
    uint32_t input_temperature;
};

void motor_adc_init(void);

float motor_adc_get_voltage(void);
float motor_adc_get_current(void);
float motor_adc_get_temperature(void);

struct motor_adc_sample motor_adc_get_last_sample(void);
