#pragma once

#define MOTOR_ADC_RESOLUTION  12

struct motor_adc_sample
{
    uint64_t timestamp;

    int phase_values[3];
    int input_voltage;
    int input_current;
    int input_temperature;
};

void motor_adc_init(void);
void update_voltage_current_temperate(void);

float motor_adc_get_voltage(void);
float motor_adc_get_current(void);
float motor_adc_get_temperature(void);

struct motor_adc_sample motor_adc_get_last_sample(void);
