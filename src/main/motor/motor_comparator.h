#pragma once

void motor_comparator_init(void);
void motor_comparator_set_input_source(uint_fast8_t phase);

void motor_comparator_enable_from_isr(void);
void motor_comparator_disable_from_isr(void);

extern void motor_comparator_zc_callback(bool compare_result);
