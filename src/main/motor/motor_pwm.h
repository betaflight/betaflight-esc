#pragma once

typedef enum {
    PHASE_A = 0,
    PHASE_B = 1,
    PHASE_C = 2,
} motorPhase_e;

typedef struct motor_pwm_Config_s {
    uint16_t mot_pwm_hz;
    uint16_t mot_pwm_dt_ns;
} motor_pwm_Config_t;

struct motor_pwm_commutation_step
{
	motorPhase_e positive;
	motorPhase_e negative;
	motorPhase_e floating;
};

enum motor_pwm_phase_manip
{
    MOTOR_PWM_MANIP_LOW,
    MOTOR_PWM_MANIP_HIGH,
    MOTOR_PWM_MANIP_FLOATING,
    MOTOR_PWM_MANIP_HALF,
    MOTOR_PWM_MANIP_END_
};

void motor_pwm_init(void);
void motor_pwm_set_freewheeling(void);
void motor_pwm_set_break(void);
int motor_pwm_compute_pwm_val(float duty_cycle);
void motor_pwm_set_step_and_pwm(const struct motor_pwm_commutation_step* step, int pwm_val);
void motor_pwm_beep(int frequency, int duration_msec);

/**
 * Direct phase control - for self-testing
 */
void motor_pwm_manip(const enum motor_pwm_phase_manip command[MOTOR_NUM_PHASES]);
