#pragma once

#define AFetHiOff()			LL_GPIO_ResetOutputPin(A_FET_HI_GPIO, A_FET_HI_PIN);
#define AFetHiOn()			LL_GPIO_SetOutputPin(A_FET_HI_GPIO, A_FET_HI_PIN);

#define BFetHiOff()			LL_GPIO_ResetOutputPin(B_FET_HI_GPIO, B_FET_HI_PIN);
#define BFetHiOn()			LL_GPIO_SetOutputPin(B_FET_HI_GPIO, B_FET_HI_PIN);

#define CFetHiOff()			LL_GPIO_ResetOutputPin(C_FET_HI_GPIO, C_FET_HI_PIN);
#define CFetHiOn()			LL_GPIO_SetOutputPin(C_FET_HI_GPIO, C_FET_HI_PIN);

#define AFetLoOff()			LL_GPIO_ResetOutputPin(A_FET_LO_GPIO, A_FET_LO_PIN);
#define AFetLoOn()			LL_GPIO_SetOutputPin(A_FET_LO_GPIO, A_FET_LO_PIN);

#define BFetLoOff()			LL_GPIO_ResetOutputPin(B_FET_LO_GPIO, B_FET_LO_PIN);
#define BFetLoOn()			LL_GPIO_SetOutputPin(B_FET_LO_GPIO, B_FET_LO_PIN);

#define CFetLoOff()			LL_GPIO_ResetOutputPin(C_FET_LO_GPIO, C_FET_LO_PIN);
#define CFetLoOn()			LL_GPIO_SetOutputPin(C_FET_LO_GPIO, C_FET_LO_PIN);

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
