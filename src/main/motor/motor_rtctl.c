#include "include.h"

// AC -> AB -> CB -> CA -> BA -> BC
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_FORWARD[MOTOR_NUM_COMMUTATION_STEPS] = {
	{PHASE_B, PHASE_A, PHASE_C},
	{PHASE_B, PHASE_C, PHASE_A},
	{PHASE_A, PHASE_C, PHASE_B},
	{PHASE_A, PHASE_B, PHASE_C},
	{PHASE_C, PHASE_B, PHASE_A},
	{PHASE_C, PHASE_A, PHASE_B}
};

// CA -> CB -> AB -> AC -> BC -> BA
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_REVERSE[MOTOR_NUM_COMMUTATION_STEPS] = {
    {PHASE_C, PHASE_A, PHASE_B},
    {PHASE_C, PHASE_B, PHASE_A},
    {PHASE_A, PHASE_B, PHASE_C},
    {PHASE_A, PHASE_C, PHASE_B},
    {PHASE_B, PHASE_C, PHASE_A},
    {PHASE_B, PHASE_A, PHASE_C}
};
