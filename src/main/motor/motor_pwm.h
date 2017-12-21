#pragma once

typedef struct motor_pwm_Config_s {
    uint16_t mot_pwm_hz;// PWM carrier frequency, in hertz.电机PWM频率值, 16Khz - 48khz
    uint16_t mot_pwm_dt_ns;// PWM switching dead time, in nanoseconds.电机PWM死区值
} motor_pwm_Config_t;

// 转向步骤,有六个步骤(每一步都包含对三相的操作)
struct motor_pwm_commutation_step
{
    int_fast8_t positive;// 正极(正向PWM)
    int_fast8_t negative;// 负极(直通)
    int_fast8_t floating;// 浮动相,用于测量过零点比较
};

// 电机相 操作模式
enum motor_pwm_phase_manip
{
    MOTOR_PWM_MANIP_LOW,// 低
    MOTOR_PWM_MANIP_HIGH,// 高
    MOTOR_PWM_MANIP_FLOATING,// 浮动
    MOTOR_PWM_MANIP_HALF,// 中点
    MOTOR_PWM_MANIP_END_// 结束
};

// 初始化PWM生成器
void motor_pwm_init(void);
// 释放电机,停止输出PWM方波
void motor_pwm_set_freewheeling(void);
// 电机紧急停车
void motor_pwm_set_break(void);
// 根据占空比的比值生成对应的PWM比较值 Duty cycle in [-1; 1]
int motor_pwm_compute_pwm_val(float duty_cycle);
// 调速&换相
void motor_pwm_set_step_and_pwm(const struct motor_pwm_commutation_step* step, int pwm_val);
// 电机蜂鸣
void motor_pwm_beep(int frequency, int duration_msec);

/**
 * Direct phase control - for self-testing
 */
void motor_pwm_manip(const enum motor_pwm_phase_manip command[MOTOR_NUM_PHASES]);
