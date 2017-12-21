#pragma once

typedef struct motor_rtctl_Config_s {
    // Timing advance settings
    // 最小的换向提前角
    uint16_t mot_tim_adv_min;// the minimum commutation advance angle, in electrical rotor position degrees.
    // 最大的换向提前角
    uint16_t mot_tim_adv_max;// the maximum commutation advance angle, in electrical rotor position degrees.
    // 当换相时间小于该值时,以该值进行换相
    uint16_t mot_tim_cp_max;// when the current commutation period is shorter than this value, the maximum advance angle will be used. The units are microseconds.
    // 当换相时间大于该值时,以该值进行换相
    uint16_t mot_tim_cp_min;// when the current commutation period is longer than this value, the minimum advance angle will be used. The units are microseconds.

    // Most important parameters
    uint16_t mot_blank_usec;// the duration of the blanking time after the commutation, in microseconds.
    uint16_t mot_zc_fails_max;// zero crossing detection failure threshold. If the failure counter exceeds this threshold, the rotor will be considered stalled.
    uint16_t mot_comm_per_max;// maximum commutationperiod,inmicroseconds. The estimated commutation period will be constrained by the controller to not exceed this value

    // Spinup settings
    uint32_t mot_spup_st_cp;// the maximum commutation period for the spin-up mode, in microseconds. This is also the initial commutation period.
    uint16_t mot_spup_to_ms;// the overall spin-up timeout, in milliseconds. If a spin-up attempt could not be completed in this amount of time, the rotor will be considered stalled.
    uint16_t mot_spup_blnk_pm;// the duration of the extended blanking time during spin-up in permill(i.e. one-thousands; 10 permill = 1%) of the current commutation period.
} motor_rtctl_Config_t;

/**
 * High-level motor control state.表示电机控制状态量
 */
enum motor_rtctl_state
{
    /**
     * Unpowered.未通电
     * The motor can be rotated by an external force.电机可以自由旋转
     * Next state: starting.
     */
    MOTOR_RTCTL_STATE_IDLE,// 空闲中

    /**
     * Motor is attempting to start.电机尝试启动
     * Next state: running on success, idle on failure.
     */
    MOTOR_RTCTL_STATE_STARTING,// 开始启动

    /**
     * Motor is running in normal mode.电机正常转动中
     * Next state: idle.
     */
    MOTOR_RTCTL_STATE_RUNNING// 转动正常运行中
};

/**
 * 初始化硬件与控制逻辑 Initialize the hardware and control logic
 */
void motor_rtctl_init(void);

/**
 * Start the motor.启动电机
 * @param [in] initial_duty_cycle       Initial PWM duty cycle, (0; 1]
 * @param [in] target_duty_cycle        PWM duty cycle that will be applied once the motor has started, (0; 1]
 * @param [in] spinup_ramp_duration     Duration of the duty cycle ramp in seconds
 * @param [in] reverse                  Spin direction
 * @param [in] num_prior_attempts       Number of attempts performed before this one, used to switch spinup settings
 */
void motor_rtctl_start(float initial_duty_cycle, float target_duty_cycle, float spinup_ramp_duration, bool reverse, unsigned num_prior_attempts);

/**
 * Engage freewheeling.紧急停车
 */
void motor_rtctl_stop(void);

/**
 * Configure PWM duty cycle
 * @param [in] duty_cycle PWM duty cycle [-1; 1], negative - braking, positive - forward
 */
void motor_rtctl_set_duty_cycle(float duty_cycle);

/**
 * Returns motor state.
 */
enum motor_rtctl_state motor_rtctl_get_state(void);

/**
 * Make noise.
 * Will work only if the motor is not running.
 */
void motor_rtctl_beep(int frequency, int duration_msec);

/**
 * Returns the commutation period if running, 0 if not.获取换相时间
 */
uint32_t motor_rtctl_get_comm_period_hnsec(void);

/**
 * Emergency deactivation.
 * Can be executed from any context.
 */
void motor_rtctl_emergency(void);

/**
 * Returns input voltage and current.
 * If the motor is running, sampling is synchronized with ZC and lowpass filters are applied.
 * If the motor is not running, immediate values are taken.
 * Higher-order low pass filter should be applied to these values anyway.
 * @param [out] out_voltage Volts
 * @param [out] out_current Amperes
 */
void motor_rtctl_get_input_voltage_current(float* out_voltage, float* out_current);

/**
 * Minimum safe comm period. Depends on PWM frequency.
 */
uint32_t motor_rtctl_get_min_comm_period_hnsec(void);

/**
 * Current timestamp in hectonanoseconds since boot; never overflows.
 */
#define motor_rtctl_timestamp_hnsec() motor_timer_hnsec()
