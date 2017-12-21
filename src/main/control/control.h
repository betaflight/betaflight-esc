#pragma once

typedef struct motor_Config_s {
    double mot_v_min;// 正常模式下的最小MOS管输入电压 Min inverter source voltage for normal mode
    double mot_v_spinup;// 开始转动时的初始电压 Initial voltage of spin-up voltage ramp
    double mot_spup_vramp_t;// Duration of spin-up voltage ramp
    double mot_dc_accel;// Duty cycle ramp bypass threshold
    double mot_dc_slope;// Duty cycle ramp steepness

    uint16_t mot_num_poles;// Number of magnetic poles on the rotor
    uint16_t ctl_dir;// Direction of rotation: 0 - forward, 1 - reverse

    uint16_t mot_rpm_min;// Minimum setpoint for RPM control loop

    double mot_i_max;// Maximum inverter current
    double mot_i_max_p;// Current limiter proportional gain
    double mot_lpf_freq;// Current/voltage low-pass filter corner frequency

    uint16_t mot_stop_thres;//  lock-up threshold. If the rotor has stalled this number of times in a row,the controller will lock up until a zero setpoint is received.
} motor_Config_t;

/**
 * 电机控制模式
 */
enum motor_control_mode
{
    MOTOR_CONTROL_MODE_OPENLOOP,// 开环模式:电机启动使用开环控制模式
    MOTOR_CONTROL_MODE_RPM// 转速模式:电机正常运转后使用闭环转速控制模式
};

/**
 * 电机限制条件
 */
enum motor_limit_mask
{
    MOTOR_LIMIT_RPM = 1,// 转速限制
    MOTOR_LIMIT_CURRENT = 2,// 电流限制
    MOTOR_LIMIT_ACCEL = 4// 加速度限制
};

/**
 * 转向改变模块检测状态
 */
enum motor_forced_rotation_direction
{
    MOTOR_FORCED_ROTATION_NONE,		// 无操作
    MOTOR_FORCED_ROTATION_FORWARD,	// 前转
    MOTOR_FORCED_ROTATION_REVERSE,	// 后转
};

// 电机初始化
void motor_init(void);

/**
 * Sets the duty cycle. Control mode will be OPENLOOP.
 * 开环调速:通过设置PWM值从而改变占空比,达到电压调速
 *
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * 如果没有新的设定值,则电机将会在该时间值内保持该速度进行转动
 *
 * @param [in] dc     Duty cycle [0.0; 1.0] 占空比
 * @param [in] ttl_ms TTL in milliseconds 生存时间(Time To Live)
 */
void motor_set_duty_cycle(float dc, int ttl_ms);

/**
 * Sets the RPM setpoint. Control mode will be RPM.
 * 设置电机转速
 *
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * 如果没有新的设定值,则电机将会在该时间值内保持该速度进行转动
 * @param [in] rpm    RPM setpoint
 * @param [in] ttl_ms TTL in milliseconds 生存时间(Time To Live)
 */
void motor_set_rpm(unsigned rpm, int ttl_ms);

/**
 * Returns current duty cycle.
 * 获取电机控制的占空比(0.0 - 1.0)
 */
float motor_get_duty_cycle(void);

/**
 * Returns current RPM.
 * 获取电机转速(转/分)
 */
unsigned motor_get_rpm(void);

/**
 * 停转电机
 */
void motor_stop(void);

/**
 * 获取电机控制模式
 * MOTOR_CONTROL_MODE_OPENLOOP,// 开环模式:电机启动使用开环控制模式
 * MOTOR_CONTROL_MODE_RPM // 转速模式:电机正常运转后使用闭环转速控制模式
 */
enum motor_control_mode motor_get_control_mode(void);

/**
 * Returns the motor state.
 * 电机是否正常转动
 * @return True if the motor is running; false if starting or not running.
 */
bool motor_is_running(void);

/**
 * Returns the motor state.
 * 电机是否状态空闲
 * @return True if the motor is idle; false if starting or running.
 */
bool motor_is_idle(void);

/**
 * Returns true if the motor controller has given up trying to start.
 * 电机是否堵转
 * @return True if the motor controller is locked.
 */
bool motor_is_blocked(void);

/**
 * Returns the bitmask of currently active limits.
 * 获取限制标志位(电流限制 or 转速限制 or 加速度限制)
 */
int motor_get_limit_mask(void);

/**
 * Returns filtered input voltage and current.
 * 获取滤波后的电源和电流
 * @param [out] out_voltage Volts
 * @param [out] out_current Amperes
 */
void motor_get_input_voltage_current(float* out_voltage, float* out_current);

// 测试电机硬件
int motor_test_hardware(void);
// 测试电机转动
int motor_test_motor(void);
// 电机蜂鸣器响起
void motor_beep(int frequency, int duration_msec);
// 电机进入紧急情况
void motor_emergency(void);
