#pragma once

typedef struct motor_Config_s {
    double mot_v_min;           // Min inverter source voltage for normal mode
    double mot_v_spinup;        // Initial voltage of spin-up voltage ramp
    double mot_spup_vramp_t;    // Duration of spin-up voltage ramp
    double mot_dc_accel;        // Duty cycle ramp bypass threshold
    double mot_dc_slope;        // Duty cycle ramp steepness

    uint16_t mot_num_poles;     // Number of magnetic poles on the rotor
    uint16_t ctl_dir;           // Direction of rotation: 0 - forward, 1 - reverse

    uint16_t mot_rpm_min;       // Minimum setpoint for RPM control loop

    double mot_i_max;           // Maximum inverter current
    double mot_i_max_p;         // Current limiter proportional gain
    double mot_lpf_freq;        // Current/voltage low-pass filter corner frequency

    uint16_t mot_stop_thres;    //  lock-up threshold. If the rotor has stalled this number of times in a row,the controller will lock up until a zero setpoint is received.
} motor_Config_t;

enum motor_control_mode
{
    MOTOR_CONTROL_MODE_OPENLOOP,
    MOTOR_CONTROL_MODE_RPM
};

enum motor_limit_mask
{
    MOTOR_LIMIT_RPM = 1,
    MOTOR_LIMIT_CURRENT = 2,
    MOTOR_LIMIT_ACCEL = 4
};

enum motor_forced_rotation_direction
{
    MOTOR_FORCED_ROTATION_NONE,
    MOTOR_FORCED_ROTATION_FORWARD,
    MOTOR_FORCED_ROTATION_REVERSE,
};

void motor_init(void);

/**
 * Sets the duty cycle. Control mode will be OPENLOOP.
 * 
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 *
 * @param [in] dc     Duty cycle [0.0; 1.0] 
 * @param [in] ttl_ms TTL in milliseconds (Time To Live)
 */
void motor_set_duty_cycle(float dc, int ttl_ms);

/**
 * Sets the RPM setpoint. Control mode will be RPM.
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * 
 * @param [in] rpm    RPM setpoint
 * @param [in] ttl_ms TTL in milliseconds (Time To Live)
 */
void motor_set_rpm(unsigned rpm, int ttl_ms);
float motor_get_duty_cycle(void);
unsigned motor_get_rpm(void);
void motor_stop(void);
enum motor_control_mode motor_get_control_mode(void);
bool motor_is_running(void);
bool motor_is_idle(void);
bool motor_is_blocked(void);
int motor_get_limit_mask(void);
void motor_get_input_voltage_current(float* out_voltage, float* out_current);
int motor_test_hardware(void);
int motor_test_motor(void);
void motor_beep(int frequency, int duration_msec);
void motor_emergency(void);
