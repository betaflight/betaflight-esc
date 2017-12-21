#include "include.h"

#define IDLE_CONTROL_PERIOD_MSEC  10
#define WATCHDOG_TIMEOUT_SEC     5

#define MAX_BEEP_DURATION_MSEC    1000

#define MIN_VALID_INPUT_VOLTAGE 4.0f
#define MAX_VALID_INPUT_VOLTAGE 40.0f

/*
 * TODO: Current implementation is a mess.
 * Maybe it should be redesigned from scratch as a nice FSM.
 */
static struct state
{
    enum motor_control_mode mode;
    int limit_mask;

    float dc_actual;
    float dc_openloop_setpoint;

    unsigned rpm_setpoint;

    int setpoint_ttl_ms;
    int num_unexpected_stops;

    float input_voltage;
    float input_current;
    float input_curent_offset;

    float filtered_input_current_for_limiter;

    enum motor_rtctl_state rtctl_state;

    int beep_frequency;
    int beep_duration_msec;
} _state;

static struct params
{
    float dc_min_voltage;
    float dc_spinup_voltage;
    float spinup_voltage_ramp_duration;
    float dc_step_max;
    float dc_slope;

    int poles;
    bool reverse;

    uint32_t comm_period_limit;
    unsigned rpm_max;
    unsigned rpm_min;

    float current_limit;
    float current_limit_p;

    float voltage_current_lowpass_tau;
    int num_unexpected_stops_to_latch;
} _params;

/**
 * erpm_to_comm_period = lambda erpm: ((10000000 * 60) / erpm) / 6
 * comm_period_to_erpm = lambda cp: (10000000 * 60) / (cp * 6)
 */
static unsigned comm_period_to_rpm(uint32_t comm_period_hnsec)
{
    assert_param(_params.poles > 0);
    if (comm_period_hnsec == 0) {
        return 0;
    }

    const uint32_t x = (120ULL * (uint64_t)HNSEC_PER_SEC) / (_params.poles * 6);
    return x / comm_period_hnsec;
}

static void init_parameters(void)
{
    _params.dc_min_voltage    = motor_Config()->mot_v_min;
    _params.dc_spinup_voltage = motor_Config()->mot_v_spinup;
    _params.spinup_voltage_ramp_duration = motor_Config()->mot_spup_vramp_t;
    _params.dc_step_max    = motor_Config()->mot_dc_accel;
    _params.dc_slope       = motor_Config()->mot_dc_slope;

    _params.poles = motor_Config()->mot_num_poles;
    _params.reverse = motor_Config()->ctl_dir;

    _params.comm_period_limit = motor_rtctl_get_min_comm_period_hnsec();
    _params.rpm_max = comm_period_to_rpm(_params.comm_period_limit);
    _params.rpm_min = motor_Config()->mot_rpm_min;

    _params.current_limit = motor_Config()->mot_i_max;
    _params.current_limit_p = motor_Config()->mot_i_max_p;

    _params.voltage_current_lowpass_tau = 1.0 / motor_Config()->mot_lpf_freq;
    _params.num_unexpected_stops_to_latch = motor_Config()->mot_stop_thres;

    printf("Motor: RPM range: [%u, %u]; poles: %i\n", _params.rpm_min, _params.rpm_max, _params.poles);
}

static void poll_beep(void)
{
    const bool do_beep =
        (_state.beep_frequency > 0) &&
        (_state.beep_duration_msec > 0) &&
        (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE);

    if (do_beep) {
        if (_state.beep_duration_msec > MAX_BEEP_DURATION_MSEC) {
            _state.beep_duration_msec = MAX_BEEP_DURATION_MSEC;
        }
        motor_rtctl_beep(_state.beep_frequency, _state.beep_duration_msec);
    }

    _state.beep_frequency = 0;
    _state.beep_duration_msec = 0;
}

static float lowpass(float xold, float xnew, float tau, float dt)
{
    return (dt * xnew + tau * xold) / (dt + tau);
}

static void init_filters(void)
{
    // Assuming that initial current is zero
    motor_rtctl_get_input_voltage_current(&_state.input_voltage, &_state.input_curent_offset);
    _state.input_current = 0.0f;
    _state.filtered_input_current_for_limiter = 0.0f;
}

static void update_filters(float dt)
{
    float voltage = 0, current = 0;
    motor_rtctl_get_input_voltage_current(&voltage, &current);

    if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
        // Current sensor offset calibration, corner frequency is much lower.
        const float offset_tau = _params.voltage_current_lowpass_tau * 100;
        _state.input_curent_offset = lowpass(_state.input_curent_offset, current, offset_tau, dt);
    }

    current -= _state.input_curent_offset;

    _state.input_voltage = lowpass(_state.input_voltage, voltage, _params.voltage_current_lowpass_tau, dt);
    _state.input_current = lowpass(_state.input_current, current, _params.voltage_current_lowpass_tau, dt);

    _state.filtered_input_current_for_limiter =
        lowpass(_state.filtered_input_current_for_limiter, _state.input_current,
                1.0F, dt);
}

static void stop(bool expected)
{
    motor_rtctl_stop();

    _state.limit_mask = 0;
    _state.dc_actual = 0.0;
    _state.dc_openloop_setpoint = 0.0;
    _state.rpm_setpoint = 0;
    _state.setpoint_ttl_ms = 0;
    _state.filtered_input_current_for_limiter = 0.0;
    _state.rtctl_state = motor_rtctl_get_state();
    if (expected) {
        _state.num_unexpected_stops = 0;
    } else {
        _state.num_unexpected_stops++;
    }

    rpmctl_reset();
}

static void handle_unexpected_stop(void)
{
    // The motor will not be restarted automatically till the next setpoint update
    stop(false);

    // Usually unexpected stop means that the control is in error, so it's good to have some insight
    printf("Motor: Unexpected stop [%i of %i], below is some debug info\n",
           _state.num_unexpected_stops, _params.num_unexpected_stops_to_latch);

    // Wait some more before the possibly immediately following restart to serve other threads
    // delayMs(10);
}

static void update_control_non_running(void)
{
    // Do not change anything while the motor is starting 
    const enum motor_rtctl_state rtctl_state = motor_rtctl_get_state();
    if (rtctl_state == MOTOR_RTCTL_STATE_STARTING) {
        return;
    }

    // Start if necessary
    const bool need_start =
        (_state.mode == MOTOR_CONTROL_MODE_OPENLOOP && (_state.dc_openloop_setpoint > 0)) ||
        (_state.mode == MOTOR_CONTROL_MODE_RPM && (_state.rpm_setpoint > 0));

    if (need_start && (_state.num_unexpected_stops < _params.num_unexpected_stops_to_latch)) {

        const uint64_t timestamp = motor_rtctl_timestamp_hnsec();

        _state.dc_actual = _params.dc_min_voltage / _state.input_voltage;

        motor_rtctl_start(_params.dc_spinup_voltage / _state.input_voltage,
                          _params.dc_min_voltage    / _state.input_voltage,
                          _params.spinup_voltage_ramp_duration,
                          _params.reverse, _state.num_unexpected_stops);

        _state.rtctl_state = motor_rtctl_get_state();

        _state.setpoint_ttl_ms += (motor_rtctl_timestamp_hnsec() - timestamp) / HNSEC_PER_MSEC;

        if (_state.rtctl_state == MOTOR_RTCTL_STATE_IDLE) {
            handle_unexpected_stop();
        }
    }
}

static float update_control_open_loop(uint32_t comm_period)
{
    const float min_dc = _params.dc_min_voltage / _state.input_voltage;

    if (_state.dc_openloop_setpoint <= 0) {
        return nan("");
    }
    if (_state.dc_openloop_setpoint < min_dc) {
        _state.dc_openloop_setpoint = min_dc;
    }

    const uint32_t cp_limit = _params.comm_period_limit * 5 / 4;

    if (comm_period < cp_limit) {
        const float c1 = cp_limit;                          // Begin limiting at this comm period
        const float c0 = _params.comm_period_limit / 4;     // Reach zero dcyc at this comm period
        const float dc = (comm_period - c0) / (c1 - c0);

        if (dc < _state.dc_openloop_setpoint) {
            _state.limit_mask |= MOTOR_LIMIT_RPM;
            return dc;
        }
    }
    _state.limit_mask &= ~MOTOR_LIMIT_RPM;
    return _state.dc_openloop_setpoint;
}

static float update_control_rpm(uint32_t comm_period, float dt)
{
    if (_state.rpm_setpoint <= 0) {
        return nan("");
    }
    if (_state.rpm_setpoint < _params.rpm_min) {
        _state.rpm_setpoint = _params.rpm_min;
    }

    const struct rpmctl_input input = {
        _state.limit_mask,
        dt,
        (float)comm_period_to_rpm(comm_period),
        _state.rpm_setpoint
    };
    return rpmctl_update(&input);
}

static float update_control_current_limit(float new_duty_cycle)
{
    const bool overcurrent = _state.filtered_input_current_for_limiter > _params.current_limit;

    const bool braking = _state.dc_actual <= 0.0f || new_duty_cycle <= 0.0f;

    if (overcurrent && !braking) {
        const float error = _state.filtered_input_current_for_limiter - _params.current_limit;

        const float comp = error * _params.current_limit_p;
        assert_param(comp >= 0.0f);

        const float min_dc = _params.dc_min_voltage / _state.input_voltage;

        new_duty_cycle -= comp * _state.dc_actual;

        if (new_duty_cycle < min_dc) {
            new_duty_cycle = min_dc;
        }
        _state.limit_mask |= MOTOR_LIMIT_CURRENT;
    } else {
        _state.limit_mask &= ~MOTOR_LIMIT_CURRENT;
    }

    return new_duty_cycle;
}

static float update_control_dc_slope(float new_duty_cycle, float dt)
{
    const float dc_step_max = (fabsf(new_duty_cycle) + fabsf(_state.dc_actual)) * 0.5f * _params.dc_step_max;
    if (fabsf(new_duty_cycle - _state.dc_actual) > dc_step_max) {
        float step = _params.dc_slope * dt;

        if (step > dc_step_max) {
            step = dc_step_max;
        }
        if (new_duty_cycle < _state.dc_actual) {
            step = -step;
        }
        new_duty_cycle = _state.dc_actual + step;
        _state.limit_mask |= MOTOR_LIMIT_ACCEL;
    } else {
        _state.limit_mask &= ~MOTOR_LIMIT_ACCEL;
    }

    return new_duty_cycle;
}

static void update_control(uint32_t comm_period, float dt)
{
    const enum motor_rtctl_state new_rtctl_state = motor_rtctl_get_state();

    const bool just_stopped =
        new_rtctl_state == MOTOR_RTCTL_STATE_IDLE &&
        _state.rtctl_state != MOTOR_RTCTL_STATE_IDLE;
    if (just_stopped) {
        handle_unexpected_stop();
    }

    _state.rtctl_state = new_rtctl_state;
    if (comm_period == 0 || _state.rtctl_state != MOTOR_RTCTL_STATE_RUNNING) {
        update_control_non_running();
        return;
    }

    float new_duty_cycle = nan("");
    if (_state.mode == MOTOR_CONTROL_MODE_OPENLOOP) {
        new_duty_cycle = update_control_open_loop(comm_period);
    } else if (_state.mode == MOTOR_CONTROL_MODE_RPM) {
        new_duty_cycle = update_control_rpm(comm_period, dt);
    }

    if (!isfinite(new_duty_cycle)) {
        stop(true);
        return;
    }

    new_duty_cycle = update_control_current_limit(new_duty_cycle);
    new_duty_cycle = update_control_dc_slope(new_duty_cycle, dt);

    _state.dc_actual = new_duty_cycle;
    motor_rtctl_set_duty_cycle(_state.dc_actual); 
}

static void update_setpoint_ttl(int dt_ms)
{
    const enum motor_rtctl_state rtctl_state = motor_rtctl_get_state();

    if (_state.setpoint_ttl_ms <= 0 || rtctl_state != MOTOR_RTCTL_STATE_RUNNING) {
        return;
    }

    _state.setpoint_ttl_ms -= dt_ms;
    if (_state.setpoint_ttl_ms <= 0) {
        stop(true);
        printf("Motor: Setpoint TTL expired, stop\n");
    }
}

static void control_thread(void *pvParameters)
{
    UNUSED(pvParameters);

    uint64_t timestamp_hnsec = motor_rtctl_timestamp_hnsec();

    while (1) {
        const uint32_t comm_period = motor_rtctl_get_comm_period_hnsec();

        unsigned control_period_ms = IDLE_CONTROL_PERIOD_MSEC;
        if (comm_period > 0) {
            control_period_ms = comm_period / HNSEC_PER_MSEC;
        }

        if (control_period_ms < 1) {
            control_period_ms = 1;
        } else if (control_period_ms > IDLE_CONTROL_PERIOD_MSEC) {
            control_period_ms = IDLE_CONTROL_PERIOD_MSEC;
        }

        /*
         * The event must be set only when the mutex is unlocked.
         * Otherwise this thread will take control, stumble upon the locked mutex, return the control
         * to the thread that holds the mutex, unlock the mutex, then proceed.
         */
        // ulTaskNotifyTake(pdTRUE, (TickType_t)pdMS_TO_TICKS(control_period_ms));

        const uint64_t new_timestamp_hnsec = motor_rtctl_timestamp_hnsec();

        const uint32_t dt_hnsec = new_timestamp_hnsec - timestamp_hnsec;

        const float dt = dt_hnsec / (float)HNSEC_PER_SEC;

        timestamp_hnsec = new_timestamp_hnsec;

        update_filters(dt);

        update_setpoint_ttl(dt_hnsec / HNSEC_PER_MSEC);
        update_control(comm_period, dt);

        poll_beep();

        // watchdog_feed();
    }
}

void motor_init(void)
{
    // watchdog_init(WATCHDOG_TIMEOUT_SEC);

    motor_rtctl_init();

    init_parameters();

    init_filters();
    if (_state.input_voltage < MIN_VALID_INPUT_VOLTAGE || _state.input_voltage > MAX_VALID_INPUT_VOLTAGE) {
        printf("Motor: Invalid input voltage: %f\n", (double)(_state.input_voltage));
        // return -1;
    }

    rpmctl_init();

    motor_rtctl_stop();

    // _mutex = xSemaphoreCreateMutex();

    // xTaskCreate(control_thread, "motors", 4 * configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, &motor_control_thread);
}

void motor_stop(void)
{
    stop(true);
}

/**
 * Sets the duty cycle. Control mode will be OPENLOOP.
 *
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 *
 * @param [in] dc     Duty cycle [0.0; 1.0] 
 * @param [in] ttl_ms TTL in milliseconds (Time To Live)
 */
void motor_set_duty_cycle(float dc, int ttl_ms)
{

    if (_state.mode != MOTOR_CONTROL_MODE_OPENLOOP) {
        
        _state.mode = MOTOR_CONTROL_MODE_OPENLOOP;
        _state.limit_mask = 0;
    }

    if (dc < 0.0f) {
        dc = 0.0f;
    }
    if (dc > 1.0f) {
        dc = 1.0f;
    }

    _state.dc_openloop_setpoint = dc;
    _state.setpoint_ttl_ms = ttl_ms;

    if (dc == 0.0f) {
        _state.num_unexpected_stops = 0;
    }

    // Wake the control thread to process the new setpoint immediately 通知控制线程更新
    // xTaskNotifyGive(motor_control_thread);
}

/**
 * Sets the RPM setpoint. Control mode will be RPM.
 *
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * @param [in] rpm    RPM setpoint
 * @param [in] ttl_ms TTL in milliseconds (Time To Live)
 */
void motor_set_rpm(unsigned rpm, int ttl_ms)
{

    if (_state.mode != MOTOR_CONTROL_MODE_RPM) {
        _state.mode = MOTOR_CONTROL_MODE_RPM;
        _state.limit_mask = 0;
    }

    if (rpm > _params.rpm_max) {
        rpm = _params.rpm_max;
    }

    _state.rpm_setpoint = rpm;
    _state.setpoint_ttl_ms = ttl_ms;

    if (rpm == 0) {
        _state.num_unexpected_stops = 0;
    }

    // Wake the control thread to process the new setpoint immediately
    // xTaskNotifyGive(motor_control_thread);
}

float motor_get_duty_cycle(void)
{

    float ret = _state.dc_actual;

    return ret;
}

unsigned motor_get_rpm(void)
{

    uint32_t cp = motor_rtctl_get_comm_period_hnsec();
    unsigned rpm = (cp > 0) ? comm_period_to_rpm(cp) : 0;

    return rpm;
}

enum motor_control_mode motor_get_control_mode(void)
{

    enum motor_control_mode ret = _state.mode;

    return ret;
}

bool motor_is_running(void)
{
    bool ret = motor_rtctl_get_state() == MOTOR_RTCTL_STATE_RUNNING;

    return ret;
}

bool motor_is_idle(void)
{
    bool ret = motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE;

    return ret;
}

bool motor_is_blocked(void)
{
    bool ret = _state.num_unexpected_stops >= _params.num_unexpected_stops_to_latch;

    return ret;
}


int motor_get_limit_mask(void)
{
    int ret = _state.limit_mask;

    return ret;
}

void motor_get_input_voltage_current(float* out_voltage, float* out_current)
{
    if (out_voltage) {
        *out_voltage = _state.input_voltage;
    }

    if (out_current) {
        *out_current = _state.input_current;
    }
}

int motor_test_hardware(void)
{
    int res = motor_rtctl_test_hardware();
    if (res > 0) { // Try harder in case of failure
        res = motor_rtctl_test_hardware();
    }

    return res;
}

int motor_test_motor(void)
{

    const int res = motor_rtctl_test_motor();

    return res;
}

void motor_beep(int frequency, int duration_msec)
{
    if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {

        _state.beep_frequency = frequency;

        _state.beep_duration_msec = duration_msec;
        // Wake the control thread to process the new setpoint immediately
        // xTaskNotifyGive(motor_control_thread);
    }
}

void motor_emergency(void)
{
    motor_rtctl_emergency();
}
