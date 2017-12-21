#include "include.h"

#define IDLE_CONTROL_PERIOD_MSEC  10
#define WATCHDOG_TIMEOUT_SEC     5

#define MAX_BEEP_DURATION_MSEC    1000

#define MIN_VALID_INPUT_VOLTAGE 4.0
#define MAX_VALID_INPUT_VOLTAGE 40.0

/*
 * TODO: Current implementation is a mess.
 * Maybe it should be redesigned from scratch as a nice FSM.
 */
static struct state
{
    enum motor_control_mode mode;// 电机控制模式(开环调速 or 闭环调速)
    int limit_mask;// 限制条件

    float dc_actual;// 实际占空比(当前PWM输出占空比)
    float dc_openloop_setpoint;// 期望占空比

    unsigned rpm_setpoint;// 期望转速值

    int setpoint_ttl_ms;// 期望占空比的持续时间
    int num_unexpected_stops;// 意外停转(堵转)次数

    float input_voltage;// 输入电压
    float input_current;// 输入电流
    float input_curent_offset;// 电流偏差

    float filtered_input_current_for_limiter;// 额定电流值

    enum motor_rtctl_state rtctl_state;// 电机实时控制模块状态

    int beep_frequency;// 蜂鸣器发生频率
    int beep_duration_msec;// 蜂鸣持续时间
} _state;

/**
 * 参数值
 */
static struct params
{
    float dc_min_voltage;// 占空比对应输入最小电压
    float dc_spinup_voltage;// 占空比对应电机启动电压
    float spinup_voltage_ramp_duration;// 电机启动时间
    float dc_step_max;// 占空比最大梯级
    float dc_slope;// 占空比斜率

    int poles;// 电机极数
    bool reverse;// 是否反转

    uint32_t comm_period_limit;// 限制的换相时间
    unsigned rpm_max;// 最大转速
    unsigned rpm_min;// 最小转速

    float current_limit;// 电流限制
    float current_limit_p;// 电流限制P控制器的比例项

    float voltage_current_lowpass_tau;// 电压电流低通滤波器的系数
    int num_unexpected_stops_to_latch;// 意外停转次数大于该值时判定为电机堵转
} _params;

/**
 * 通过换相时间计算当前电机的转速
 * erpm_to_comm_period = lambda erpm: ((10000000 * 60) / erpm) / 6
 * comm_period_to_erpm = lambda cp: (10000000 * 60) / (cp * 6)
 */
static unsigned comm_period_to_rpm(uint32_t comm_period_hnsec)
{
    // 检查电机磁极数不为零
    assert_param(_params.poles > 0);

    // 检查换相时间是否为零,防止程序执行除零运算
    if (comm_period_hnsec == 0) {
        return 0;
    }

    const uint32_t x = (120ULL * (uint64_t)HNSEC_PER_SEC) / (_params.poles * 6);
    return x / comm_period_hnsec;
}

/**
 * 参数配置
 */
static void init_parameters(void)
{
    _params.dc_min_voltage    = motor_Config()->mot_v_min;// MOS管最小导通电压
    _params.dc_spinup_voltage = motor_Config()->mot_v_spinup;// MOS管的启动电压
    _params.spinup_voltage_ramp_duration = motor_Config()->mot_spup_vramp_t;// 启动时间
    _params.dc_step_max    = motor_Config()->mot_dc_accel;
    _params.dc_slope       = motor_Config()->mot_dc_slope;

    _params.poles = motor_Config()->mot_num_poles;
    _params.reverse = motor_Config()->ctl_dir;

    _params.comm_period_limit = motor_rtctl_get_min_comm_period_hnsec();
    _params.rpm_max = comm_period_to_rpm(_params.comm_period_limit);
    _params.rpm_min = motor_Config()->mot_rpm_min;

    _params.current_limit = motor_Config()->mot_i_max;
    _params.current_limit_p = motor_Config()->mot_i_max_p;

    _params.voltage_current_lowpass_tau = 1.0f / motor_Config()->mot_lpf_freq;
    _params.num_unexpected_stops_to_latch = motor_Config()->mot_stop_thres;

    printf("Motor: RPM range: [%u, %u]; poles: %i\n", _params.rpm_min, _params.rpm_max, _params.poles);
}

/**
 * 蜂鸣器响
 */
static void poll_beep(void)
{
    // 判断参数的有效性并且电机控制在空闲状态
    const bool do_beep =
        (_state.beep_frequency > 0) &&
        (_state.beep_duration_msec > 0) &&
        (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE);

    // 结果为真, 则使电机开始蜂鸣
    if (do_beep) {
        if (_state.beep_duration_msec > MAX_BEEP_DURATION_MSEC) {
            _state.beep_duration_msec = MAX_BEEP_DURATION_MSEC;
        }
        motor_rtctl_beep(_state.beep_frequency, _state.beep_duration_msec);
    }

    // 清零变量, 只响一遍
    _state.beep_frequency = 0;
    _state.beep_duration_msec = 0;
}

/**
 * 一阶低通数字滤波器
 */
static float lowpass(float xold, float xnew, float tau, float dt)
{
    return (dt * xnew + tau * xold) / (dt + tau);
}

/**
 * 初始化滤波器(电压&电流)
 */
static void init_filters(void)
{
    // Assuming that initial current is zero
    motor_rtctl_get_input_voltage_current(&_state.input_voltage, &_state.input_curent_offset);
    _state.input_current = 0.0f;
    _state.filtered_input_current_for_limiter = 0.0f;
}

/**
 * 更新滤波系数与更新电压电流值
 */
static void update_filters(float dt)
{
    // 电压/电流值 变量
    float voltage = 0, current = 0;
    // 电机实时控制模块_获取输入的电压和电流
    motor_rtctl_get_input_voltage_current(&voltage, &current);

    // 如果当前电机控制状态为空闲时, 则可以使用电流计偏差自校准
    if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
        // Current sensor offset calibration, corner frequency is much lower.电流计偏差自校准
        const float offset_tau = _params.voltage_current_lowpass_tau * 100;// 偏差值
        // 计算电流零偏值
        _state.input_curent_offset = lowpass(_state.input_curent_offset, current, offset_tau, dt);
    }

    // 电流值 = 电流值 - 电流零偏值
    current -= _state.input_curent_offset;

    // 对电流值和电压值进行一阶低通滤波操作
    _state.input_voltage = lowpass(_state.input_voltage, voltage, _params.voltage_current_lowpass_tau, dt);
    _state.input_current = lowpass(_state.input_current, current, _params.voltage_current_lowpass_tau, dt);

    // 计算滤波后的电流额定值,电流不允许超过该值
    _state.filtered_input_current_for_limiter =
        lowpass(_state.filtered_input_current_for_limiter, _state.input_current,
                1.0F, dt);
}

/**
 * 电机停转
 */
static void stop(bool expected)
{
    // 电机实时控制模块_让电机停转
    motor_rtctl_stop();

    // 变量清零
    _state.limit_mask = 0;
    _state.dc_actual = 0.0;
    _state.dc_openloop_setpoint = 0.0;
    _state.rpm_setpoint = 0;
    _state.setpoint_ttl_ms = 0;
    _state.filtered_input_current_for_limiter = 0.0;
    _state.rtctl_state = motor_rtctl_get_state();// 获取电机控制模块状态
    if (expected) {
        _state.num_unexpected_stops = 0;
    } else {
        _state.num_unexpected_stops++;
    }

    // 重置转速闭环控制模块
    rpmctl_reset();
}

/**
 * 处理意外停转
 */
static void handle_unexpected_stop(void)
{
    // The motor will not be restarted automatically till the next setpoint update
    stop(false);

    // Usually unexpected stop means that the control is fucked up, so it's good to have some insight
    printf("Motor: Unexpected stop [%i of %i], below is some debug info\n",
           _state.num_unexpected_stops, _params.num_unexpected_stops_to_latch);

    // Wait some more before the possibly immediately following restart to serve other threads
     // delayMs(10);
}

/**
 * 准备启动电机
 */
static void update_control_non_running(void)
{
    // Do not change anything while the motor is starting 如果电机正在启动,则不进行任何干预
    const enum motor_rtctl_state rtctl_state = motor_rtctl_get_state();
    if (rtctl_state == MOTOR_RTCTL_STATE_STARTING) {
        return;
    }

    // Start if necessary 判断电机是否需要启动(开环模式下期望值是否大于零,闭环模式下期望值是否大于零)
    const bool need_start =
        (_state.mode == MOTOR_CONTROL_MODE_OPENLOOP && (_state.dc_openloop_setpoint > 0)) ||
        (_state.mode == MOTOR_CONTROL_MODE_RPM && (_state.rpm_setpoint > 0));

    // 开始启动电机(标志位 & 电机不堵转)
    if (need_start && (_state.num_unexpected_stops < _params.num_unexpected_stops_to_latch)) {
        // 获取启动时的时间戳
        const uint64_t timestamp = motor_rtctl_timestamp_hnsec();

        // 获取当前实际的占空比 = 占空比最小电压值 / 电机输入电压
        _state.dc_actual = _params.dc_min_voltage / _state.input_voltage;

        // 电机实时控制模块启动
        motor_rtctl_start(_params.dc_spinup_voltage / _state.input_voltage,
                          _params.dc_min_voltage    / _state.input_voltage,
                          _params.spinup_voltage_ramp_duration,
                          _params.reverse, _state.num_unexpected_stops);

        // 更新电机实时控制模块状态
        _state.rtctl_state = motor_rtctl_get_state();

        // This HACK prevents the setpoint TTL expiration in case of long startup 累加持续时间
        _state.setpoint_ttl_ms += (motor_rtctl_timestamp_hnsec() - timestamp) / HNSEC_PER_MSEC;

        // 处理意外停转
        if (_state.rtctl_state == MOTOR_RTCTL_STATE_IDLE) {
            handle_unexpected_stop();
        }
    }
}

/**
 * 更新开环控制(换相时间)
 */
static float update_control_open_loop(uint32_t comm_period)
{
    // 占空比最小值 = 最小电压值 / 电机输入的电压值
    const float min_dc = _params.dc_min_voltage / _state.input_voltage;

    // 参数有效性判断
    if (_state.dc_openloop_setpoint <= 0) {
        return nan("");
    }
    if (_state.dc_openloop_setpoint < min_dc) {
        _state.dc_openloop_setpoint = min_dc;
    }

    const uint32_t cp_limit = _params.comm_period_limit * 5 / 4;

    if (comm_period < cp_limit) {
        // Simple P controller 简易的比例控制器
        const float c1 = cp_limit;                    		// Begin limiting at this comm period
        const float c0 = _params.comm_period_limit / 4;         // Reach zero dcyc at this comm period
        const float dc = (comm_period - c0) / (c1 - c0);

        if (dc < _state.dc_openloop_setpoint) {
            _state.limit_mask |= MOTOR_LIMIT_RPM;
            return dc;
        }
    }
    _state.limit_mask &= ~MOTOR_LIMIT_RPM;
    return _state.dc_openloop_setpoint;
}

/**
 * 提高转速(通过PID进行控制)
 */
static float update_control_rpm(uint32_t comm_period, float dt)
{
    // 参数有效性判断
    if (_state.rpm_setpoint <= 0) {
        return nan("");
    }
    if (_state.rpm_setpoint < _params.rpm_min) {
        _state.rpm_setpoint = _params.rpm_min;
    }

    const struct rpmctl_input input = {
        _state.limit_mask,// 限制条件
        dt,// 增量时间
        (float)comm_period_to_rpm(comm_period),// 当前值
        _state.rpm_setpoint// 期望值
    };
    return rpmctl_update(&input);
}

/**
 * 电机限流
 */
static float update_control_current_limit(float new_duty_cycle)
{
    // 是否过流
    const bool overcurrent = _state.filtered_input_current_for_limiter > _params.current_limit;
    // 是否刹车中,刹车瞬间有可能造成电流突变
    const bool braking = _state.dc_actual <= 0.0f || new_duty_cycle <= 0.0f;

    // 如果过流且不在刹车状态,则采取相应的措施
    if (overcurrent && !braking) {
        // 误差 = 额定值 - 当前值
        const float error = _state.filtered_input_current_for_limiter - _params.current_limit;

        // 误差比例项
        const float comp = error * _params.current_limit_p;
        assert_param(comp >= 0.0f);

        // 计算最小的占空比 = 最小电压 / 输入电压值
        const float min_dc = _params.dc_min_voltage / _state.input_voltage;
        // 计算新的占空比 = 当前占空比 - 误差比例项
        new_duty_cycle -= comp * _state.dc_actual;
        // 控制器限幅
        if (new_duty_cycle < min_dc) {
            new_duty_cycle = min_dc;
        }
        // 更新标志位
        _state.limit_mask |= MOTOR_LIMIT_CURRENT;
    } else {
        _state.limit_mask &= ~MOTOR_LIMIT_CURRENT;
    }

    // 返回新的占空比
    return new_duty_cycle;
}

/**
 * 更新控制占空比斜率
 */
static float update_control_dc_slope(float new_duty_cycle, float dt)
{
    const float dc_step_max = (fabsf(new_duty_cycle) + fabsf(_state.dc_actual)) * 0.5f * _params.dc_step_max;
    // 判断新的占空比 与 当前占空比差值 是否大于限定最大值
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

/**
 * 更新控制器
 */
static void update_control(uint32_t comm_period, float dt)
{
    /*
     * Start/stop management 获取电机实时控制模块的状态值
     */
    const enum motor_rtctl_state new_rtctl_state = motor_rtctl_get_state();

    const bool just_stopped =
        new_rtctl_state == MOTOR_RTCTL_STATE_IDLE &&
        _state.rtctl_state != MOTOR_RTCTL_STATE_IDLE;
    if (just_stopped) {
        handle_unexpected_stop();
    }

    _state.rtctl_state = new_rtctl_state;
    if (comm_period == 0 || _state.rtctl_state != MOTOR_RTCTL_STATE_RUNNING) {
        update_control_non_running();// 准备启动电机
        return;
    }

    /*
     * Primary control logic; can return NAN to stop the motor 根据不同的模式进行更新电机控制手段
     */
    float new_duty_cycle = nan("");
    if (_state.mode == MOTOR_CONTROL_MODE_OPENLOOP) {
        new_duty_cycle = update_control_open_loop(comm_period);// 开环调速控制
    } else if (_state.mode == MOTOR_CONTROL_MODE_RPM) {
        new_duty_cycle = update_control_rpm(comm_period, dt);// 闭环调速控制
    }

    if (!isfinite(new_duty_cycle)) {
        stop(true);
        return;
    }

    /*
     * Limiters 占空比限值器
     */
    new_duty_cycle = update_control_current_limit(new_duty_cycle);// 电流限制
    new_duty_cycle = update_control_dc_slope(new_duty_cycle, dt);// 斜率限制

    /*
     * Update 更新电机控制
     */
    _state.dc_actual = new_duty_cycle;// 将处理后的占空比赋值到当前占空比值
    motor_rtctl_set_duty_cycle(_state.dc_actual);// 电机输出PWM值重新设定
}

/**
 * 更新期望值的生命周期
 */
static void update_setpoint_ttl(int dt_ms)
{
    // 获取当前电机状态
    const enum motor_rtctl_state rtctl_state = motor_rtctl_get_state();
    // 参数有限性判断
    if (_state.setpoint_ttl_ms <= 0 || rtctl_state != MOTOR_RTCTL_STATE_RUNNING) {
        return;
    }

    // 递减ttl时间
    _state.setpoint_ttl_ms -= dt_ms;
    if (_state.setpoint_ttl_ms <= 0) {
        stop(true);
        printf("Motor: Setpoint TTL expired, stop\n");
    }
}

/*
 * 电机控制线程
 */
static void control_thread(void *pvParameters)
{
    uint64_t timestamp_hnsec = motor_rtctl_timestamp_hnsec();

    while (1) {
        /*
         * Control loop period adapts to comm period.
         */
        // 获取电机平均换相时间
        const uint32_t comm_period = motor_rtctl_get_comm_period_hnsec();

        unsigned control_period_ms = IDLE_CONTROL_PERIOD_MSEC;
        if (comm_period > 0) {
            // 计算控制间隔
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
        // 等待获取任务通知值
        // ulTaskNotifyTake(pdTRUE, (TickType_t)pdMS_TO_TICKS(control_period_ms));// 阻塞时间

        // 提取信号量


        // 获取当前时间戳
        const uint64_t new_timestamp_hnsec = motor_rtctl_timestamp_hnsec();
        // 计算增量时间(亚微秒)
        const uint32_t dt_hnsec = new_timestamp_hnsec - timestamp_hnsec;
        // 计算增量时间
        const float dt = dt_hnsec / (float)HNSEC_PER_SEC;
        // 更新时间戳,用于下次计算增量时间
        timestamp_hnsec = new_timestamp_hnsec;

        // 更新电流电压滤波器
        update_filters(dt);
        // 更新期望值的持续时间
        update_setpoint_ttl(dt_hnsec / HNSEC_PER_MSEC);
        update_control(comm_period, dt);

        // 检查蜂鸣器是否需要发声
        poll_beep();

        // 对外释放信号量


        // watchdog_feed();
    }
}

/**
 * 电机初始化
 */
void motor_init(void)
{
    // 创建看门狗(10S)
    // watchdog_init(WATCHDOG_TIMEOUT_SEC);

    // 初始化电机实时控制模块(初始化定时器, 初始化ADC检测)
    motor_rtctl_init();

    // 配置电机参数
    init_parameters();

    // 初始化滤波器
    init_filters();
    if (_state.input_voltage < MIN_VALID_INPUT_VOLTAGE || _state.input_voltage > MAX_VALID_INPUT_VOLTAGE) {
        printf("Motor: Invalid input voltage: %f\n", _state.input_voltage);
        // return -1;
    }

    // 转速控制模块初始化(每分钟转数 Revolutions Per Minute)
    rpmctl_init();

    // 电机实时控制模块_让电机停转
    motor_rtctl_stop();

    // 创建互斥信号量
    // _mutex = xSemaphoreCreateMutex();

    // 任务创建
    // xTaskCreate(control_thread, "motors", 4 * configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, &motor_control_thread);
}

/**
 * 停转电机
 */
void motor_stop(void)
{

    stop(true);

}

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
void motor_set_duty_cycle(float dc, int ttl_ms)
{


    // 判断当前模式是否为开环控制模式
    if (_state.mode != MOTOR_CONTROL_MODE_OPENLOOP) {
        // 调速时,设置为开环控制模式
        _state.mode = MOTOR_CONTROL_MODE_OPENLOOP;
        _state.limit_mask = 0;
    }

    // 参数限幅
    if (dc < 0.0) {
        dc = 0.0;
    }
    if (dc > 1.0) {
        dc = 1.0;
    }
    // 设定开环控制时占空比目标值
    _state.dc_openloop_setpoint = dc;
    _state.setpoint_ttl_ms = ttl_ms;

    if (dc == 0.0) {
        _state.num_unexpected_stops = 0;
    }



    // Wake the control thread to process the new setpoint immediately 通知控制线程更新
    // xTaskNotifyGive(motor_control_thread);
}

/**
 * Sets the RPM setpoint. Control mode will be RPM.
 * 闭环调速:设置电机转速
 *
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * 如果没有新的设定值,则电机将会在该时间值内保持该速度进行转动
 * @param [in] rpm    RPM setpoint
 * @param [in] ttl_ms TTL in milliseconds 生存时间(Time To Live)
 */
void motor_set_rpm(unsigned rpm, int ttl_ms)
{


    // 判断当前模式是否为闭环转速控制模式
    if (_state.mode != MOTOR_CONTROL_MODE_RPM) {
        // 调速时,设置为闭环转速控制模式
        _state.mode = MOTOR_CONTROL_MODE_RPM;
        _state.limit_mask = 0;
    }

    // 判断参数有效性,不允许超过最大电机转速
    if (rpm > _params.rpm_max) {
        rpm = _params.rpm_max;
    }
    // 目标值
    _state.rpm_setpoint = rpm;
    _state.setpoint_ttl_ms = ttl_ms;

    if (rpm == 0) {
        _state.num_unexpected_stops = 0;
    }



    // Wake the control thread to process the new setpoint immediately
    // xTaskNotifyGive(motor_control_thread);
}

/**
 * 获取电机控制的占空比(0.0 - 1.0)
 */
float motor_get_duty_cycle(void)
{

    float ret = _state.dc_actual;

    return ret;
}

/**
 * 获取电机转速(转/分)
 */
unsigned motor_get_rpm(void)
{

    // 获取换相时间
    uint32_t cp = motor_rtctl_get_comm_period_hnsec();
    // 通过换相时间计算当前电机的转速
    unsigned rpm = (cp > 0) ? comm_period_to_rpm(cp) : 0;

    return rpm;
}

/**
 * 获取电机控制模式
 * MOTOR_CONTROL_MODE_OPENLOOP,// 开环模式:电机启动使用开环控制模式
 * MOTOR_CONTROL_MODE_RPM // 转速模式:电机正常运转后使用闭环转速控制模式
 */
enum motor_control_mode motor_get_control_mode(void)
{

    enum motor_control_mode ret = _state.mode;

    return ret;
}

/**
 * 电机是否正常转动
 */
bool motor_is_running(void)
{

    bool ret = motor_rtctl_get_state() == MOTOR_RTCTL_STATE_RUNNING;

    return ret;
}

/**
 * 电机是否状态空闲
 */
bool motor_is_idle(void)
{

    bool ret = motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE;

    return ret;
}

/**
 * 电机是否堵转(发生意外的停止次数)
 */
bool motor_is_blocked(void)
{

    bool ret = _state.num_unexpected_stops >= _params.num_unexpected_stops_to_latch;

    return ret;
}

/**
 * 获取限制标志位(电流限制 or 转速限制 or 加速度限制)
 */
int motor_get_limit_mask(void)
{

    int ret = _state.limit_mask;

    return ret;
}

/**
 * 获取电机输入的电压和电流
 */
void motor_get_input_voltage_current(float* out_voltage, float* out_current)
{


    if (out_voltage) {
        *out_voltage = _state.input_voltage;
    }
    if (out_current) {
        *out_current = _state.input_current;
    }

}

/**
 * 测试电机硬件
 */
int motor_test_hardware(void)
{


    int res = motor_rtctl_test_hardware();
    if (res > 0) { // Try harder in case of failure
        res = motor_rtctl_test_hardware();
    }


    return res;
}

/**
 * 测试电机转动
 */
int motor_test_motor(void)
{

    const int res = motor_rtctl_test_motor();

    return res;
}

/**
 * 电机蜂鸣器响起
 */
void motor_beep(int frequency, int duration_msec)
{


    // 判断电机是否空闲状态
    if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
        // 赋值蜂鸣频率
        _state.beep_frequency = frequency;
        // 赋值蜂鸣时间
        _state.beep_duration_msec = duration_msec;
        // Wake the control thread to process the new setpoint immediately
        // xTaskNotifyGive(motor_control_thread);
    }


}

/**
 * 电机进入紧急情况
 */
void motor_emergency(void)
{
    motor_rtctl_emergency();
}
