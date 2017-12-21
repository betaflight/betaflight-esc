#include "include.h"

/**
 * Computes the timing advance in comm_period units 计算超前进角
 */
#define TIMING_ADVANCE64(comm_period, degrees) \
    (((int64_t)comm_period * (int64_t)degrees) / 64LL)

#define MIN(a, b)                  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                  (((a) > (b)) ? (a) : (b))

/**
 * Commutation tables 换相表格
 * Phase order: Positive, Negative, Floating
 *
 * FORWARD TABLE:
 *   Step  01234501...
 *   BEMF  -+-+-+-+...
 *           __
 *   A     _/  \__/
 *         __    __
 *   B       \__/
 *             __
 *   C     \__/  \_
 *
 * REVERSE TABLE:
 *   Step  01234501...
 *   BEMF  -+-+-+-+...
 *           __
 *   A     _/  \__/
 *             __
 *   B     \__/  \_
 *         __    __
 *   C       \__/
 */
// AC -> AB -> CB -> CA -> BA -> BC
// BL32启动序列: CB -> AC -> BA -> CB -> AC -> BA -> CB -> AC -> BA
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_FORWARD[MOTOR_NUM_COMMUTATION_STEPS] = {
    {1, 0, 2},      // BA通电,B相上管注入PWM,A相下管直通,检测C相过零
    {1, 2, 0},      // BC通电,B相上管注入PWM,C相下管直通,检测A相过零
    {0, 2, 1},      // AC通电,A相上管注入PWM,C相下管直通,检测B相过零
    {0, 1, 2},      // AB通电,A相上管注入PWM,B相下管直通,检测C相过零
    {2, 1, 0},      // CB通电,C相上管注入PWM,B相下管直通,检测A相过零
    {2, 0, 1}       // CA通电,C相上管注入PWM,A相下管直通,检测B相过零
};

// CA -> CB -> AB -> AC -> BC -> BA
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_REVERSE[MOTOR_NUM_COMMUTATION_STEPS] = {
    {2, 0, 1},      // CA通电,C相上管注入PWM,A相下管直通,检测B相过零
    {2, 1, 0},      // CB通电,C相上管注入PWM,B相下管直通,检测A相过零
    {0, 1, 2},      // AB通电,A相上管注入PWM,B相下管直通,检测C相过零
    {0, 2, 1},      // AC通电,A相上管注入PWM,C相下管直通,检测B相过零
    {1, 2, 0},      // BC通电,B相上管注入PWM,C相下管直通,检测A相过零
    {1, 0, 2}       // BA通电,B相上管注入PWM,A相下管直通,检测C相过零
};

/**
 * 标志位
 */
enum flags
{
    FLAG_ACTIVE        = 1,// 电机被激活
    FLAG_SPINUP        = 2,// 电机启动旋转中
    FLAG_SYNC_RECOVERY = 4 // 电机同步恢复中
};

/**
 * 过零点事件检测结果 zero crossing detection
 */
enum zc_detection_result
{
    ZC_NOT_DETECTED,// 未检测到过零点事件
    ZC_DETECTED,	// 已检测到过零点事件
    ZC_FAILED,		// 检测过零点事件失败
    ZC_DESATURATION	// 去饱和 Sapog enters a desaturation mode in order to bring the system back into an observable state.
};

static struct control_state            /// 电机控制状态标志位
{
    unsigned flags;// 标志位
    enum zc_detection_result zc_detection_result;// 过零点事件检测结果

    uint64_t blank_time_deadline;// 空白的时间期限
    uint64_t prev_zc_timestamp;// 上一次的过零事件发生时的时间戳
    uint64_t prev_comm_timestamp;// 上一次换相事件发生时的时间戳

    uint32_t comm_period;// 换相时间(间隔)
    uint32_t averaged_comm_period;// 平均换相时间(间隔)

    int current_comm_step;// 当前换相步(0 1 2 3 4 5)
    const struct motor_pwm_commutation_step* comm_table;// 转向步骤表(正向 or 反向)

    unsigned immediate_zc_failures;// 过零检测事件_总失败次数
    unsigned immediate_zc_detects;// 过零检测事件_总次数
    unsigned immediate_desaturations;// 过零检测事件_总饱和次数

    int pwm_val;// 当前PWM脉宽控制值
    int pwm_val_before_spinup;// 电机启动前的PWM值
    int pwm_val_after_spinup;// 电机启动后的PWM值

    bool spinup_prev_zc_timestamp_set;
    uint64_t spinup_ramp_duration_hnsec;// 电机斜坡加速时间

    uint64_t started_at;// 电机启动对应的时间戳
} _state;

/**
 * 预先计算好的参数值
 */
static struct precomputed_params       /// Parameters are read only
{
    int timing_advance_min_deg64;
    int timing_advance_max_deg64;
    uint32_t max_comm_period_for_max_timing_advance;// 最大换相时间
    uint32_t max_comm_period_for_min_timing_advance;// 最小换相时间

    unsigned zc_failures_max;
    uint32_t comm_period_max;
    int comm_blank_hnsec;

    uint32_t spinup_start_comm_period;
    uint32_t spinup_timeout;
    uint32_t spinup_blanking_time_permil;
} _params;

/**
 * 电机参数配置
 */
static void init_parameters(void)
{
    _params.timing_advance_min_deg64               = motor_rtctl_Config()->mot_tim_adv_min * 64 / 60;// 最小超前进角角度
    _params.timing_advance_max_deg64               = motor_rtctl_Config()->mot_tim_adv_max * 64 / 60;// 最大超前进角角度
    _params.max_comm_period_for_max_timing_advance = motor_rtctl_Config()->mot_tim_cp_max * HNSEC_PER_USEC;
    _params.max_comm_period_for_min_timing_advance = motor_rtctl_Config()->mot_tim_cp_min * HNSEC_PER_USEC;

    _params.zc_failures_max  = motor_rtctl_Config()->mot_zc_fails_max;
    _params.comm_period_max  = motor_rtctl_Config()->mot_comm_per_max * HNSEC_PER_USEC;
    _params.comm_blank_hnsec = motor_rtctl_Config()->mot_blank_usec * HNSEC_PER_USEC;
    _params.spinup_start_comm_period = motor_rtctl_Config()->mot_spup_st_cp * HNSEC_PER_USEC;
    _params.spinup_timeout           = motor_rtctl_Config()->mot_spup_to_ms * HNSEC_PER_MSEC;
    _params.spinup_blanking_time_permil = motor_rtctl_Config()->mot_spup_blnk_pm;

    /*
     * Validation
     */
    if (_params.timing_advance_min_deg64 > _params.timing_advance_max_deg64) {
        _params.timing_advance_min_deg64 = _params.timing_advance_max_deg64;  // Minimizing
    }
    assert_param(_params.timing_advance_min_deg64 <= _params.timing_advance_max_deg64);

    if (_params.max_comm_period_for_max_timing_advance > _params.max_comm_period_for_min_timing_advance) {
        _params.max_comm_period_for_max_timing_advance = _params.max_comm_period_for_min_timing_advance; // Min
    }
    assert_param(_params.max_comm_period_for_max_timing_advance <= _params.max_comm_period_for_min_timing_advance);

    if (_params.spinup_start_comm_period < _params.comm_period_max) {
        _params.spinup_start_comm_period = _params.comm_period_max;
    }
}

static void stop_from_isr(void)
{
    _state.flags = 0;
    motor_timer_cancel();
    motor_pwm_set_freewheeling();
}

static void engage_current_comm_step(void)
{
    motor_pwm_set_step_and_pwm(_state.comm_table + _state.current_comm_step, _state.pwm_val);
}

/**
 * 注册好步
 */
static void register_good_step(void)
{
    if (_state.immediate_zc_failures > 0) {// 过零检测事件_总失败次数
        _state.immediate_zc_detects++;// 过零检测事件_总次数自增一
        if (_state.immediate_zc_detects > MOTOR_NUM_COMMUTATION_STEPS) {
            _state.immediate_zc_failures = 0;
        }
    }

    if (_state.immediate_desaturations > 0) {// 过零检测事件_总饱和次数
        _state.immediate_desaturations--;
    }
}

/**
 * 注册坏步
 */
static void register_bad_step(bool* need_to_stop)
{
    _state.immediate_zc_detects = 0;// 过零检测事件_总次数清零
    _state.immediate_zc_failures++;// 过零检测事件_总失败次数自增一
    *need_to_stop = _state.immediate_zc_failures > _params.zc_failures_max;	// 电机是否需要停转
}

// 获取有效的电机超前进角,无刷电调运用超前角技术实现对电机的弱磁控制,一个满进角为64度
static inline int get_effective_timing_advance_deg64(void)
{
    /*
     * Handling extremes 处理极端情况
     */
    if (_state.comm_period <= _params.max_comm_period_for_max_timing_advance) {
        return _params.timing_advance_max_deg64;// 返回最大进角时的换相时间
    }

    if (_state.comm_period >= _params.max_comm_period_for_min_timing_advance) {
        return _params.timing_advance_min_deg64;// 返回最小进角的换相时间
    }

    if (_state.flags & (FLAG_SPINUP | FLAG_SYNC_RECOVERY)) {
        return _params.timing_advance_min_deg64;// 返回最小进角的换相时间
    }

    /*
     * Linear interpolation 线性插值(角度增量 = 最大进角的换相时间 - 最小进角的换相时间)
     */
    const int tim_delta = _params.timing_advance_max_deg64 - _params.timing_advance_min_deg64;

    //
    const int result = _params.timing_advance_max_deg64 -
                       (tim_delta * (int64_t)(_state.comm_period - _params.max_comm_period_for_max_timing_advance) /
                        (_params.max_comm_period_for_min_timing_advance - _params.max_comm_period_for_max_timing_advance));
    return result;
}

/**
 * 当检测到过零点事件后的处理函数(实际上都是计算换相时间)
 */
static void handle_detected_zc(uint64_t zc_timestamp)
{
    // 同步恢复状态中
    if (_state.flags & FLAG_SYNC_RECOVERY) {
        /*
         * TODO: Proper sync recovery:
         * - Disable PWM
         * - Forget current comm step number - we're out of sync, so the step number is likely to be wrong
         * - Infer the current rotor position from the BEMF signals
         * - Measure the comm period using two subsequent ZC events
         * - Resume PWM
         */
        // 计算换相时间 = 过零事件发生时的时间戳 - 上一次过零事件发生时的时间戳
        _state.comm_period = zc_timestamp - _state.prev_zc_timestamp;
        // 执行调速&换相操作
        engage_current_comm_step();
    } else {
        // 计算过零总耗时 = 上一次过零事件发生时的时间戳 + 换相时间
        const uint64_t predicted_zc_ts = _state.prev_zc_timestamp + _state.comm_period;
        // 补偿时间戳
        zc_timestamp = (predicted_zc_ts + zc_timestamp + 2ULL) / 2ULL;

        // 计算换相时间 = 过零事件发生时的时间戳 - 上一次过零事件发生时的时间戳
        _state.comm_period = zc_timestamp - _state.prev_zc_timestamp;
    }

    // 更新时间戳
    _state.prev_zc_timestamp = zc_timestamp;
    // 更新换相时间(不能超过换相最大时间值)
    _state.comm_period = MIN(_state.comm_period, _params.comm_period_max);
    // 设置零点检测结果为检测到零点
    _state.zc_detection_result = ZC_DETECTED;

    // 计算平均换相间隔
    _state.averaged_comm_period = (_state.comm_period + _state.averaged_comm_period * 3) / 4;// 使用权值平均计算方法(互补滤波)

    // 失能电机比较器比较中断
    motor_comparator_disable_from_isr();
}

static void prepare_zc_detector_for_next_step(void)
{
    // 获取当前的换相步骤
    const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
    motor_comparator_set_input_source(step->floating);
}

// 获取比较器的期望值
static bool get_comparator_expect_result(void)
{
    // 获取当前的换相步骤
    const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;

    bool result;
    if(step->positive == 0 && step->negative == 1) { // AB通电,检测C相,由1->0
        result = false;
    } else if(step->positive == 0 && step->negative == 2) {// AC通电,检测B相,由0->1
        result = true;
    } else if(step->positive == 1 && step->negative == 2) {// BC通电,检测A相,由1->0
        result = false;
    } else if(step->positive == 1 && step->negative == 0) {// BA通电,检测C相,由0->1
        result = true;
    } else if(step->positive == 2 && step->negative == 0) {// CA通电,检测B相,由1->0
        result = false;
    } else if(step->positive == 2 && step->negative == 1) {// CB通电,检测A相,由0->1
        result = true;
    }

    return result;
}

/**
 * 电机比较器采样回调,检查是否过零点
 */
void motor_comparator_zc_callback(bool compare_result)
{
    // 当中断来临时,获取当前时间戳
    const uint64_t timestamp = motor_timer_hnsec();

    // printf("Comp:%d\n", compare_result);
    // 判断电机是否在行进过程中
    const bool proceed =
        ((_state.flags & FLAG_ACTIVE) != 0) &&
        (_state.zc_detection_result == ZC_NOT_DETECTED) &&
        (timestamp >= _state.blank_time_deadline);

    // 如果不是在行进过程中,则直接返回
    if (!proceed) {
        return;
    }

    // 判断是否过零点
    bool past_zc = (compare_result == get_comparator_expect_result());// 如果比较值等于期望值则过零出现

    if ((_state.flags & FLAG_SPINUP) == 0) {// 电机不是在启动中
        // [发生过零点事件,进行事件处理]
        handle_detected_zc(timestamp);
    } else {
        if (past_zc) {
        } else {
            // It is ABSOLUTELY CRUCIAL to provide a correct estimate of the last zero cross timestamp
            // when transitioning from spinup mode to normal mode, otherwise the normal mode will
            // quickly run out of sync!
            _state.prev_zc_timestamp = timestamp;
            _state.spinup_prev_zc_timestamp_set = true;
        }

        if (!_state.spinup_prev_zc_timestamp_set) {
            // We didn't have a chance to detect ZC properly, so we speculate
            _state.prev_zc_timestamp = timestamp;
            _state.spinup_prev_zc_timestamp_set = true;
        }

        const uint32_t new_comm_period = timestamp - _state.prev_comm_timestamp;

        // We're using 3x averaging in order to compensate for phase asymmetry
        _state.comm_period =
            MIN((new_comm_period + _state.comm_period * 2) / 3,
                _params.spinup_start_comm_period);

        if (_state.averaged_comm_period > 0) {
            _state.averaged_comm_period =
                (_state.comm_period + _state.averaged_comm_period * 3) / 4;
        } else {
            _state.averaged_comm_period = _state.comm_period;
        }

        _state.zc_detection_result = ZC_DETECTED;

        motor_timer_set_relative(0);
        motor_comparator_disable_from_isr();

        if (_state.averaged_comm_period <= _params.comm_period_max) {
            if (_state.pwm_val >= _state.pwm_val_after_spinup) {
                _state.flags &= ~FLAG_SPINUP;
            } else {
                // Speed up the ramp a bit in order to converge faster
                _state.pwm_val++;
            }
        }
    }
}

/**
 * 电机定时器回调函数(通过亚微秒级的时钟定时进而控制电机逻辑)
 * 过零事件真正发生后,需要延时一段时间并通过定时器进行回调.此处进行真正的换相!
 */
void motor_timer_callback(uint64_t timestamp_hnsec)
{
    // 如果电机未被激活则直接返回不作处理
    if (!(_state.flags & FLAG_ACTIVE)) {
        return;
    }

    // 定时器回调,判断是否在启动中
    if ((_state.flags & FLAG_SPINUP) == 0) {
        /*
         * Missing a step drops the advance angle back to negative 15 degrees temporarily,
         * in order to account for possible rapid deceleration.
         * We also pick the greater value among the real measured comm period and its average,
         * which greatly helps during very intensive deceleration if the motor has severe phase asymmetry,
         * or the load varies highly. The reason it helps with asymmetry is because during deceleration,
         * a short comm period phase can be followed by a long comm period phase; if, during this transition,
         * the commutation period drops due to deceleration, the zero cross detection deadline may occur
         * before the real zero cross happens. Since during deceleration the average comm period is
         * always greater than the real comm period, this problem can be avoided.
         */
        // 获取换相间隔(取最大值)
        const uint32_t cp = MAX(_state.comm_period, _state.averaged_comm_period);

        // 过零事件检测超时 = 换相间隔 + ......
        const uint32_t zc_detection_timeout = cp +
                                              TIMING_ADVANCE64(cp, get_effective_timing_advance_deg64()) +
                                              TIMING_ADVANCE64(cp, 16);

        // 设置相对延时时间
        motor_timer_set_relative(zc_detection_timeout);
    } else {
        // 设置相对延时时间
        motor_timer_set_relative(_params.spinup_start_comm_period);
        _state.spinup_prev_zc_timestamp_set = false;
    }

    // Next comm step 处理完成当前步,进入下一步准备
    _state.prev_comm_timestamp = timestamp_hnsec;// 记录当前换相时间戳
    // 六步换相 自增一步(0 1 2 3 4 5)
    _state.current_comm_step++;
    // 如果大于等于6步则直接清零,重新进行六步换相
    if (_state.current_comm_step >= MOTOR_NUM_COMMUTATION_STEPS) {
        _state.current_comm_step = 0;
    }

    // 是否需要停车
    bool stop_now = false;

    // 根据过零检测事件的结果进行处理
    switch (_state.zc_detection_result) {
    case ZC_DETECTED: {
        engage_current_comm_step();// 检测到过零事件,执行调速&换相操作
        register_good_step();// 注册好步
        _state.flags &= ~FLAG_SYNC_RECOVERY;
        break;
    }

    case ZC_DESATURATION: {// 饱和?
        engage_current_comm_step();// 检测到过零事件,执行调速&换相操作
        _state.prev_zc_timestamp = timestamp_hnsec - _state.comm_period / 2;
        _state.flags |= FLAG_SYNC_RECOVERY;
        _state.immediate_desaturations++;
        if (_state.immediate_desaturations >= _params.zc_failures_max) {
            stop_now = true;// 停车
        }
        break;
    }

    case ZC_NOT_DETECTED:
    case ZC_FAILED: {// 检测过零事件失败或未检测
        if (_state.flags & FLAG_SPINUP) {
            engage_current_comm_step();// 如果是刚启动则强制换相
        } else {
            if ((_state.flags & FLAG_SYNC_RECOVERY) == 0) {
                // Try to run one more step in powered mode...
                engage_current_comm_step();
            } else {
                // Disable power as a last resort - we're probably too much out of sync already
                motor_pwm_set_freewheeling();
            }
            _state.flags |= FLAG_SYNC_RECOVERY;
        }
        _state.prev_zc_timestamp = timestamp_hnsec - _state.comm_period / 2;
        register_bad_step(&stop_now);
        break;
    }

    default: {
        stop_now = true;
    }
    }

    // 是否需要电机停转
    if (stop_now) {
        stop_from_isr(); // No bounce no play
        return;
    }

    // 重新清零检测结果
    _state.zc_detection_result = ZC_NOT_DETECTED;
    _state.blank_time_deadline = timestamp_hnsec + _params.comm_blank_hnsec;

    // 准备下一步的检测
    prepare_zc_detector_for_next_step();
    // 重新开启比较器检测
    motor_comparator_enable_from_isr();

    // Special spinup processing 处理电机强制启动过程
    if ((_state.flags & FLAG_SPINUP) != 0) {
        // Spinup blanking time override
        const uint64_t blanking_time =
            ((uint64_t)_state.comm_period * (uint64_t)_params.spinup_blanking_time_permil) / 1000U;

        const uint64_t new_blanking_deadline = timestamp_hnsec + blanking_time;

        if (new_blanking_deadline > _state.blank_time_deadline) {
            _state.blank_time_deadline = new_blanking_deadline;
        }

        // Spinup voltage ramp handling
        const uint64_t delta = timestamp_hnsec - _state.started_at;
        if (delta >= _state.spinup_ramp_duration_hnsec) {
            _state.pwm_val = _state.pwm_val_after_spinup;// 赋予转动后的PWM值
        } else {
            _state.immediate_zc_failures = 0;

            // 计算新的PWM脉宽值
            const int new_pwm_val = _state.pwm_val_before_spinup +
                                    (((uint64_t)(_state.pwm_val_after_spinup - _state.pwm_val_before_spinup)) * delta) /
                                    _state.spinup_ramp_duration_hnsec;

            assert_param(new_pwm_val >= _state.pwm_val_before_spinup);
            assert_param(new_pwm_val <= _state.pwm_val_after_spinup);

            if (new_pwm_val > _state.pwm_val) {
                _state.pwm_val = new_pwm_val;
            }
        }

        // Spinup timeout 启动超时,则直接停机
        if ((_state.started_at + _params.spinup_timeout) <= timestamp_hnsec) {
            stop_from_isr();
        }
    }
}

/**
 * 初始化电机实时控制模块
 */
void motor_rtctl_init(void)
{


    // 7.电机参数配置
    init_parameters();

    // 8.电机控制模块停止
    motor_rtctl_stop();
}

/**
 * 电机实时控制模块_让电机开始转动起来
 */
void motor_rtctl_start(float initial_duty_cycle, float target_duty_cycle, float spinup_ramp_duration, bool reverse, unsigned num_prior_attempts)
{
    (void) num_prior_attempts;

    // 电机停转
    motor_rtctl_stop();                    // Just in case

    // 判断占空比参数是否正确
    if ((initial_duty_cycle <= 0) || (target_duty_cycle <= 0)) {
        return;
    }

    /*
     * Initialize the structs 初始化结构体
     */
    memset(&_state, 0, sizeof(_state));    // Mighty reset

    if ((spinup_ramp_duration > 0.0F) && (initial_duty_cycle < target_duty_cycle)) {
        // 计算电机启动前的PWM值
        _state.pwm_val_before_spinup = motor_pwm_compute_pwm_val(initial_duty_cycle);
        // 计算电机启动后的PWM值
        _state.pwm_val_after_spinup  = motor_pwm_compute_pwm_val(target_duty_cycle);
        // 计算电机启动升压升频时间(亚微秒)
        _state.spinup_ramp_duration_hnsec = (uint32_t)(spinup_ramp_duration * ((float)HNSEC_PER_SEC) + 0.5F);
    } else {
        // 计算电机启动前的PWM值
        _state.pwm_val_before_spinup = motor_pwm_compute_pwm_val(target_duty_cycle);  // no ramp
        // 计算电机启动后的PWM值
        _state.pwm_val_after_spinup  = _state.pwm_val_before_spinup;
        // 电机启动无升压升频过程
        _state.spinup_ramp_duration_hnsec = 0;
    }

    // 更新电机当前控制占空比 = 电机启动前的PWM值
    _state.pwm_val = _state.pwm_val_before_spinup;

    // 赋值电机换相顺序表
    _state.comm_table = reverse ? COMMUTATION_TABLE_REVERSE : COMMUTATION_TABLE_FORWARD;
    // 记录电机开始换相时间
    _state.comm_period = _params.spinup_start_comm_period;

    // 上一次过零事件发生的时间 = 当前时刻 - 换相时间/2
    _state.prev_zc_timestamp = motor_timer_hnsec() - _state.comm_period / 2;
    // 假设当前已经捕获到过零点事件
    _state.zc_detection_result = ZC_DETECTED;
    // 状态变更为激活并启动中
    _state.flags = FLAG_ACTIVE | FLAG_SPINUP;

    // 记录当前时刻
    _state.started_at = motor_timer_hnsec();
    // 设置相对的延时时间(亚纳秒),延时结束后回调对应函数
    motor_timer_set_relative(_state.comm_period / 2);

    printf("Motor: RTCTL Spinup: PWM val %d --> %d\n",
           _state.pwm_val_before_spinup, _state.pwm_val_after_spinup);
}

/**
 * 电机实时控制模块_让电机停转
 */
void motor_rtctl_stop(void)
{
    _state.flags = 0;
    motor_timer_cancel();
    _state.flags = 0;

    irq_primask_disable();
    motor_comparator_enable_from_isr(); // ADC should be enabled by default
    irq_primask_enable();

    motor_pwm_set_freewheeling();
}

/**
 * 电机实时控制模块_设置电机PWM输出的占空比
 */
void motor_rtctl_set_duty_cycle(float duty_cycle)
{
    // We don't need a critical section to write an integer
    _state.pwm_val = motor_pwm_compute_pwm_val(duty_cycle);
}

/**
 * 获取当前电机实时控制模块状态
 */
enum motor_rtctl_state motor_rtctl_get_state(void)
{
    volatile const unsigned flags = _state.flags;

    if (flags & FLAG_ACTIVE) {
        return (flags & FLAG_SPINUP) ? MOTOR_RTCTL_STATE_STARTING : MOTOR_RTCTL_STATE_RUNNING;
    } else {
        return MOTOR_RTCTL_STATE_IDLE;
    }
}

/**
 * 电机实时控制模块_蜂鸣器响起来
 */
void motor_rtctl_beep(int frequency, int duration_msec)
{
    // 当电机正在运转时直接返回,无蜂鸣
    if (_state.flags & FLAG_ACTIVE) {
        return;
    }

    // 关闭中断
    irq_primask_disable();
    // 关闭比较器检测
    motor_comparator_disable_from_isr();
    // 重新打开中断
    irq_primask_enable();

    // 电机蜂鸣响(频率 时间)
    motor_pwm_beep(frequency, duration_msec);

    // 关闭中断
    irq_primask_disable();
    // 蜂鸣结束后重新打开比较器检测
    motor_comparator_enable_from_isr();
    // 重新打开中断
    irq_primask_enable();

    /*
     * Motor windings may get saturated after beeping, making immediately following spinup unreliable.
     * This little delay fixes that, not in the best way though.
     */
    // delayMs(10);
}

/**
 * 获取换相平均时间(亚纳秒)
 */
uint32_t motor_rtctl_get_comm_period_hnsec(void)
{
    if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
        return 0;
    }

    // 返回换相的平均时间
    return _state.averaged_comm_period;
}

void motor_rtctl_emergency(void)
{
    const irqstate_t irqstate = irq_primask_save();
    {
        motor_pwm_set_break();
        _state.flags = 0;
        motor_timer_cancel();
    }
    irq_primask_restore(irqstate);
}

void motor_rtctl_get_input_voltage_current(float* out_voltage, float* out_current)
{
    if (out_voltage) {
        *out_voltage = motor_adc_get_voltage();
    }
    if (out_current) {
        *out_current = motor_adc_get_current();
    }
}

/**
 * 电机实时控制模块_获取最小的换相时间(亚纳秒)
 */
uint32_t motor_rtctl_get_min_comm_period_hnsec(void)
{
    // Ensure some number of ADC samples per comm period
    // return MAX(109 * HNSEC_PER_USEC, motor_adc_sampling_period_hnsec() * 8);
    return 0;
}
