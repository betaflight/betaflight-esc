#include "include.h"

/**
 * Computes the timing advance in comm_period units 
 */
#define TIMING_ADVANCE64(comm_period, degrees) \
    (((int64_t)comm_period * (int64_t)degrees) / 64LL)

#define MIN(a, b)                  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                  (((a) > (b)) ? (a) : (b))

/**
 * Commutation tables 
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
// CB -> AC -> BA -> CB -> AC -> BA -> CB -> AC -> BA
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

enum flags
{
    FLAG_ACTIVE        = 1,
    FLAG_SPINUP        = 2,
    FLAG_SYNC_RECOVERY = 4 
};

/**
 *  zero crossing detection
 */
enum zc_detection_result
{
    ZC_NOT_DETECTED, 
    ZC_DETECTED, 
    ZC_FAILED,
    ZC_DESATURATION
};

static struct control_state
{
    unsigned flags;
    enum zc_detection_result zc_detection_result;

    uint64_t blank_time_deadline;
    uint64_t prev_zc_timestamp;
    uint64_t prev_comm_timestamp;

    uint32_t comm_period;
    uint32_t averaged_comm_period;

    int current_comm_step;
    const struct motor_pwm_commutation_step* comm_table;

    unsigned immediate_zc_failures;
    unsigned immediate_zc_detects;
    unsigned immediate_desaturations;

    int pwm_val;
    int pwm_val_before_spinup;
    int pwm_val_after_spinup;

    bool spinup_prev_zc_timestamp_set;
    uint64_t spinup_ramp_duration_hnsec;

    uint64_t started_at;
} _state;

static struct precomputed_params       /// Parameters are read only
{
    int timing_advance_min_deg64;
    int timing_advance_max_deg64;
    uint32_t max_comm_period_for_max_timing_advance;
    uint32_t max_comm_period_for_min_timing_advance;

    unsigned zc_failures_max;
    uint32_t comm_period_max;
    int comm_blank_hnsec;

    uint32_t spinup_start_comm_period;
    uint32_t spinup_timeout;
    uint32_t spinup_blanking_time_permil;
} _params;

static void init_parameters(void)
{
    _params.timing_advance_min_deg64               = motor_rtctl_Config()->mot_tim_adv_min * 64 / 60;
    _params.timing_advance_max_deg64               = motor_rtctl_Config()->mot_tim_adv_max * 64 / 60;
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

static void register_good_step(void)
{
    if (_state.immediate_zc_failures > 0) {
        _state.immediate_zc_detects++;
        if (_state.immediate_zc_detects > MOTOR_NUM_COMMUTATION_STEPS) {
            _state.immediate_zc_failures = 0;
        }
    }

    if (_state.immediate_desaturations > 0) {
        _state.immediate_desaturations--;
    }
}

static void register_bad_step(bool* need_to_stop)
{
    _state.immediate_zc_detects = 0;
    _state.immediate_zc_failures++;
    *need_to_stop = _state.immediate_zc_failures > _params.zc_failures_max;
}

static inline int get_effective_timing_advance_deg64(void)
{
    /*
     * Handling extremes
     */
    if (_state.comm_period <= _params.max_comm_period_for_max_timing_advance) {
        return _params.timing_advance_max_deg64;
    }

    if (_state.comm_period >= _params.max_comm_period_for_min_timing_advance) {
        return _params.timing_advance_min_deg64;
    }

    if (_state.flags & (FLAG_SPINUP | FLAG_SYNC_RECOVERY)) {
        return _params.timing_advance_min_deg64;
    }

    /*
     * Linear interpolation
     */
    const int tim_delta = _params.timing_advance_max_deg64 - _params.timing_advance_min_deg64;

    //
    const int result = _params.timing_advance_max_deg64 -
                       (tim_delta * (int64_t)(_state.comm_period - _params.max_comm_period_for_max_timing_advance) /
                        (_params.max_comm_period_for_min_timing_advance - _params.max_comm_period_for_max_timing_advance));
    return result;
}

static void handle_detected_zc(uint64_t zc_timestamp)
{
    if (_state.flags & FLAG_SYNC_RECOVERY) {
        /*
         * TODO: Proper sync recovery:
         * - Disable PWM
         * - Forget current comm step number - we're out of sync, so the step number is likely to be wrong
         * - Infer the current rotor position from the BEMF signals
         * - Measure the comm period using two subsequent ZC events
         * - Resume PWM
         */
        _state.comm_period = zc_timestamp - _state.prev_zc_timestamp;
        engage_current_comm_step();
    } else {
        const uint64_t predicted_zc_ts = _state.prev_zc_timestamp + _state.comm_period;
        zc_timestamp = (predicted_zc_ts + zc_timestamp + 2ULL) / 2ULL;
        _state.comm_period = zc_timestamp - _state.prev_zc_timestamp;
    }

    _state.prev_zc_timestamp = zc_timestamp;
    _state.comm_period = MIN(_state.comm_period, _params.comm_period_max);
    _state.zc_detection_result = ZC_DETECTED;

    _state.averaged_comm_period = (_state.comm_period + _state.averaged_comm_period * 3) / 4;

    motor_comparator_disable_from_isr();
}

static void prepare_zc_detector_for_next_step(void)
{
    const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
    motor_comparator_set_input_source(step->floating);
}

static bool get_comparator_expect_result(void)
{
    const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;

    bool result;
    if(step->positive == 0 && step->negative == 1) {
        result = false;
    } else if(step->positive == 0 && step->negative == 2) {
        result = true;
    } else if(step->positive == 1 && step->negative == 2) {
        result = false;
    } else if(step->positive == 1 && step->negative == 0) {
        result = true;
    } else if(step->positive == 2 && step->negative == 0) {
        result = false;
    } else if(step->positive == 2 && step->negative == 1) {
        result = true;
    }

    return result;
}

void motor_comparator_zc_callback(bool compare_result)
{
    const uint64_t timestamp = motor_timer_hnsec();

    // printf("Comp:%d\n", compare_result);
    const bool proceed =
        ((_state.flags & FLAG_ACTIVE) != 0) &&
        (_state.zc_detection_result == ZC_NOT_DETECTED) &&
        (timestamp >= _state.blank_time_deadline);

    if (!proceed) {
        return;
    }

    bool past_zc = (compare_result == get_comparator_expect_result());

    if ((_state.flags & FLAG_SPINUP) == 0) {
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

void motor_timer_callback(uint64_t timestamp_hnsec)
{
    if (!(_state.flags & FLAG_ACTIVE)) {
        return;
    }

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
        const uint32_t cp = MAX(_state.comm_period, _state.averaged_comm_period);


        const uint32_t zc_detection_timeout = cp +
                                              TIMING_ADVANCE64(cp, get_effective_timing_advance_deg64()) +
                                              TIMING_ADVANCE64(cp, 16);

        motor_timer_set_relative(zc_detection_timeout);
    } else {
        motor_timer_set_relative(_params.spinup_start_comm_period);
        _state.spinup_prev_zc_timestamp_set = false;
    }

    _state.prev_comm_timestamp = timestamp_hnsec;
    _state.current_comm_step++;

    if (_state.current_comm_step >= MOTOR_NUM_COMMUTATION_STEPS) {
        _state.current_comm_step = 0;
    }

    bool stop_now = false;

    switch (_state.zc_detection_result) {
    case ZC_DETECTED: {
        engage_current_comm_step();
        register_good_step();
        _state.flags &= ~FLAG_SYNC_RECOVERY;
        break;
    }
    case ZC_DESATURATION: {
        engage_current_comm_step();
        _state.prev_zc_timestamp = timestamp_hnsec - _state.comm_period / 2;
        _state.flags |= FLAG_SYNC_RECOVERY;
        _state.immediate_desaturations++;
        if (_state.immediate_desaturations >= _params.zc_failures_max) {
            stop_now = true;
        }
        break;
    }
    case ZC_NOT_DETECTED:
    case ZC_FAILED: {
        if (_state.flags & FLAG_SPINUP) {
            engage_current_comm_step();
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

    if (stop_now) {
        stop_from_isr(); // No bounce no play
        return;
    }

    _state.zc_detection_result = ZC_NOT_DETECTED;
    _state.blank_time_deadline = timestamp_hnsec + _params.comm_blank_hnsec;

    prepare_zc_detector_for_next_step();
    motor_comparator_enable_from_isr();

    // Special spinup processing 
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
            _state.pwm_val = _state.pwm_val_after_spinup;
        } else {
            _state.immediate_zc_failures = 0;
            const int new_pwm_val = _state.pwm_val_before_spinup +
                                    (((uint64_t)(_state.pwm_val_after_spinup - _state.pwm_val_before_spinup)) * delta) /
                                    _state.spinup_ramp_duration_hnsec;

            assert_param(new_pwm_val >= _state.pwm_val_before_spinup);
            assert_param(new_pwm_val <= _state.pwm_val_after_spinup);

            if (new_pwm_val > _state.pwm_val) {
                _state.pwm_val = new_pwm_val;
            }
        }

        if ((_state.started_at + _params.spinup_timeout) <= timestamp_hnsec) {
            stop_from_isr();
        }
    }
}

void motor_rtctl_init(void)
{
    init_parameters();
    motor_rtctl_stop();
}

void motor_rtctl_start(float initial_duty_cycle, float target_duty_cycle, float spinup_ramp_duration, bool reverse, unsigned num_prior_attempts)
{
    (void) num_prior_attempts;

    motor_rtctl_stop();                    // Just in case

    if ((initial_duty_cycle <= 0) || (target_duty_cycle <= 0)) {
        return;
    }

    /*
     * Initialize the structs 
     */
    memset(&_state, 0, sizeof(_state));    // Mighty reset

    if ((spinup_ramp_duration > 0.0F) && (initial_duty_cycle < target_duty_cycle)) {
        _state.pwm_val_before_spinup = motor_pwm_compute_pwm_val(initial_duty_cycle);
        _state.pwm_val_after_spinup  = motor_pwm_compute_pwm_val(target_duty_cycle);
        _state.spinup_ramp_duration_hnsec = (uint32_t)(spinup_ramp_duration * ((float)HNSEC_PER_SEC) + 0.5F);
    } else {
        _state.pwm_val_before_spinup = motor_pwm_compute_pwm_val(target_duty_cycle);
        _state.pwm_val_after_spinup  = _state.pwm_val_before_spinup;
        _state.spinup_ramp_duration_hnsec = 0;
    }

    _state.pwm_val = _state.pwm_val_before_spinup;

    _state.comm_table = reverse ? COMMUTATION_TABLE_REVERSE : COMMUTATION_TABLE_FORWARD;
    _state.comm_period = _params.spinup_start_comm_period;

    _state.prev_zc_timestamp = motor_timer_hnsec() - _state.comm_period / 2;
    _state.zc_detection_result = ZC_DETECTED;
    _state.flags = FLAG_ACTIVE | FLAG_SPINUP;


    _state.started_at = motor_timer_hnsec();
    motor_timer_set_relative(_state.comm_period / 2);

    printf("Motor: RTCTL Spinup: PWM val %d --> %d\n",
           _state.pwm_val_before_spinup, _state.pwm_val_after_spinup);
}

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

void motor_rtctl_set_duty_cycle(float duty_cycle)
{
    // We don't need a critical section to write an integer
    _state.pwm_val = motor_pwm_compute_pwm_val(duty_cycle);
}

enum motor_rtctl_state motor_rtctl_get_state(void)
{
    volatile const unsigned flags = _state.flags;

    if (flags & FLAG_ACTIVE) {
        return (flags & FLAG_SPINUP) ? MOTOR_RTCTL_STATE_STARTING : MOTOR_RTCTL_STATE_RUNNING;
    } else {
        return MOTOR_RTCTL_STATE_IDLE;
    }
}

void motor_rtctl_beep(int frequency, int duration_msec)
{
    if (_state.flags & FLAG_ACTIVE) {
        return;
    }

    irq_primask_disable();
    motor_comparator_disable_from_isr();
    irq_primask_enable();
    motor_pwm_beep(frequency, duration_msec);

    irq_primask_disable();
    motor_comparator_enable_from_isr();
    irq_primask_enable();

    /*
     * Motor windings may get saturated after beeping, making immediately following spinup unreliable.
     * This little delay fixes that, not in the best way though.
     */
    // delayMs(10);
}

uint32_t motor_rtctl_get_comm_period_hnsec(void)
{
    if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
        return 0;
    }
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

uint32_t motor_rtctl_get_min_comm_period_hnsec(void)
{
    // Ensure some number of ADC samples per comm period
    // return MAX(109 * HNSEC_PER_USEC, motor_adc_sampling_period_hnsec() * 8);
    return 0;
}
