#include "include.h"

static uint16_t _pwm_max;
static uint16_t _pwm_mid;
static uint16_t _pwm_min;

static void init_constants(uint16_t frequency, uint16_t pwm_dead_time_ns)
{
    const int pwm_steps = SystemCoreClock / frequency;
    _pwm_max = pwm_steps - 1;
    _pwm_mid = pwm_steps / 2;
    _pwm_min = (uint16_t)(((float)pwm_dead_time_ns / 1e9f) / (1.f / SystemCoreClock));
}

static void init_timers(uint16_t pwm_dead_time_ns)
{
    /* GPIOA and GPIOB clocks enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB);

    /* GPIOA Configuration: Channel 1, 2, 1N and 3 as alternate function push-pull */
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* GPIOB Configuration: Channel 2N and 3N as alternate function push-pull */
    GPIO_InitStructure.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM pins to AF2 */
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_0, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_1, LL_GPIO_AF_2);

    /* TIM1 clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    LL_TIM_InitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_TimeBaseStructure.Autoreload = _pwm_max;
    TIM_TimeBaseStructure.ClockDivision = 0;
    TIM_TimeBaseStructure.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_TimeBaseStructure);

    LL_TIM_OC_InitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OCInitStructure.OCNState = LL_TIM_OCSTATE_ENABLE;
    TIM_OCInitStructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
    TIM_OCInitStructure.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
    TIM_OCInitStructure.CompareValue = TIM1->ARR / 2;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OCInitStructure);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OCInitStructure);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    /*
     * Dead time generator setup.
     * DTS clock divider set 0, hence fDTS = input clock.
     * DTG bit 7 must be 0, otherwise it will change multiplier which is not supported yet.
     * At 48 MHz one tick ~ 20.8 nsec, max 127 * 20.8 ~ 2.645 usec, which is large enough.
     */
    uint16_t dead_time_ticks = (uint16_t)(pwm_dead_time_ns / 20.8);
    if (dead_time_ticks > 127) {
        dead_time_ticks = 127;
    }

    /* Automatic Output enable, Break, dead time and lock configuration*/
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.OSSRState = LL_TIM_OSSR_ENABLE;
    TIM_BDTRInitStructure.OSSIState = LL_TIM_OSSI_ENABLE;
    TIM_BDTRInitStructure.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStructure.DeadTime = dead_time_ticks;
    TIM_BDTRInitStructure.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStructure.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStructure.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStructure);

    LL_TIM_CC_EnablePreload(TIM1);
    LL_TIM_EnableARRPreload(TIM1);

    LL_TIM_GenerateEvent_COM(TIM1);

    /* TIM1 counter enable */
    LL_TIM_EnableCounter(TIM1);

    /* Main Output Enable */
    LL_TIM_EnableAllOutputs(TIM1);
}

void motor_pwm_init(void)
{
    init_constants(motor_pwm_Config()->mot_pwm_hz, motor_pwm_Config()->mot_pwm_dt_ns);

    init_timers(motor_pwm_Config()->mot_pwm_dt_ns);

    motor_pwm_set_freewheeling();
}

static inline void phase_reset_i(motorPhase_e phase)
{
    switch(phase) {
    case PHASE_A:
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
        break;

    case PHASE_B:
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
        break;

    case PHASE_C:
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
        break;

    default:
        break;
    }
}

/**
 * Assumes:
 *  - motor IRQs are disabled
 */
static inline void phase_set_i(motorPhase_e phase, uint_fast16_t pwm_val, bool isPWMInput)
{
    switch(phase) {
    case PHASE_A:
        LL_TIM_OC_SetCompareCH3(TIM1, pwm_val);
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, isPWMInput ? LL_TIM_OCMODE_PWM1 : LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
        break;

    case PHASE_B:
        LL_TIM_OC_SetCompareCH2(TIM1, pwm_val);
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, isPWMInput ? LL_TIM_OCMODE_PWM1 : LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
        break;

    case PHASE_C:
        LL_TIM_OC_SetCompareCH1(TIM1, pwm_val);
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, isPWMInput ? LL_TIM_OCMODE_PWM1 : LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
        break;

    default:
        break;
    }
}

static inline void motor_pwm_stop(void)
{
    LL_TIM_OC_SetCompareCH1(TIM1, _pwm_mid);
    LL_TIM_OC_SetCompareCH2(TIM1, _pwm_mid);
    LL_TIM_OC_SetCompareCH3(TIM1, _pwm_mid);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

    /* Generate TIM1 COM event by software */
    LL_TIM_GenerateEvent_COM(TIM1);
}

static void motor_pwm_brake(void)
{
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

    /* Generate TIM1 COM event by software */
    LL_TIM_GenerateEvent_COM(TIM1);
}

void motor_pwm_manip(const enum motor_pwm_phase_manip command[MOTOR_NUM_PHASES])
{
    irq_primask_disable();
    motor_pwm_stop();
    irq_primask_enable();

    for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
        if (command[phase] == MOTOR_PWM_MANIP_HIGH) {
            // We don't want to engage 100% duty cycle because the high side pump needs switching
            const int pwm_val = motor_pwm_compute_pwm_val(0.80f);
            irq_primask_disable();
            phase_set_i(phase, pwm_val, false);
            irq_primask_enable();
        } else if (command[phase] == MOTOR_PWM_MANIP_HALF) {
            irq_primask_disable();
            phase_set_i(phase, _pwm_mid, false);
            irq_primask_enable();
        } else if (command[phase] == MOTOR_PWM_MANIP_LOW) {
            irq_primask_disable();
            phase_set_i(phase, 0, false);
            irq_primask_enable();
        } else if (command[phase] == MOTOR_PWM_MANIP_FLOATING) {
            // Nothing to do
        }
    }
}

void motor_pwm_set_freewheeling(void)
{
    irq_primask_disable();
    motor_pwm_stop();
    irq_primask_enable();
}

void motor_pwm_set_break(void)
{
    irq_primask_disable();
    motor_pwm_brake();
    irq_primask_enable();
}

int motor_pwm_compute_pwm_val(float duty_cycle)
{
    /*
     * Normalize into [0; PWM_TOP] regardless of sign 
     */
    const float abs_duty_cycle = fabs(duty_cycle);
    uint_fast16_t int_duty_cycle = 0;
    if (abs_duty_cycle > 0.999f) {
        int_duty_cycle = _pwm_max;
    } else {
        int_duty_cycle = (uint_fast16_t)(abs_duty_cycle * _pwm_max);
    }

    /*
     * Compute the complementary duty cycle
     * Ref. "Influence of PWM Schemes and Commutation Methods for DC and Brushless Motors and Drives", page 4.
     */
    int output = 0;

    if (duty_cycle >= 0) {
        // Forward mode
        output = _pwm_max - ((_pwm_max - int_duty_cycle) / 2) + 1;
    } else {
        // Braking mode
        output = (_pwm_max - int_duty_cycle) / 2;

        if (output < _pwm_min) {
            output = _pwm_min;
        }
    }

    return output;
}

void motor_pwm_set_step_and_pwm(const struct motor_pwm_commutation_step* step, int pwm_val)
{
    phase_reset_i(step->floating);
    phase_set_i(step->positive, pwm_val, true);
    phase_set_i(step->negative, pwm_val, false);
    /* Generate TIM1 COM event by software */
    LL_TIM_GenerateEvent_COM(TIM1);
}

void motor_pwm_beep(int frequency, int duration_msec)
{
    static const float DUTY_CYCLE = 0.01;
    static const int ACTIVE_USEC_MAX = 20;

    motor_pwm_set_freewheeling();

    frequency = (frequency < 100)  ? 100  : frequency;
    frequency = (frequency > 5000) ? 5000 : frequency;

    // TODO HACK: Longer beeps disrupt other processes, beeping should be done in a non-blocking way from ISR
    duration_msec = (duration_msec < 1)   ? 1   : duration_msec;
    duration_msec = (duration_msec > 100) ? 100 : duration_msec;

    /*
     * Timing constants 
     */
    const int half_period_hnsec = (HNSEC_PER_SEC / frequency) / 2;

    int active_hnsec = half_period_hnsec * DUTY_CYCLE;

    if (active_hnsec > ACTIVE_USEC_MAX * HNSEC_PER_USEC) {
        active_hnsec = ACTIVE_USEC_MAX * HNSEC_PER_USEC;
    }

    const int idle_hnsec = half_period_hnsec - active_hnsec;
    const uint64_t end_time = motor_timer_hnsec() + duration_msec * HNSEC_PER_MSEC;

    /*
     * FET round robin
     * This way we can beep even if some FETs went bananas
     */
    static unsigned _phase_sel;
    const int low_phase_first  = _phase_sel++ % MOTOR_NUM_PHASES;
    const int low_phase_second = _phase_sel++ % MOTOR_NUM_PHASES;
    const int high_phase       = _phase_sel++ % MOTOR_NUM_PHASES;
    _phase_sel++;              // We need to increment it not by multiple of 3
    assert_param(low_phase_first != high_phase && low_phase_second != high_phase);

    /*
     * Commutations
     * No high side pumping
     */
    phase_set_i(low_phase_first, 0, false);
    phase_set_i(low_phase_second, 0, false);
    phase_set_i(high_phase, 0, false);

    while (end_time > motor_timer_hnsec()) {
        irq_primask_disable();
        phase_set_i(high_phase, _pwm_max, false);
        irq_primask_enable();

        motor_timer_hndelay(active_hnsec);

        irq_primask_disable();
        phase_set_i(high_phase, 0, false);
        irq_primask_enable();

        motor_timer_hndelay(idle_hnsec);
    }

    motor_pwm_set_freewheeling();
}
