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
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    /* GPIOA Configuration: Channel 1, 2, 1N and 3 as alternate function push-pull */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* GPIOB Configuration: Channel 2N and 3N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM pins to AF2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);

    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = _pwm_max;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

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
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = dead_time_ticks;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_CCPreloadControl(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_GenerateEvent(TIM1, TIM_EventSource_Update | TIM_EventSource_COM);

    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);

    /* Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
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
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
        break;

    case PHASE_B:
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
        break;

    case PHASE_C:
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
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
        TIM_SetCompare3(TIM1, pwm_val);
        TIM_SelectOCxM(TIM1, TIM_Channel_3, isPWMInput ? TIM_OCMode_PWM1 : TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
        break;

    case PHASE_B:
        TIM_SetCompare2(TIM1, pwm_val);
        TIM_SelectOCxM(TIM1, TIM_Channel_2, isPWMInput ? TIM_OCMode_PWM1 : TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
        break;

    case PHASE_C:
        TIM_SetCompare1(TIM1, pwm_val);
        TIM_SelectOCxM(TIM1, TIM_Channel_1, isPWMInput ? TIM_OCMode_PWM1 : TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
        break;

    default:
        break;
    }
}

static inline void motor_pwm_stop(void)
{
    TIM_SetCompare1(TIM1, _pwm_mid);
    TIM_SetCompare2(TIM1, _pwm_mid);
    TIM_SetCompare3(TIM1, _pwm_mid);

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

    /* Generate TIM1 COM event by software */
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

static void motor_pwm_brake(void)
{
    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

    /* Generate TIM1 COM event by software */
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
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
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
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
