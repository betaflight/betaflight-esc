#include "include.h"

static uint16_t _pwm_max;// 上桥MOS管的最大PWM值
static uint16_t _pwm_mid;// 上桥MOS管的中位PWM值(不完全中位,因为存在PWM死区)
static uint16_t _pwm_min;// 上桥MOS管的最小PWM值

/**
 * 初始化常量
 */
static void init_constants(uint16_t frequency, uint16_t pwm_dead_time_ns)
{
    // PWM步长 = 48Mhz / 频率值
    const int pwm_steps = SystemCoreClock / frequency;
    // 计算 上桥MOS管的最大PWM值(重装载值)
    _pwm_max = pwm_steps - 1;
    // 计算 上桥MOS管的中位PWM值
    _pwm_mid = pwm_steps / 2;
    // 计算 上桥MOS管的最小PWM值(需要减去死区值)
    _pwm_min = (uint16_t)(((float)pwm_dead_time_ns / 1e9f) / (1.f / SystemCoreClock));
}

/**
 * 初始化定时器硬件部分
 */
static void init_timers(uint16_t pwm_dead_time_ns)
{
    /* GPIOA and GPIOB clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    /*--------------------IO结构体初始化-------------------------*/
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

    /*--------------------时基结构体初始化-------------------------*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;// 不预分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上计数模式
    TIM_TimeBaseStructure.TIM_Period = _pwm_max;// 定时器的计数周期设置
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /*--------------------输出比较结构体初始化-------------------*/
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;// 配置为 PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;// 输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;// 互补输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;// 输出通道电平极性配置
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;// 互补输出通道电平极性配置
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;// 输出通道空闲电平极性配置
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;// 互补输出通道空闲电平极性配置
    TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);// 初始化定时器通道1输出PWM
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);// 初始化定时器通道2输出PWM
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);// 初始化定时器通道3输出PWM

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);// 使能预分频装载值
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);// 使能预分频装载值
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);// 使能预分频装载值

    /*
     * 计算死区时间
     * Dead time generator setup.
     * DTS clock divider set 0, hence fDTS = input clock.
     * DTG bit 7 must be 0, otherwise it will change multiplier which is not supported yet.
     * At 48 MHz one tick ~ 20.8 nsec, max 127 * 20.8 ~ 2.645 usec, which is large enough.
     */
    uint16_t dead_time_ticks = (uint16_t)(pwm_dead_time_ns / 20.8);
    if (dead_time_ticks > 127) {
        dead_time_ticks = 127;
    }

    /*-------------------刹车和死区结构体初始化-------------------*/
    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = dead_time_ticks;// 设置互补输出的死区时间,免得上下两管同时导通
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;// 主输出使能TIM_BDTR_AOE
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_CCPreloadControl(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /*-------------------TIM1事件产生寄存器-------------------*/
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update | TIM_EventSource_COM);// 重新初始化计数器, 并产生一个(寄存器)更新事件

    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);

    /* Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
 * 初始化MOS管对应的定时器
 */
void motor_pwm_init(void)
{
    // 初始化参数
    init_constants(motor_pwm_Config()->mot_pwm_hz, motor_pwm_Config()->mot_pwm_dt_ns);

    // 根据参数初始化高级定时器1
    init_timers(motor_pwm_Config()->mot_pwm_dt_ns);

    // 电机惯性滑行
    motor_pwm_set_freewheeling();
}

/**
 * 关闭电机单相的PWM输出.
 */
static inline void phase_reset_i(uint_fast8_t phase)
{
    switch(phase) {
    case 0:
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
        break;

    case 1:
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
        break;

    case 2:
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
 *  - motor IRQs are disabled 六步换相核心程序(H-PWM,L-ON)
 */
static inline void phase_set_i(uint_fast8_t phase, uint_fast16_t pwm_val, bool isPWMInput)
{
    switch(phase) {
    case 0:
        TIM_SetCompare3(TIM1, pwm_val);// 输出C相比较值
        TIM_SelectOCxM(TIM1, TIM_Channel_3, isPWMInput ? TIM_OCMode_PWM1 : TIM_OCMode_Inactive);
        // 对C相输出互补双极PWM波
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
        break;

    case 1:
        TIM_SetCompare2(TIM1, pwm_val);// 输出B相比较值
        TIM_SelectOCxM(TIM1, TIM_Channel_2, isPWMInput ? TIM_OCMode_PWM1 : TIM_OCMode_Inactive);
        // 对B相输出互补双极PWM波
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
        break;

    case 2:
        TIM_SetCompare1(TIM1, pwm_val);// 输出A相比较值
        TIM_SelectOCxM(TIM1, TIM_Channel_1, isPWMInput ? TIM_OCMode_PWM1 : TIM_OCMode_Inactive);
        // 对A相输出互补双极PWM波
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
        break;

    default:
        break;
    }
}

// 停止PWM输出(下管全关,上管全关)
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

// 电机刹车(下管全开,上管全关)
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

// 操作三相PWM值输出
void motor_pwm_manip(const enum motor_pwm_phase_manip command[MOTOR_NUM_PHASES])
{
    // 关闭中断
    irq_primask_disable();
    // 关闭电机全部相的PWM输出,清零&重新初始化
    motor_pwm_stop();
    // 开启中断
    irq_primask_enable();

    // 遍历电机三相
    for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
        if (command[phase] == MOTOR_PWM_MANIP_HIGH) {// 高位
            // We don't want to engage 100% duty cycle because the high side pump needs switching
            const int pwm_val = motor_pwm_compute_pwm_val(0.80f);
            irq_primask_disable();
            phase_set_i(phase, pwm_val, false);
            irq_primask_enable();
        } else if (command[phase] == MOTOR_PWM_MANIP_HALF) {// 中立点
            irq_primask_disable();
            phase_set_i(phase, _pwm_mid, false);
            irq_primask_enable();
        } else if (command[phase] == MOTOR_PWM_MANIP_LOW) {// 低位
            irq_primask_disable();
            phase_set_i(phase, 0, false);
            irq_primask_enable();
        } else if (command[phase] == MOTOR_PWM_MANIP_FLOATING) {// 浮动
            // Nothing to do
        }
    }
}

/**
 * 电机惯性滑行,不受PWM电压控制
 */
void motor_pwm_set_freewheeling(void)
{
    // 关闭中断
    irq_primask_disable();
    // 停止输出PWM方波
    motor_pwm_stop();
    // 重新打开中断
    irq_primask_enable();
}

/**
 * 电机紧急停车
 */
void motor_pwm_set_break(void)
{
    irq_primask_disable();
    motor_pwm_brake();
    irq_primask_enable();
}

/**
 * 根据占空比的比值计算对应的PWM值
 */
int motor_pwm_compute_pwm_val(float duty_cycle)
{
    /*
     * Normalize into [0; PWM_TOP] regardless of sign 线性化
     */
    const float abs_duty_cycle = fabs(duty_cycle);// 归一化
    uint_fast16_t int_duty_cycle = 0;
    if (abs_duty_cycle > 0.999) {
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
        // Forward mode 正数时
        output = _pwm_max - ((_pwm_max - int_duty_cycle) / 2) + 1;
    } else {
        // Braking mode 负数时
        output = (_pwm_max - int_duty_cycle) / 2;

        // 限幅
        if (output < _pwm_min) {
            output = _pwm_min;
        }
    }

    // 获得输出值
    return output;
}

/**
 * 电机换向&调速(对三相进行操作)
 */
void motor_pwm_set_step_and_pwm(const struct motor_pwm_commutation_step* step, int pwm_val)
{
    // 浮空相---->处理
    phase_reset_i(step->floating);
    // 正极---->PWM处理
    phase_set_i(step->positive, pwm_val, true);
    // 负极---->通断处理,高电平(强置输出模式)(下管直接导通,力矩较大)
    phase_set_i(step->negative, pwm_val, false);

    // 无刷电机换向时,一般是三相要同时换向的,但是你在软件设置换向时肯定是一次只能设置一相,这就达不到三相同时换向了.
    // 其实简单的办法就是启用STM32的COM事件,你先逐个设置好每相的换向(注意:此时虽然设置了,但实际上并不会进行换向),然后再调用COM事件.此时,三相将同时换向.
    /* Generate TIM1 COM event by software */
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/**
 * 电机蜂鸣响(频率 时间)
 */
void motor_pwm_beep(int frequency, int duration_msec)
{
    static const float DUTY_CYCLE = 0.01;
    static const int ACTIVE_USEC_MAX = 20;

    // 电机惯性滑行
    motor_pwm_set_freewheeling();

    // 频率限幅(100Hz - 5000Hz)
    frequency = (frequency < 100)  ? 100  : frequency;
    frequency = (frequency > 5000) ? 5000 : frequency;

    // TODO HACK: Longer beeps disrupt other processes, beeping should be done in a non-blocking way from ISR
    // 时间限幅(1ms - 100ms)
    duration_msec = (duration_msec < 1)   ? 1   : duration_msec;
    duration_msec = (duration_msec > 100) ? 100 : duration_msec;

    /*
     * Timing constants 时间常数
     */
    const int half_period_hnsec = (HNSEC_PER_SEC / frequency) / 2;

    int active_hnsec = half_period_hnsec * DUTY_CYCLE;

    if (active_hnsec > ACTIVE_USEC_MAX * HNSEC_PER_USEC) {
        active_hnsec = ACTIVE_USEC_MAX * HNSEC_PER_USEC;
    }

    // 空闲时间 = 总时间 - 激活时间
    const int idle_hnsec = half_period_hnsec - active_hnsec;

    // 结束时间
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

    // 电机惯性滑行
    motor_pwm_set_freewheeling();
}
