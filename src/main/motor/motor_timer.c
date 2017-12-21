#include "include.h"

/**
 * The timer frequency is a compromise between maximum delay and timer resolution
 * 2 MHz --> ~32ms max
 * 4 MHz --> ~16ms max
 * 5 MHz --> ~13ms max
 * Note that we build a postscaler on top of the hardware timer that allows us to generate arbitrarily long delays
 * regardless of the hardware capabilities.
 */
static const int MAX_FREQUENCY = 4000000;// 计时器最大计数频率为4Mhz

static const uint64_t INT_1E9 = 1000000000ul;
static const uint32_t TICKS_PER_OVERFLOW = 0xFFFF + 1;// 定时器溢出时的计数值

// nanoSecond 纳秒,十亿分之一秒
static uint32_t _nanosec_per_tick = 0;    // 0 means uninitialized
// 当"时间戳定时器17"中断时,该变量自增
static volatile uint64_t _raw_ticks = 0;
// 剩余的ticks数
static volatile int64_t _remaining_ticks = 0;

/**
 * Event callout timer declaration 定时器14,提供给电机精确的延时回调函数,就是通过设置绝对或相对的延时时间,再回调电机控制的相关函数
 * ------->>>>> Hard real time callout interface for motor control logic (preempts the kernel)
 */
void TIM14_IRQHandler(void)
{
    if ((TIM14->SR & TIM_SR_CC1IF) && (TIM14->DIER & TIM_DIER_CC1IE)) {
        TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);// 清理中断标志位

        if (_remaining_ticks <= (MAX_FREQUENCY / 1000000)) {// MIN_REMAINING_TICKS
            TIM_ITConfig(TIM14, TIM_IT_CC1, DISABLE);// Disable this compare match 失能该捕获匹配
            const uint64_t timestamp = motor_timer_hnsec() - 2;// 时间戳
            motor_timer_callback(timestamp);// 计时结束,马上回调函数
        } else {
            if (_remaining_ticks >= TICKS_PER_OVERFLOW) {// 定时器计数值溢出了,不需要重新配置CCR寄存器
                // We don't need to re-configure the CCR register - the value won't change
                _remaining_ticks -= TICKS_PER_OVERFLOW; // Go around 直接递减到零
            } else {// 定时器计数值未溢出
                TIM14->CCR1 += _remaining_ticks;
                _remaining_ticks = 0;                   // Go around the last time
            }
        }
    }
}

/**
 * Timestamping timer declaration 定时器17 获取准确的时间戳
 * ------->>>>> High precision timestamping for motor control logic (sub-microsecond resolution, never overflows)
 */
void TIM17_IRQHandler(void)
{
    // 清除TIM17中断标志位
    TIM_ClearITPendingBit(TIM17, TIM_IT_Update);

    _raw_ticks += TICKS_PER_OVERFLOW;
}

// 初始化电机定时器14和定时器17, 定时50ns
void motor_timer_init(void)
{
    /* TIM14 & TIM17 clock Power-on and reset */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM14, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM14, DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM17, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM17, DISABLE);

    // 计算预分频值 Find the optimal prescaler value = 48Mhz / 12Mhz = 4
    uint32_t prescaler = (uint32_t)(SystemCoreClock / ((float)MAX_FREQUENCY)); // Initial value

    if (prescaler < 1) {
        prescaler = 1;
    }

    for (;; prescaler++) {
        if (SystemCoreClock % prescaler) {
            continue;
        }

        const uint32_t prescaled_clock = SystemCoreClock / prescaler;
        if (INT_1E9 % prescaled_clock) {
            continue;
        }
        break; // Ok, current prescaler value can divide the timer frequency with no remainder
    }

    // 定时器分辨率,即每个tick中断对应多少纳秒, 1秒/频率 = 每tick中断对应时间
    _nanosec_per_tick = INT_1E9 / (SystemCoreClock / prescaler);// 1000000000/4=250纳秒

    printf("Motor: Timer resolution: %u nanosec, %u\n", (unsigned)_nanosec_per_tick, (unsigned)prescaler);

    /* Enable the TIM14 & TIM17 gloabal Interrupt */
    nvicEnableVector(TIM14_IRQn, 0);
    nvicEnableVector(TIM17_IRQn, 0);

    /* Time base configuration */
    TIM14->ARR = 0xFFFF;// 自动重装值
    TIM14->PSC = (uint16_t)(prescaler - 1);// 12分频,计数频率为4Mhz,分辨率为250纳秒
    TIM14->CR1 = TIM_CR1_URS;
    TIM14->SR  = 0;
    TIM14->EGR = TIM_EGR_UG;     // Reload immediately
    TIM14->CR1 = TIM_CR1_CEN;    // Start

    // Start the timestamping timer
    TIM17->ARR = 0xFFFF;// 自动重装值
    TIM17->PSC = (uint16_t)(prescaler - 1);// 12分频,计数频率为4Mhz,分辨率为250纳秒
    TIM17->CR1 = TIM_CR1_URS;
    TIM17->SR  = 0;
    TIM17->EGR = TIM_EGR_UG;     // Reload immediately
    TIM17->DIER = TIM_DIER_UIE;
    TIM17->CR1 = TIM_CR1_CEN;    // Start
}

/**
 * Returns the current timestamp in hectonanoseconds (10^-7).返回当前时间戳(亚微秒级)
 */
uint64_t motor_timer_hnsec(void)
{
    // 定义 tick 变量
    volatile uint64_t ticks = 0;
    volatile uint_fast16_t sample = 0;

    while (1) {
        ticks = _raw_ticks;// 变量赋值
        sample = TIM_GetCounter(TIM17);// 获取当前计数器的计数值

        const volatile uint64_t ticks2 = _raw_ticks;// 变量赋值

        // 比较是否一致
        if (ticks == ticks2) {
            if (TIM_GetFlagStatus(TIM17, TIM_FLAG_Update)) {
                sample = TIM_GetCounter(TIM17);
                ticks += TICKS_PER_OVERFLOW;
            }
            break;
        }
    }

    // 时间戳 = ( ticks总数 + 采样数 ) * 分辨率(每个tick中断对应多少纳秒,目前为250纳秒中断一次) / 100
    return ((ticks + sample) * _nanosec_per_tick) / 100;
}

// 设置相对的延时时间(亚纳秒),延时结束后回调对应函数,返回增量时间
void motor_timer_set_relative(int64_t delay_hnsec)
{
    delay_hnsec -= 1 * HNSEC_PER_USEC;
    if (delay_hnsec < 0) {
        delay_hnsec = 0;
    }

    // 计算所需要的ticks数 = 时间/分辨率
    int64_t delay_ticks = (delay_hnsec * 100) / _nanosec_per_tick;

    /*
     * Interrupts must be disabled completely because the following
     * sequence requires strict timing.
     * No port_*() functions are allowed here!
     */
    irq_primask_disable();

    if (delay_ticks <= 0xFFFF) {
        _remaining_ticks = 0;
    } else {
        _remaining_ticks = delay_ticks - 0xFFFF;
        delay_ticks = 0xFFFF;
    }

    if (delay_hnsec > HNSEC_PER_USEC) {
        TIM14->CCR1 = TIM_GetCounter(TIM14) + delay_ticks;
        TIM14->SR = ~TIM_SR_CC1IF;             // Acknowledge IRQ
        TIM14->DIER |= TIM_DIER_CC1IE;         // Enable this compare match
    } else {
        // Force the update event immediately because the delay is too small
        TIM14->DIER |= TIM_DIER_CC1IE;  // Either here or at the next statement IRQ will be generated
        TIM14->EGR = TIM_EGR_CC1G;
    }

    irq_primask_enable();
}

// 设置绝对的延时时间(亚纳秒),延时结束后回调对应函数,返回增量时间
int64_t motor_timer_set_absolute(uint64_t timestamp_hnsec)
{
    // 获取当前时间戳
    const uint64_t current_timestamp = motor_timer_hnsec();
    // 计算增量时间 = 设定的绝对时间戳 - 当前时间戳
    const int64_t delta = (int64_t)timestamp_hnsec - (int64_t)current_timestamp;
    // 判断增量时间是否有效
    if (delta > 0) {
        motor_timer_set_relative(delta);
    } else {
        motor_timer_set_relative(0);
    }

    // 返回增量时间
    return delta;
}

// 取消定时器14的定时回调任务
void motor_timer_cancel(void)
{
    TIM_ITConfig(TIM14, TIM_IT_CC1, DISABLE);// Disable this compare match 失能该捕获匹配
    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);// 清零捕捉中断标志
}

// 延时函数, 延时 "hnsecs" 亚微妙
void motor_timer_hndelay(int hnsecs)
{
    static const int OVERHEAD_HNSEC = 1 * HNSEC_PER_USEC;// 1微秒 = 10亚微秒
    if (hnsecs > OVERHEAD_HNSEC) {
        hnsecs -= OVERHEAD_HNSEC;
    } else {
        hnsecs = 0;
    }

    // 计算延时后的到期时间 = 当前时间 + 所延时时间
    const uint64_t deadline = motor_timer_hnsec() + hnsecs;
    // 死循环进行亚微妙延时
    while (motor_timer_hnsec() < deadline);
}
