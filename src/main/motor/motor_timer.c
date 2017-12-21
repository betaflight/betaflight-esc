#include "include.h"

/**
 * The timer frequency is a compromise between maximum delay and timer resolution
 * 2 MHz --> ~32ms max
 * 4 MHz --> ~16ms max
 * 5 MHz --> ~13ms max
 * Note that we build a postscaler on top of the hardware timer that allows us to generate arbitrarily long delays
 * regardless of the hardware capabilities.
 */
static const int MAX_FREQUENCY = 4000000;

static const uint64_t INT_1E9 = 1000000000ul;
static const uint32_t TICKS_PER_OVERFLOW = 0xFFFF + 1;

// nanoSecond 
static uint32_t _nanosec_per_tick = 0;
static volatile uint64_t _raw_ticks = 0;
static volatile int64_t _remaining_ticks = 0;

/**
 * Event callout timer declaration
 * ------->>>>> Hard real time callout interface for motor control logic (preempts the kernel)
 */
void TIM14_IRQHandler(void)
{
    if ((TIM14->SR & TIM_SR_CC1IF) && (TIM14->DIER & TIM_DIER_CC1IE)) {
        TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);

        if (_remaining_ticks <= (MAX_FREQUENCY / 1000000)) {// MIN_REMAINING_TICKS
            TIM_ITConfig(TIM14, TIM_IT_CC1, DISABLE);// Disable this compare match 
            const uint64_t timestamp = motor_timer_hnsec() - 2;
            motor_timer_callback(timestamp);
        } else {
            if (_remaining_ticks >= TICKS_PER_OVERFLOW) {
                // We don't need to re-configure the CCR register - the value won't change
                _remaining_ticks -= TICKS_PER_OVERFLOW; // Go around 
            } else {
                TIM14->CCR1 += _remaining_ticks;
                _remaining_ticks = 0;                   // Go around the last time
            }
        }
    }
}

/**
 * Timestamping timer declaration
 * ------->>>>> High precision timestamping for motor control logic (sub-microsecond resolution, never overflows)
 */
void TIM17_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM17, TIM_IT_Update);

    _raw_ticks += TICKS_PER_OVERFLOW;
}

void motor_timer_init(void)
{
    /* TIM14 & TIM17 clock Power-on and reset */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM14, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM14, DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM17, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM17, DISABLE);

    // Find the optimal prescaler value = 48Mhz / 12Mhz = 4
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

    _nanosec_per_tick = INT_1E9 / (SystemCoreClock / prescaler);// 1000000000/4=250

    printf("Motor: Timer resolution: %u nanosec, %u\n", (unsigned)_nanosec_per_tick, (unsigned)prescaler);

    /* Enable the TIM14 & TIM17 gloabal Interrupt */
    nvicEnableVector(TIM14_IRQn, 0);
    nvicEnableVector(TIM17_IRQn, 0);

    /* Time base configuration */
    TIM14->ARR = 0xFFFF;
    TIM14->PSC = (uint16_t)(prescaler - 1);
    TIM14->CR1 = TIM_CR1_URS;
    TIM14->SR  = 0;
    TIM14->EGR = TIM_EGR_UG;     // Reload immediately
    TIM14->CR1 = TIM_CR1_CEN;    // Start

    // Start the timestamping timer
    TIM17->ARR = 0xFFFF;
    TIM17->PSC = (uint16_t)(prescaler - 1);
    TIM17->CR1 = TIM_CR1_URS;
    TIM17->SR  = 0;
    TIM17->EGR = TIM_EGR_UG;     // Reload immediately
    TIM17->DIER = TIM_DIER_UIE;
    TIM17->CR1 = TIM_CR1_CEN;    // Start
}

uint64_t motor_timer_hnsec(void)
{
    volatile uint64_t ticks = 0;
    volatile uint_fast16_t sample = 0;

    while (1) {
        ticks = _raw_ticks;
        sample = TIM_GetCounter(TIM17);

        const volatile uint64_t ticks2 = _raw_ticks;

        if (ticks == ticks2) {
            if (TIM_GetFlagStatus(TIM17, TIM_FLAG_Update)) {
                sample = TIM_GetCounter(TIM17);
                ticks += TICKS_PER_OVERFLOW;
            }
            break;
        }
    }

    return ((ticks + sample) * _nanosec_per_tick) / 100;
}

void motor_timer_set_relative(int64_t delay_hnsec)
{
    delay_hnsec -= 1 * HNSEC_PER_USEC;
    if (delay_hnsec < 0) {
        delay_hnsec = 0;
    }

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

int64_t motor_timer_set_absolute(uint64_t timestamp_hnsec)
{
    const uint64_t current_timestamp = motor_timer_hnsec();
    const int64_t delta = (int64_t)timestamp_hnsec - (int64_t)current_timestamp;
    if (delta > 0) {
        motor_timer_set_relative(delta);
    } else {
        motor_timer_set_relative(0);
    }

    return delta;
}

void motor_timer_cancel(void)
{
    TIM_ITConfig(TIM14, TIM_IT_CC1, DISABLE);// Disable this compare match 
    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
}

void motor_timer_hndelay(int hnsecs)
{
    static const int OVERHEAD_HNSEC = 1 * HNSEC_PER_USEC;
    if (hnsecs > OVERHEAD_HNSEC) {
        hnsecs -= OVERHEAD_HNSEC;
    } else {
        hnsecs = 0;
    }
    
    const uint64_t deadline = motor_timer_hnsec() + hnsecs;
    while (motor_timer_hnsec() < deadline);
}
