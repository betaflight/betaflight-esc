#include "include.h"

static volatile uint32_t sysTickUptime = 0;

void nvicEnableVector(IRQn_Type irq, uint8_t prio)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPriority = prio;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SysTick_Handler(void)
{
    sysTickUptime++;
    timer_ticks();
}

void system_init(void)
{
    // Systick滴答定时器
    SysTick_Config(SystemCoreClock / 1000);
}

void system_reset(void)
{
    NVIC_SystemReset();
}

uint32_t system_ticks(void)
{
    return sysTickUptime;
}

void delayMs(uint32_t nMs)
{
    const uint32_t now = system_ticks();
    while (system_ticks() - now < nMs);
}

void delayUs(uint64_t nUs)
{
    motor_timer_hndelay(nUs * HNSEC_PER_USEC);
}
