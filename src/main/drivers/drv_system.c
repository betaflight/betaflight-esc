#include "include.h"

#include "stm32f0xx_hal_cortex.h"

static volatile uint32_t sysTickUptime = 0;

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI48)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 48000000
  *            PREDIV                         = 2
  *            PLLMUL                         = 2
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    /* Enable HSI48 and wait for activation*/
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1);

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    /* Set APB1 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    /* Set systick to 1ms in using frequency set to 48MHz */
    /* This frequency can be calculated through LL RCC macro */
    /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI48_VALUE, LL_RCC_PLL_MUL_2, LL_RCC_PREDIV_DIV_2) */
    LL_Init1msTick(48000000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(48000000);
}

void nvicEnableVector(IRQn_Type irq, uint8_t priority)
{
    HAL_NVIC_SetPriority(irq, priority ,0U);
    HAL_NVIC_EnableIRQ(irq);
}

void SysTick_Handler(void)
{
    sysTickUptime++;
    timer_ticks();
}

void system_init(void)
{
	SystemClock_Config();
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
