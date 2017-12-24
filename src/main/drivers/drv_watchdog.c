#include "include.h"

#include "stm32f0xx_ll_iwdg.h"

void watchdog_init(uint8_t timeout_s)
{
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    LL_IWDG_EnableWriteAccess(IWDG);

    LL_IWDG_SetPrescaler(IWDG, 4);

    LL_IWDG_SetReloadCounter(IWDG, timeout_s * 625);

    LL_IWDG_ReloadCounter(IWDG);

    LL_IWDG_Enable(IWDG);
}

void watchdog_feed(void)
{
    LL_IWDG_ReloadCounter(IWDG);
}
