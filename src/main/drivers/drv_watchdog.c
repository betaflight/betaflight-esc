#include "include.h"

void watchdog_init(uint8_t timeout_s)
{
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    IWDG_SetPrescaler(4);

    IWDG_SetReload(timeout_s * 625);

    IWDG_ReloadCounter();

    IWDG_Enable();
}

void watchdog_feed(void)
{
    IWDG_ReloadCounter();
}
