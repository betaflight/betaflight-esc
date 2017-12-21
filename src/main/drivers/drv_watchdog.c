#include "include.h"

/**
* 看门狗初始化接口(定时10秒) LSI:40kHz
 * Tout = (4*2^prv) / 40 * rlv (s) , prv是预分频器寄存器的值, rlv是重装载寄存器的值
 */
void watchdog_init(uint8_t timeout_s)
{
    // 使能 预分频寄存器 PR 和重装载寄存器 RLR 可写
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // 设置预分频器值
    IWDG_SetPrescaler(4);

    // 设置重装载寄存器值
    IWDG_SetReload(timeout_s * 625);// 最大4095

    // 把重装载寄存器的值放到计数器中
    /* Reload IWDG counter */
    IWDG_ReloadCounter();

    // 使能 IWDG
    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
}

/**
 * 看门狗喂狗接口
 */
void watchdog_feed(void)
{
    // 把重装载寄存器的值放到计数器中，喂狗，防止 IWDG 复位。当计数器的值减到 0 的时候会产生系统复位
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
}
