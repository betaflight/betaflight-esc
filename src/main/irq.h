#pragma once

#include <stdint.h>
#include <assert.h>

// PRIMASK置位(写1), 开启屏蔽,屏蔽除了NMI和硬件错误(hardfault)外的所有中断. 清除此位,关闭屏蔽.
typedef uint32_t irqstate_t;

static inline irqstate_t irq_primask_save(void)
{
    const irqstate_t primask = __get_PRIMASK();
    __set_PRIMASK(1);
    return primask;
}

static inline void irq_primask_restore(irqstate_t state)
{
    __set_PRIMASK(state);
}

// 屏蔽全部中断
static inline void irq_primask_disable(void)
{
    __set_PRIMASK(1);
}

// 接触屏蔽全部中断
static inline void irq_primask_enable(void) {
    __set_PRIMASK(0);
}
