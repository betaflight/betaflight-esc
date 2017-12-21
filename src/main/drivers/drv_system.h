#pragma once

void system_init(void);
void system_reset(void);

uint32_t system_ticks(void);

void nvicEnableVector(IRQn_Type irq, uint8_t prio);

void delayMs(uint32_t nMs);
void delayUs(uint64_t nUs);
