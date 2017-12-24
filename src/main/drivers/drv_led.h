#pragma once

#include "include.h"

#define LED0_TOGGLE()    LL_GPIO_TogglePin(LED0_GPIO, LED0_PIN)
#define LED0_OFF()       LL_GPIO_SetOutputPin(LED0_GPIO, LED0_PIN)
#define LED0_ON()        LL_GPIO_ResetOutputPin(LED0_GPIO, LED0_PIN)

#define LED1_TOGGLE()    LL_GPIO_TogglePin(LED1_GPIO, LED1_PIN)
#define LED1_OFF()       LL_GPIO_SetOutputPin(LED1_GPIO, LED1_PIN)
#define LED1_ON()        LL_GPIO_ResetOutputPin(LED1_GPIO, LED1_PIN)

#define LED2_TOGGLE()    digitLL_GPIO_TogglePinalToggle(LED2_GPIO, LED2_PIN)
#define LED2_OFF()       LL_GPIO_SetOutputPin(LED2_GPIO, LED2_PIN)
#define LED2_ON()        LL_GPIO_ResetOutputPin(LED2_GPIO, LED2_PIN)

void led_init(void);
