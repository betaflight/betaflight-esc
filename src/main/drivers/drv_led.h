#pragma once

// PB8 --- BLUE
#define LED_BLUE_GPIO   GPIOB
#define LED_BLUE_PIN    GPIO_Pin_8
#define LED_BLUE_PERIPHERAL RCC_AHBPeriph_GPIOB

// PB3 --- RED
#define LED_RED_GPIO   GPIOB
#define LED_RED_PIN    GPIO_Pin_3
#define LED_RED_PERIPHERAL RCC_AHBPeriph_GPIOB

// PB5 --- GREEN
#define LED_GREEN_GPIO   GPIOB
#define LED_GREEN_PIN    GPIO_Pin_5
#define LED_GREEN_PERIPHERAL RCC_AHBPeriph_GPIOB

#define LED_RED_TOGGLE()    digitalToggle(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()       digitalHi(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_ON()        digitalLo(LED_RED_GPIO, LED_RED_PIN)

#define LED_GREEN_TOGGLE()  digitalToggle(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()     digitalHi(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_ON()      digitalLo(LED_GREEN_GPIO, LED_GREEN_PIN)

#define LED_BLUE_TOGGLE()   digitalToggle(LED_BLUE_GPIO, LED_BLUE_PIN)
#define LED_BLUE_OFF()      digitalHi(LED_BLUE_GPIO, LED_BLUE_PIN)
#define LED_BLUE_ON()       digitalLo(LED_BLUE_GPIO, LED_BLUE_PIN)

static inline void digitalHi(GPIO_TypeDef *p, uint16_t i) {
    p->BSRR = i;
}

static inline void digitalLo(GPIO_TypeDef *p, uint16_t i)     {
    p->BRR = i;
}

static inline void digitalToggle(GPIO_TypeDef *p, uint16_t i) {
    p->ODR ^= i;
}

void led_init(void);
