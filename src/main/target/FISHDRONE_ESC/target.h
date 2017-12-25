#pragma once

// PB8 --- BLUE
#define LED0_GPIO       GPIOB
#define LED0_PIN        LL_GPIO_PIN_8
#define LED0_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PB3 --- RED
#define LED1_GPIO       GPIOB
#define LED1_PIN        LL_GPIO_PIN_3
#define LED1_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PB5 --- GREEN
#define LED2_GPIO       GPIOB
#define LED2_PIN        LL_GPIO_PIN_5
#define LED2_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PA10 --- Phase_A_HI
#define A_FET_HI_GPIO GPIOA
#define A_FET_HI_PIN LL_GPIO_PIN_10

// PA9 --- Phase_B_HI
#define B_FET_HI_GPIO GPIOA
#define B_FET_HI_PIN LL_GPIO_PIN_9

// PA8 --- Phase_C_HI
#define C_FET_HI_GPIO GPIOA
#define C_FET_HI_PIN LL_GPIO_PIN_8

// PB1 --- Phase_A_LO
#define A_FET_LO_GPIO GPIOB
#define A_FET_LO_PIN LL_GPIO_PIN_1

// PB0 --- Phase_B_LO
#define B_FET_LO_GPIO GPIOB
#define B_FET_LO_PIN LL_GPIO_PIN_0

// PA7 --- Phase_C_LO
#define C_FET_LO_GPIO GPIOA
#define C_FET_LO_PIN LL_GPIO_PIN_7

#define AFetHiOff()			LL_GPIO_ResetOutputPin(A_FET_HI_GPIO, A_FET_HI_PIN);
#define AFetHiOn()			LL_GPIO_SetOutputPin(A_FET_HI_GPIO, A_FET_HI_PIN);

#define BFetHiOff()			LL_GPIO_ResetOutputPin(B_FET_HI_GPIO, B_FET_HI_PIN);
#define BFetHiOn()			LL_GPIO_SetOutputPin(B_FET_HI_GPIO, B_FET_HI_PIN);

#define CFetHiOff()			LL_GPIO_ResetOutputPin(C_FET_HI_GPIO, C_FET_HI_PIN);
#define CFetHiOn()			LL_GPIO_SetOutputPin(C_FET_HI_GPIO, C_FET_HI_PIN);

#define AFetLoOff()			LL_GPIO_ResetOutputPin(A_FET_LO_GPIO, A_FET_LO_PIN);
#define AFetLoOn()			LL_GPIO_SetOutputPin(A_FET_LO_GPIO, A_FET_LO_PIN);

#define BFetLoOff()			LL_GPIO_ResetOutputPin(B_FET_LO_GPIO, B_FET_LO_PIN);
#define BFetLoOn()			LL_GPIO_SetOutputPin(B_FET_LO_GPIO, B_FET_LO_PIN);

#define CFetLoOff()			LL_GPIO_ResetOutputPin(C_FET_LO_GPIO, C_FET_LO_PIN);
#define CFetLoOn()			LL_GPIO_SetOutputPin(C_FET_LO_GPIO, C_FET_LO_PIN);
