#include "include.h"

void led_init(void)
{
    LL_AHB1_GRP1_EnableClock(LED0_PERIPHERAL | LED1_PERIPHERAL | LED2_PERIPHERAL);

    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;

    GPIO_InitStructure.Pin = LED0_PIN;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(LED0_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = LED1_PIN;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(LED1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = LED2_PIN;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(LED2_GPIO, &GPIO_InitStructure);

    LED0_OFF();
    LED1_OFF();
    LED2_OFF();
}
