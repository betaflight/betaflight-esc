#include "include.h"

void led_init(void)
{
    RCC_AHBPeriphClockCmd(LED_RED_PERIPHERAL | LED_GREEN_PERIPHERAL | LED_BLUE_PERIPHERAL, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
    GPIO_Init(LED_RED_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_GREEN_PIN;
    GPIO_Init(LED_GREEN_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
    GPIO_Init(LED_BLUE_GPIO, &GPIO_InitStructure);

    LED_GREEN_OFF();
    LED_BLUE_OFF();
    LED_RED_OFF();
}
