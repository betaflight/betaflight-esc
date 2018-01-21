#include "include.h"

#include "drv_led.h"

void led_init(void)
{
    LL_AHB1_GRP1_EnableClock(LED0_PERIPHERAL | LED1_PERIPHERAL | LED2_PERIPHERAL);

    LL_GPIO_InitTypeDef gpioInit;
    gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
    gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpioInit.Pull = LL_GPIO_PULL_NO;
    gpioInit.Alternate = LL_GPIO_AF_0;

    gpioInit.Pin = LED0_PIN;
    LL_GPIO_Init(LED0_GPIO, &gpioInit);

    gpioInit.Pin = LED1_PIN;
    LL_GPIO_Init(LED1_GPIO, &gpioInit);

    gpioInit.Pin = LED2_PIN;
    LL_GPIO_Init(LED2_GPIO, &gpioInit);

    LED0_OFF();
    LED1_OFF();
    LED2_OFF();
}
