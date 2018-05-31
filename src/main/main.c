/*
 * This file is part of Betaflight-ESC (BFESC).
 *
 * Betaflight-ESC is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight-ESC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight-ESC.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "include.h"

#include "config/cli.h"

#include "drivers/drv_led.h"
#include "drivers/drv_uart.h"

void _init(void)
{
    //SystemInit();
}

struct Timer ledTimer;
struct Timer consoleTimer;

void timer_led_callback(void)
{
    LED0_TOGGLE();
}



int main(void)
{
    HAL_Init();
    SystemCoreClockUpdate();
    
    system_init();
    
    serialInit();

    ensureEEPROMContainsValidData();
    readEEPROM();

    led_init();
    //motor_signal_init();

    motor_pwm_init();

    //motor_comparator_init();

    motor_adc_init();
    printf("%s %d.%d.%d\r\n", FW_FIRMWARE_NAME, FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH_LEVEL);
    printf("Build time %s %s\r\n",__DATE__, __TIME__ );



    timer_init(&ledTimer, timer_led_callback, 0, 500);
    timer_start(&ledTimer);

    timer_init(&consoleTimer, cliProcess, 0, 100);
    timer_start(&consoleTimer);
    LED0_ON();
    while(1) {
        timer_loop();
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    printf("Wrong parameters value: file %s on line %d\r\n", file, (int)line);

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
