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

#include "include.h"

void _init(void)
{
    SystemInit();
}

struct Timer timerLedUpdate;

/*
// AC -> AB -> CB -> CA -> BA -> BC
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_FORWARD[MOTOR_NUM_COMMUTATION_STEPS] = {
    {1, 0, 2},
    {1, 2, 0},
    {0, 2, 1},
    {0, 1, 2},
    {2, 1, 0},
    {2, 0, 1}
};

static void motorTestTask(void)
{
    static int current_comm_step;
    motor_pwm_set_step_and_pwm(COMMUTATION_TABLE_FORWARD + current_comm_step, 100);
    current_comm_step++;
    
    if (current_comm_step >= MOTOR_NUM_COMMUTATION_STEPS) {
        current_comm_step = 0;
    }
}
*/

void timer_led_callback(void)
{
    LED_RED_TOGGLE();
    // update_voltage_current_temperate();
    // printf("ticks:%llu\n",motor_timer_hnsec());
    // printf("temp:%f\n",motor_adc_get_temperature());
    // motorTestTask();
}

int main(void)
{
    system_init();

    serialInit();

    ensureEEPROMContainsValidData();
    readEEPROM();

    led_init();

    motor_signal_init();

    motor_timer_init();

    motor_pwm_init();

    motor_comparator_init();

    motor_adc_init();

    timer_init(&timerLedUpdate, timer_led_callback, 0, 100);
    timer_start(&timerLedUpdate);

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
