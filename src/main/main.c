#include "include.h"

struct Timer timerLedUpdate;

// AC -> AB -> CB -> CA -> BA -> BC
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_FORWARD[MOTOR_NUM_COMMUTATION_STEPS] = {
    {1, 0, 2},      // BA通电,B相上管注入PWM,A相下管直通,检测C相过零
    {1, 2, 0},      // BC通电,B相上管注入PWM,C相下管直通,检测A相过零
    {0, 2, 1},      // AC通电,A相上管注入PWM,C相下管直通,检测B相过零
    {0, 1, 2},      // AB通电,A相上管注入PWM,B相下管直通,检测C相过零
    {2, 1, 0},      // CB通电,C相上管注入PWM,B相下管直通,检测A相过零
    {2, 0, 1}       // CA通电,C相上管注入PWM,A相下管直通,检测B相过零
};

static void motorTestTask(void)
{
    static int current_comm_step;
    motor_pwm_set_step_and_pwm(COMMUTATION_TABLE_FORWARD + current_comm_step, 100);
    // 六步换相 自增一步(0 1 2 3 4 5)
    current_comm_step++;
    // 如果大于等于6步则直接清零,重新进行六步换相
    if (current_comm_step >= MOTOR_NUM_COMMUTATION_STEPS) {
        current_comm_step = 0;
    }
}

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
    // 初始化系统
    system_init();

    // 串口初始化
    serialInit();

    // 读取EEPROM中的数据
    ensureEEPROMContainsValidData();
    readEEPROM();

    // 灯初始化
    led_init();

    // 1.输入信号模块初始化
    motor_signal_init();

    // 2.定时器模块初始化
    motor_timer_init();

    // 3.电机PWM初始化
    motor_pwm_init();

    // 4.比较器模块初始化
    motor_comparator_init();

    // 5.初始化电机检测ADC值
    motor_adc_init();

    // 初始化定时器对象,注册定时器回调处理函数,设置定时时间(ms),循环定时触发时间
    timer_init(&timerLedUpdate, timer_led_callback, 0, 100);
    timer_start(&timerLedUpdate);

    while(1) {
        // 调度器调度
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
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
