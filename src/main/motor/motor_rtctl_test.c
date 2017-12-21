#include "include.h"

static const int INITIAL_DELAY_MS = 300;
static const int SAMPLE_DELAY_MS  = 10;

static const int ANALOG_TOLERANCE_PERCENT = 5;

/**
 * 测试单相
 */
static int test_one_phase(int phase, bool level)
{
    // 参数有效性判断
    assert_param(phase >= 0 && phase < MOTOR_NUM_PHASES);

    // 三相浮空
    enum motor_pwm_phase_manip manip_cmd[MOTOR_NUM_PHASES] = {
        MOTOR_PWM_MANIP_FLOATING,
        MOTOR_PWM_MANIP_FLOATING,
        MOTOR_PWM_MANIP_FLOATING
    };

    manip_cmd[phase] = level ? MOTOR_PWM_MANIP_HIGH : MOTOR_PWM_MANIP_LOW;
    motor_pwm_manip(manip_cmd);
    // 延迟一段时间
    // delayMs(SAMPLE_DELAY_MS);
    // 获取电压
    const int sample = motor_adc_get_last_sample().phase_values[phase];

    manip_cmd[phase] = MOTOR_PWM_MANIP_FLOATING;
    motor_pwm_manip(manip_cmd);
    return sample;
}

static int compare_samples(const void* p1, const void* p2)
{
    return (*(const int*)p1 - *(const int*)p2);
}

/**
 * 测试传感器
 */
static int test_sensors(void)
{
    static const int ADC_MAX = (1U << MOTOR_ADC_RESOLUTION) - 1;

    /*
     * Enable the synchronous PWM on all phases, obtain a sample and disable PWM again
     */
    const enum motor_pwm_phase_manip manip_cmd[MOTOR_NUM_PHASES] = {
        MOTOR_PWM_MANIP_HALF,
        MOTOR_PWM_MANIP_HALF,
        MOTOR_PWM_MANIP_HALF
    };
    motor_pwm_manip(manip_cmd);
    //  delayMs(INITIAL_DELAY_MS);

    const struct motor_adc_sample sample = motor_adc_get_last_sample();

    motor_pwm_set_freewheeling();

    /*
     * Validate the obtained sample
     */
    const bool valid_voltage = (sample.input_voltage > 0) && (sample.input_voltage < ADC_MAX);
    const bool valid_current = (sample.input_current > 0) && (sample.input_current < ADC_MAX);

    if (!valid_voltage || !valid_current) {
        printf("Motor: Invalid sensor readings: raw input voltage %i, raw input current %i\n", sample.input_voltage, sample.input_current);
        return 1;
    }

    printf("Motor: Raw input voltage %i, raw input current %i\n", sample.input_voltage, sample.input_current);
    return 0;
}

/**
 * Sets high/low levels on the output FETs, reads ADC samples making sure that there are proper corellations.
 */
static int test_power_stage(void)
{
    int result = 0;
    int high_samples[MOTOR_NUM_PHASES];
    memset(high_samples, 0, sizeof(high_samples));

    const int threshold = ((1 << MOTOR_ADC_RESOLUTION) * ANALOG_TOLERANCE_PERCENT) / 100;

    motor_pwm_set_freewheeling();

    /*
     * Test phases at low level; collect high level readings
     */
    for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
        // Low level
        const int low = test_one_phase(phase, false);
        if (low > threshold) {
            printf("Motor: Selftest FAILURE at phase %i: low sample %i is above threshold %i\n",
                   phase, low, threshold);
            result++;
        }

        // High level
        const int high = test_one_phase(phase, true);
        high_samples[phase] = high;
        if (high < threshold) {
            printf("Motor: Selftest FAILURE at phase %i: high sample %i is below threshold %i\n",
                   phase, high, threshold);
            result++;
        }
        // It is not possible to check against the high threshold directly
        // because its value will depend on the supply voltage

        printf("Motor: Selftest phase %i: low %i, high %i\n", phase, low, high);
    }

    /*
     * Make sure that the high level readings are nearly identical
     */
    int high_samples_sorted[MOTOR_NUM_PHASES];
    memcpy(high_samples_sorted, high_samples, sizeof(high_samples));
    qsort(high_samples_sorted, MOTOR_NUM_PHASES, sizeof(int), compare_samples);
    const int high_median = high_samples_sorted[MOTOR_NUM_PHASES / 2];

    for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
        if (abs(high_samples[phase] - high_median) > threshold) {
            printf("Motor: Selftest FAILURE at phase %i: sample %i is far from median %i\n",
                   phase, high_samples[phase], high_median);
            result++;
        }
    }

    motor_pwm_set_freewheeling();
    return result;
}

/**
 * Detects cross-phase short circuit
 */
static int test_cross_phase_conductivity(void)
{
    int num_detects = 0;

    for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
        // Apply the high level voltage to the selected phase
        enum motor_pwm_phase_manip manip_cmd[MOTOR_NUM_PHASES] = {
            MOTOR_PWM_MANIP_FLOATING,
            MOTOR_PWM_MANIP_FLOATING,
            MOTOR_PWM_MANIP_FLOATING
        };
        manip_cmd[phase] = MOTOR_PWM_MANIP_HIGH;

        motor_pwm_set_freewheeling();
        //    delayMs(SAMPLE_DELAY_MS);
        motor_pwm_manip(manip_cmd);
        //  delayMs(SAMPLE_DELAY_MS);
        motor_pwm_set_freewheeling();

        // Read back the full ADC sample
        const struct motor_adc_sample sample = motor_adc_get_last_sample();

        // Make sure only one phase is at the high level; other must be at low level
        const int low_first  = (phase + 1) % MOTOR_NUM_PHASES;
        const int low_second = (phase + 2) % MOTOR_NUM_PHASES;
        assert_param((low_first != phase) && (low_second != phase) && (low_first != low_second));

        const int low_sum = sample.phase_values[low_first] + sample.phase_values[low_second];

        const bool valid = (low_sum * 2) < sample.phase_values[phase];

        if (!valid) {
            num_detects++;
            printf("Motor: Phase %i cross conductivity: %i %i %i\n", phase,
                   sample.phase_values[0], sample.phase_values[1], sample.phase_values[2]);
        }
    }

    motor_pwm_set_freewheeling();

    if (num_detects == MOTOR_NUM_PHASES) {
        printf("Motor: All phases are shorted, assuming the motor is connected\n");
        num_detects = 0;
    }

    return num_detects;
}

/**
 * 测试硬件
 */
int motor_rtctl_test_hardware(void)
{
    // 当前状态应为空闲状态
    if (motor_rtctl_get_state() != MOTOR_RTCTL_STATE_IDLE) {
        return -1;
    }

    motor_pwm_set_freewheeling();
//   delayMs(INITIAL_DELAY_MS);

    printf("Motor: Power stage test...\n");
    {
        int res = test_power_stage();// 测试电机
        if (res != 0) {
            return res;
        }
    }

    printf("Motor: Cross phase test...\n");
    {
        int res = test_cross_phase_conductivity();// 测试换相
        if (res != 0) {
            return res;
        }
    }

    printf("Motor: Sensors test...\n");
    {
        int res = test_sensors();// 测试传感器
        if (res != 0) {
            return res;
        }
    }

    return 0;
}

/**
 * 测试电机是否接入
 */
int motor_rtctl_test_motor(void)
{
    // 当前状态应为空闲状态
    if (motor_rtctl_get_state() != MOTOR_RTCTL_STATE_IDLE) {
        return -1;
    }

    // 计算ADC门槛电压值
    const int threshold = ((1 << MOTOR_ADC_RESOLUTION) * ANALOG_TOLERANCE_PERCENT) / 100;
    // ADC采样结构体
    struct motor_adc_sample sample;
    // 结果值
    int result = 0;
    enum motor_pwm_phase_manip manip_cmd[MOTOR_NUM_PHASES] = {
        MOTOR_PWM_MANIP_LOW,
        MOTOR_PWM_MANIP_FLOATING,
        MOTOR_PWM_MANIP_FLOATING
    };

    // 电机惯性滑行,不受PWM电压控制
    motor_pwm_set_freewheeling();
    /*
     * Test with low level
     */
    manip_cmd[0] = MOTOR_PWM_MANIP_LOW;
    motor_pwm_manip(manip_cmd);
    // delayMs(SAMPLE_DELAY_MS);
    sample = motor_adc_get_last_sample();

    if (sample.phase_values[1] > threshold || sample.phase_values[2] > threshold) {
        result++;
    }

    /*
     * Test with high level
     */
    manip_cmd[0] = MOTOR_PWM_MANIP_HIGH;
    // 人为外加PWM方波
    motor_pwm_manip(manip_cmd);
    // 延迟一段时间
    // delayMs(SAMPLE_DELAY_MS);
    // ADC采样
    sample = motor_adc_get_last_sample();

    if (abs(sample.phase_values[0] - sample.phase_values[1]) > threshold) {
        result++;
    }
    if (abs(sample.phase_values[0] - sample.phase_values[2]) > threshold) {
        result++;
    }

    // 电机惯性滑行,不受PWM电压控制
    motor_pwm_set_freewheeling();
    return result;
}
