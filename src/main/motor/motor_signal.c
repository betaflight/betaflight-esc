#include "include.h"

uint32_t newInput;

uint16_t computeDshotThrottle(uint32_t inputBuffer[32])
{
    static uint16_t throttle = 0;

    uint8_t dshotBits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t frameLength = inputBuffer[30] - inputBuffer[0];
    uint32_t DshotBitWidth = frameLength >> 4;
    uint32_t DshotBitOne = DshotBitWidth - (DshotBitWidth >> 2);
    uint32_t DshotBitZero = DshotBitOne >> 1;
    uint32_t DshotBitMargin = DshotBitWidth >> 3;

    for(uint8_t i = 0; i < 16; i++) {
        uint32_t bitDiff = inputBuffer[i*2+1] - inputBuffer[i*2];
        if(bitDiff < DshotBitZero - DshotBitMargin || bitDiff > DshotBitOne + DshotBitMargin) {
            return throttle;
        } else if(bitDiff > DshotBitZero + DshotBitMargin) {
            dshotBits[15-i] = 1;
        }
    }

    uint8_t checkCRC = ((dshotBits[15]^dshotBits[11]^dshotBits[7]) << 3
                        |(dshotBits[14]^dshotBits[10]^dshotBits[6]) << 2
                        |(dshotBits[13]^dshotBits[9]^dshotBits[5]) << 1
                        |(dshotBits[12]^dshotBits[8]^dshotBits[4]) << 0);

    uint8_t recCRC = (dshotBits[3] << 3
                      | dshotBits[2] << 2
                      | dshotBits[1] << 1
                      | dshotBits[0] << 0);

    if(checkCRC == recCRC) {
        throttle = (dshotBits[15] << 10
                    | dshotBits[14] << 9
                    | dshotBits[13] << 8
                    | dshotBits[12] << 7
                    | dshotBits[11] << 6
                    | dshotBits[10] << 5
                    | dshotBits[9] << 4
                    | dshotBits[8] << 3
                    | dshotBits[7] << 2
                    | dshotBits[6] << 1
                    | dshotBits[5] << 0);
    }

    return throttle;
}

static motorPwmProtocolTypes_e pwmProtocolType = PWM_TYPE_NONE;

// PB4:TIM3_IN1
void pwm_input_init(void)
{
    /* GPIOB clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /* TIM3 chennel1 configuration : PB4 */
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin   = LL_GPIO_PIN_4;
    GPIO_InitStructure.Mode  = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull  = LL_GPIO_PULL_UP ;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM pin to AF1 */
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_1);

    /* TIM3 clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    /* Enable the TIM3 global Interrupt */
    nvicEnableVector(TIM3_IRQn, 2);

    LL_TIM_InitTypeDef TIM_TimeBaseStructure;
    LL_TIM_StructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Prescaler = 48-1;
    TIM_TimeBaseStructure.Autoreload = 65535;
    TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
    // TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    LL_TIM_Init(TIM3, &TIM_TimeBaseStructure);

    LL_TIM_IC_InitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.ICPolarity = LL_TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = LL_TIM_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = 0x04;

    uint16_t tim_icoppositepolarity_8 = (uint16_t)LL_TIM_IC_POLARITY_RISING;
    uint16_t tim_icoppositeselection_8 = (uint16_t)LL_TIM_ACTIVEINPUT_DIRECTTI;
    /* Select the Opposite Input Polarity */
    if (TIM_ICInitStructure.ICPolarity == LL_TIM_IC_POLARITY_RISING) {
        tim_icoppositepolarity_8 = (uint16_t)LL_TIM_IC_POLARITY_FALLING;
    } else {
        tim_icoppositepolarity_8 = (uint16_t)LL_TIM_IC_POLARITY_RISING;
    }
    /* Select the Opposite Input */
    if (TIM_ICInitStructure.ICActiveInput == LL_TIM_ACTIVEINPUT_DIRECTTI) {
        tim_icoppositeselection_8 = (uint16_t)LL_TIM_ACTIVEINPUT_INDIRECTTI;
    } else {
        tim_icoppositeselection_8 = (uint16_t)LL_TIM_ACTIVEINPUT_INDIRECTTI;
    }
    /* TI1 Configuration */
    LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, TIM_ICInitStructure.ICActiveInput);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, TIM_ICInitStructure.ICFilter);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, TIM_ICInitStructure.ICPolarity);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    /* Set the Input Capture Prescaler value */
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, TIM_ICInitStructure.ICPrescaler);
    /* TI2 Configuration */
    LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, tim_icoppositeselection_8);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, TIM_ICInitStructure.ICFilter);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, tim_icoppositepolarity_8);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    /* Set the Input Capture Prescaler value */
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, TIM_ICInitStructure.ICPrescaler);

    /* Select the TIM3 Input Trigger: TI1FP1 */
    LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_TI1FP1);

    /* Select the slave Mode: Reset Mode */
    LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_RESET);
    LL_TIM_EnableMasterSlaveMode(TIM3);

    /* TIM enable counter */
    LL_TIM_EnableCounter(TIM3);

    /* Enable the CC1 Interrupt Request */
    LL_TIM_EnableIT_CC1(TIM3);
}

static __IO uint16_t throttle = 0;

void TIM3_IRQHandler(void)
{
    /* Clear TIM3 Capture compare interrupt pending bit */
    LL_TIM_ClearFlag_CC1(TIM3);

    if (pwmProtocolType == PWM_TYPE_NONE) {
        const uint16_t pulseWidth = LL_TIM_IC_GetCaptureCH2(TIM3);

        if (950 <= pulseWidth && pulseWidth <= 2050) { // 1000Us - 2000Us
            pwmProtocolType = PWM_TYPE_STANDARD;
        } else if (120 <= pulseWidth && pulseWidth <= 255) { // 125Us - 250Us
            pwmProtocolType = PWM_TYPE_ONESHOT125;
        } else if (40 <= pulseWidth && pulseWidth <= 86) { // 42Us - 84Us
            pwmProtocolType = PWM_TYPE_ONESHOT42;
        } else if (2 <= pulseWidth && pulseWidth <= 28) { // 5Us - 25Us
            pwmProtocolType = PWM_TYPE_MULTISHOT;
        } else {
            pwmProtocolType = PWM_TYPE_NONE;
        }
    }

    if (pwmProtocolType != PWM_TYPE_NONE) {
        throttle = LL_TIM_IC_GetCaptureCH2(TIM3);
    }

    // printf("---%d,%d,%d----\n", TIM_GetCapture1(TIM3), TIM_GetCapture2(TIM3), pwmProtocolType);
}

/*
static volatile unsigned _last_pulse_width_usec;

static const uint16_t MIN_VALID_PULSE_WIDTH_USEC = 500;
static const uint16_t MAX_VALID_PULSE_WIDTH_USEC = 3000;

static const float STOP_DUTY_CYCLE      = 0.03;
static const float START_MIN_DUTY_CYCLE = 0.06;
static const float START_MAX_DUTY_CYCLE = 0.40;

static const unsigned COMMAND_TTL_MS = 100;

static void signalInputTask(void)
{
    const uint16_t min_pulse_width_usec = motor_signal_Config()->pwm_min_usec;
    const uint16_t max_pulse_width_usec = motor_signal_Config()->pwm_max_usec;

    unsigned local_copy = _last_pulse_width_usec;
    if (local_copy < min_pulse_width_usec) {
        local_copy = min_pulse_width_usec;
    }
    if (local_copy > max_pulse_width_usec) {
        local_copy = max_pulse_width_usec;
    }

    float dc = (local_copy - min_pulse_width_usec) / (float)(max_pulse_width_usec - min_pulse_width_usec);

    // printf("motor_dc:%f\r\n", dc);

    if (motor_is_idle() && ((dc < START_MIN_DUTY_CYCLE) || (dc > START_MAX_DUTY_CYCLE))) {
        dc = 0;
    } else if (dc < STOP_DUTY_CYCLE) {
        dc = 0;
    } else {
        ; // Nothing to do
    }

    if (dc > 0) {
        motor_set_duty_cycle(dc, COMMAND_TTL_MS);
    } else {
        motor_stop();
    }
}
*/

// PWM/Oneshot125/Oneshot42/MultiShot/Dshot150/Dshot300/Dshot600/Dshot1200/Proshot1000
void motor_signal_init(void)
{
    pwm_input_init();
}
