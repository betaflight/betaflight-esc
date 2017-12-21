#include "include.h"

uint32_t newInput;

// 计算油门值
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

    // 校验CRC值,获得油门值
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
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    /* TIM3 chennel1 configuration : PB4 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM pin to AF1 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Enable the TIM3 global Interrupt */
    nvicEnableVector(TIM3_IRQn, 2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 48-1;// 48分频,1Mhz时钟,1Us的定时.PWM普通:48分频; Oneshot125:6分频; Oneshot42:2分频
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x04;

    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

    /* Select the TIM3 Input Trigger: TI1FP1 */
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);

    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

    /* TIM enable counter */
    TIM_Cmd(TIM3, ENABLE);

    /* Enable the CC1 Interrupt Request */
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
}

static __IO uint16_t throttle = 0;

void TIM3_IRQHandler(void)
{
    /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

    if(pwmProtocolType == PWM_TYPE_NONE) {
        // 获取高电平的脉宽宽度(?us)
        const uint16_t pulseWidth = TIM_GetCapture2(TIM3);

        if(950 <= pulseWidth && pulseWidth <= 2050) { // 1000Us - 2000Us
            pwmProtocolType = PWM_TYPE_STANDARD;
        } else if(120 <= pulseWidth && pulseWidth <= 255) { // 125Us - 250Us
            pwmProtocolType = PWM_TYPE_ONESHOT125;
        } else if(40 <= pulseWidth && pulseWidth <= 86) { // 42Us - 84Us
            pwmProtocolType = PWM_TYPE_ONESHOT42;
        } else if(2 <= pulseWidth && pulseWidth <= 28) { // 5Us - 25Us
            pwmProtocolType = PWM_TYPE_MULTISHOT;
        } else {
            pwmProtocolType = PWM_TYPE_NONE;
        }
    }

    if(pwmProtocolType != PWM_TYPE_NONE) {
        throttle = TIM_GetCapture2(TIM3);
    }

    // 周期 脉宽
    // printf("---%d,%d,%d----\n", TIM_GetCapture1(TIM3), TIM_GetCapture2(TIM3), pwmProtocolType);
}

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

    /*
     * Scale the input signal into [0, 1]
     */
    unsigned local_copy = _last_pulse_width_usec;
    if (local_copy < min_pulse_width_usec) {
        local_copy = min_pulse_width_usec;
    }
    if (local_copy > max_pulse_width_usec) {
        local_copy = max_pulse_width_usec;
    }

    // 将油门值转化为PWM的占空比(0.00 - 1.00)
    float dc = (local_copy - min_pulse_width_usec) / (float)(max_pulse_width_usec - min_pulse_width_usec);

    // printf("motor_dc:%f\r\n", dc);

    /*
    * Handle start/stop corner cases 条件判断
    */
    if (motor_is_idle() && ((dc < START_MIN_DUTY_CYCLE) || (dc > START_MAX_DUTY_CYCLE))) {
        dc = 0;
    } else if (dc < STOP_DUTY_CYCLE) {
        dc = 0;
    } else {
        ; // Nothing to do
    }

    /*
     * Pass the new command into the motor controller
     */
    if (dc > 0) {
        motor_set_duty_cycle(dc, COMMAND_TTL_MS);
    } else {
        motor_stop();
    }
}

// 自动检测信号是 PWM/Oneshot125/Oneshot42/MultiShot/Dshot150/Dshot300/Dshot600/Dshot1200/Proshot1000
void motor_signal_init(void)
{
    // 自动检测
    pwm_input_init();
}
