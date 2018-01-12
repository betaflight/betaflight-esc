#include "include.h"

#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_comp.h"

#define BEMF_A_CMP_IN LL_COMP_INPUT_MINUS_IO1    // PA0
#define BEMF_B_CMP_IN LL_COMP_INPUT_MINUS_DAC1_CH1  // PA4
#define BEMF_C_CMP_IN LL_COMP_INPUT_MINUS_DAC1_CH2  // PA5

static LL_COMP_InitTypeDef COMP_InitStructure;

void ADC1_COMP_IRQHandler(void)
{
    if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21) != RESET) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
        // motor_comparator_zc_callback(LL_COMP_GetPowerMode(COMP1) == LL_COMP_OUTPUT_LEVEL_HIGH);
    }
}

// BEMF_A:PA0
// BEMF_B:PA4
// BEMF_C:PA5
// BEMF_STAR:PA1
void motor_comparator_init(void)
{
    /* GPIOA Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    /* Configure PA0 PA1 PA4 PA5: PA1 is used as COMP1 non inveting input */
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    LL_GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* COMP Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_COMP_StructInit(&COMP_InitStructure);
    COMP_InitStructure.InputMinus = BEMF_B_CMP_IN;
    COMP_InitStructure.OutputSelection = LL_COMP_OUTPUT_NONE;
    COMP_InitStructure.PowerMode = LL_COMP_POWERMODE_HIGHSPEED;
    COMP_InitStructure.InputHysteresis = LL_COMP_HYSTERESIS_NONE;
    COMP_InitStructure.OutputPolarity = LL_COMP_OUTPUTPOL_NONINVERTED;
    LL_COMP_Init(COMP1, &COMP_InitStructure);

    /* Enable COMP1 */
    LL_COMP_Enable(COMP1);

    /* Configure EXTI Line 21 in interrupt mode */
    LL_EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.Line_0_31 = LL_EXTI_LINE_21;
    EXTI_InitStructure.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStructure.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    EXTI_InitStructure.LineCommand = ENABLE;
    LL_EXTI_Init(&EXTI_InitStructure);
    /* Clear EXTI21 line */
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);

    nvicEnableVector(ADC1_COMP_IRQn, 0);
}

void motor_comparator_set_input_source(uint_fast8_t phase)
{
    switch(phase) {
    case 0:
        COMP_InitStructure.InputMinus = BEMF_A_CMP_IN;
        break;

    case 1:
        COMP_InitStructure.InputMinus = BEMF_B_CMP_IN;
        break;

    case 2:
        COMP_InitStructure.InputMinus = BEMF_C_CMP_IN;
        break;

    default:
        return;
    }

    LL_COMP_Init(COMP1, &COMP_InitStructure);
}

void motor_comparator_enable_from_isr(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

void motor_comparator_disable_from_isr(void)
{
    NVIC_DisableIRQ(ADC1_COMP_IRQn);
}
