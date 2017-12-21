#include "include.h"

#define BEMF_A_CMP_IN COMP_InvertingInput_IO    // PA0
#define BEMF_B_CMP_IN COMP_InvertingInput_DAC1  // PA4
#define BEMF_C_CMP_IN COMP_InvertingInput_DAC2  // PA5

static COMP_InitTypeDef COMP_InitStructure;

void ADC1_COMP_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line21) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line21);
        motor_comparator_zc_callback(COMP_GetOutputLevel(COMP_Selection_COMP1) == COMP_OutputLevel_High);
    }
}

// BEMF_A:PA0
// BEMF_B:PA4
// BEMF_C:PA5
// BEMF_STAR:PA1
void motor_comparator_init(void)
{
    /* GPIOA Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* Configure PA0 PA1 PA4 PA5: PA1 is used as COMP1 non inveting input */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* COMP Peripheral clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    COMP_StructInit(&COMP_InitStructure);
    COMP_InitStructure.COMP_InvertingInput = BEMF_B_CMP_IN;
    COMP_InitStructure.COMP_Output = COMP_Output_None;
    COMP_InitStructure.COMP_Mode = COMP_Mode_HighSpeed;
    COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_No;
    COMP_InitStructure.COMP_OutputPol = COMP_OutputPol_NonInverted;
    COMP_Init(COMP_Selection_COMP1, &COMP_InitStructure);

    /* Enable COMP1 */
    COMP_Cmd(COMP_Selection_COMP1, ENABLE);

    /* Configure EXTI Line 21 in interrupt mode */
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line21;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    /* Clear EXTI21 line */
    EXTI_ClearITPendingBit(EXTI_Line21);

    nvicEnableVector(ADC1_COMP_IRQn, 0);
}

void motor_comparator_set_input_source(uint_fast8_t phase)
{
    switch(phase) {
    case 0:
        COMP_InitStructure.COMP_InvertingInput = BEMF_A_CMP_IN;
        break;

    case 1:
        COMP_InitStructure.COMP_InvertingInput = BEMF_B_CMP_IN;
        break;

    case 2:
        COMP_InitStructure.COMP_InvertingInput = BEMF_C_CMP_IN;
        break;

    default:
        return;
    }

    COMP_Init(COMP_Selection_COMP1, &COMP_InitStructure);
}

void motor_comparator_enable_from_isr(void)
{
    EXTI_ClearITPendingBit(EXTI_Line21);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

void motor_comparator_disable_from_isr(void)
{
    NVIC_DisableIRQ(ADC1_COMP_IRQn);
}
