#include "include.h"

#define LOWPASS(xold, xnew, alpha_rcpr) \
    (((xold) * (alpha_rcpr) + (xnew) + (((alpha_rcpr) + 1) / 2)) / ((alpha_rcpr) + 1))

volatile uint16_t adcValues[6];

static struct motor_adc_sample _sample;

void update_voltage_current_temperate(void)
{
    static const int ALPHA_RCPR = 7; // A power of two minus one (1, 3, 7)
    _sample.input_current = LOWPASS(_sample.input_current, adcValues[0], ALPHA_RCPR);
    _sample.input_voltage = LOWPASS(_sample.input_voltage, adcValues[1], ALPHA_RCPR);
    _sample.input_temperature = LOWPASS(_sample.input_temperature, adcValues[2], ALPHA_RCPR);

    _sample.phase_values[0] = LOWPASS(_sample.phase_values[0], adcValues[3], ALPHA_RCPR);
    _sample.phase_values[1] = LOWPASS(_sample.phase_values[1], adcValues[4], ALPHA_RCPR);
    _sample.phase_values[2] = LOWPASS(_sample.phase_values[2], adcValues[5], ALPHA_RCPR);

    // printf("A:%d,B:%d,C:%d\n", _sample.phase_values[0], _sample.phase_values[1], _sample.phase_values[2]);
}

void motor_adc_init(void)
{
    /* GPIOA Periph clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* ADC1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Configure ADC Channel3 and channel6 as analog input */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* DMA1 clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* DMA1 Channel1 Config */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 6;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* DMA1 Channel1 enable */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    /* ADC1 DeInit */
    ADC_DeInit(ADC1);
    /* Initialize ADC structure */
    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);

    /* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Convert the ADC1 Channel3 and channel6 with 55.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC1, ADC_Channel_3, ADC_SampleTime_55_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_55_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor, ADC_SampleTime_55_5Cycles);
    ADC_TempSensorCmd(ENABLE);

    ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_55_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_55_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_55_5Cycles);

    ADC_OverrunModeCmd(ADC1,ENABLE);

    ADC_GetCalibrationFactor(ADC1);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

    /* ADC1 regular Software Start Conv */
    ADC_StartOfConversion(ADC1);
}

float motor_adc_get_voltage(void)
{
    return ((_sample.input_voltage * 3.3) / 0xFFF) * 10.82;
}

float motor_adc_get_current(void)
{
#define CURRENT_METER_SCALE 400 // for Allegro ACS758LCB-100U (40mV/A)

    return (((_sample.input_current * 3.3) / 0xFFF) * 1000) / CURRENT_METER_SCALE;
}

float motor_adc_get_temperature(void)
{
    const float temperature = (float)((_sample.input_temperature * 3.3) / 0xFFF);
    return (1.43f - temperature) / 0.0043f + 25;
}

struct motor_adc_sample motor_adc_get_last_sample(void)
{
    return _sample;
}
