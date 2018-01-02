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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    LL_GPIO_InitTypeDef gpioInit;
    gpioInit.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
    gpioInit.Mode = LL_GPIO_MODE_ANALOG;
    gpioInit.Pull = LL_GPIO_PULL_NO ;
    gpioInit.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &gpioInit);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_InitTypeDef dmaInit;
    dmaInit.PeriphOrM2MSrcAddress = (uint32_t)&ADC1->DR;
    dmaInit.MemoryOrM2MDstAddress = (uint32_t)adcValues;
    dmaInit.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dmaInit.NbData = 6;
    dmaInit.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dmaInit.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dmaInit.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    dmaInit.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    dmaInit.Mode = LL_DMA_MODE_CIRCULAR;
    dmaInit.Priority = LL_DMA_PRIORITY_HIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &dmaInit);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    LL_ADC_CommonDeInit(__LL_ADC_COMMON_INSTANCE(ADC1));

    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_6);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_5);

    LL_ADC_InitTypeDef adcInit;
    LL_ADC_StructInit(&adcInit);
    adcInit.Clock = LL_ADC_CLOCK_ASYNC;
    adcInit.Resolution = LL_ADC_RESOLUTION_12B;
    adcInit.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    adcInit.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &adcInit);

    LL_ADC_REG_InitTypeDef adcRegInit;
    adcRegInit.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    adcRegInit.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    adcRegInit.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    adcRegInit.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    adcRegInit.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
    LL_ADC_REG_Init(ADC1, &adcRegInit);
    
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_55CYCLES_5);

    LL_ADC_StartCalibration(ADC1);
    while(LL_ADC_IsCalibrationOnGoing(ADC1) == 1);

    LL_ADC_Enable(ADC1);
    while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));

    LL_ADC_REG_StartConversion(ADC1);
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
