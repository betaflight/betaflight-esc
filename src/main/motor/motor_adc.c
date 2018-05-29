#include "include.h"
#include "stm32f0xx_ll_dma.h"

// Simple IIR that does some fixed point math to make the filter more accurate without floats
// (1-alpha)*xnew + alpha*xold
#define LOWPASS(xold, xnew, pow2) \
   ((((1<<pow2) - 1) * xold +  xnew) >> pow2) //max power2 of 16

    
#define ADCBUFSIZE 6
volatile uint16_t adcValues[ADCBUFSIZE];

static struct motor_adc_sample _sample;

void updateAdcValues(void)
{
    /*
    static const int ALPHA_RCPR = 7; // A power of two minus one (1, 3, 7)
    _sample.input_current = LOWPASS(_sample.input_current, adcValues[0], ALPHA_RCPR);
    _sample.input_voltage = LOWPASS(_sample.input_voltage, adcValues[1], ALPHA_RCPR);
    _sample.input_temperature = LOWPASS(_sample.input_temperature, adcValues[2], ALPHA_RCPR);

    _sample.phase_values[0] = LOWPASS(_sample.phase_values[0], adcValues[3], ALPHA_RCPR);
    _sample.phase_values[1] = LOWPASS(_sample.phase_values[1], adcValues[4], ALPHA_RCPR);
    _sample.phase_values[2] = LOWPASS(_sample.phase_values[2], adcValues[5], ALPHA_RCPR);
*/


    
#ifdef FILTERADC
    #define filterpowalpha 4    
    _sample.input_temperature = LOWPASS(_sample.input_temperature, adcValues[0], filterpowalpha);
    _sample.phase_values[0] = LOWPASS(_sample.phase_values[0], adcValues[1], filterpowalpha);
    _sample.input_voltage = LOWPASS(_sample.input_voltage, adcValues[2], filterpowalpha);
    _sample.phase_values[1] = LOWPASS(_sample.phase_values[1], adcValues[3], filterpowalpha);
    _sample.phase_values[2] = LOWPASS(_sample.phase_values[2], adcValues[4], filterpowalpha);
    _sample.input_current = LOWPASS(_sample.input_current, adcValues[5], filterpowalpha);
#else    
    _sample.input_temperature = adcValues[0];
    _sample.phase_values[0] = adcValues[1];
    _sample.input_voltage = adcValues[2];
    _sample.phase_values[1] = adcValues[3];
    _sample.phase_values[2] = adcValues[4];    
    _sample.input_current = adcValues[5];
#endif
    // printf("A:%d,B:%d,C:%d\n", _sample.phase_values[0], _sample.phase_values[1], _sample.phase_values[2]);
}

void DMA1_Channel1_IRQHandler(void){
    if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);
        updateAdcValues();
    }
}

void motor_adc_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    LL_GPIO_InitTypeDef gpioInit;
    gpioInit.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
    gpioInit.Mode = LL_GPIO_MODE_ANALOG;
    gpioInit.Pull = LL_GPIO_PULL_NO ;
    LL_GPIO_Init(GPIOA, &gpioInit);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_InitTypeDef dmaInit;
    dmaInit.PeriphOrM2MSrcAddress = LL_ADC_DMA_GetRegAddr(ADC1,LL_ADC_DMA_REG_REGULAR_DATA);
    dmaInit.MemoryOrM2MDstAddress = (uint32_t)adcValues;
    dmaInit.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dmaInit.NbData = ADCBUFSIZE;
    dmaInit.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dmaInit.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dmaInit.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    dmaInit.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    dmaInit.Mode = LL_DMA_MODE_CIRCULAR;
    dmaInit.Priority = LL_DMA_PRIORITY_HIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &dmaInit);


    LL_ADC_CommonDeInit(__LL_ADC_COMMON_INSTANCE(ADC1));
    LL_ADC_DeInit(ADC1);
    

    LL_ADC_InitTypeDef adcInit;
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
    
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0 | LL_ADC_CHANNEL_3 | LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_5 | LL_ADC_CHANNEL_6 | LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_55CYCLES_5);

    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));
    LL_ADC_ClearFlag_ADRDY(ADC1);
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsEnabled(ADC1));

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_ADC_REG_StartConversion(ADC1);
}

float motor_adc_get_voltage(void)
{
    return ((_sample.input_voltage * 3.3f) / 0xFFF) * 10.82f;
}

float motor_adc_get_current(void)
{
    #define CURRENT_METER_SCALE 400.0f // for Allegro ACS758LCB-100U (40mV/A)

    return (((_sample.input_current * 3.3f) / 0xFFF) * 1000.0f) / CURRENT_METER_SCALE;
}

float motor_adc_get_temperature(void)
{
   return __LL_ADC_CALC_TEMPERATURE(3300,_sample.input_temperature,LL_ADC_RESOLUTION_12B);
}

struct motor_adc_sample motor_adc_get_last_sample(void)
{
    return _sample;
}
