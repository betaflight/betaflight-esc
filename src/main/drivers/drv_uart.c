#include "include.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_gpio.h"

int _write (int fd, char *ptr, int len)
{
    UNUSED(fd);
    for (int i = 0; i < len; i++) {
        serialWrite(*(ptr)++);
    }
    return len;
}

static serialPort_t serialPort;

static void serialStartTxDMA(void)
{
    serialPort_t *s = &serialPort;

    LL_DMA_SetMemoryAddress(DMA1, USART_TX_DMA_CHANNEL, (uint32_t)&s->txBuf[s->txTail]);
    if (s->txHead > s->txTail) {
        LL_DMA_SetDataLength(DMA1, USART_TX_DMA_CHANNEL, s->txHead - s->txTail);
        s->txTail = s->txHead;
    } else {
        LL_DMA_SetDataLength(DMA1, USART_TX_DMA_CHANNEL, SERIAL_TX_BUFSIZE - s->txTail);
        s->txTail = 0;
    }
    /* Enable the DMA channel */
    LL_DMA_EnableChannel(DMA1, USART_TX_DMA_CHANNEL);
}

void serialWrite(char ch)
{
    serialPort_t *s = &serialPort;

    s->txBuf[s->txHead] = ch;
    s->txHead = (s->txHead + 1) % SERIAL_TX_BUFSIZE;

    if (!LL_DMA_IsEnabledChannel(DMA1, USART_TX_DMA_CHANNEL)) {
        serialStartTxDMA();
    }
}

bool serialAvailable(void)
{
    return (LL_DMA_GetDataLength(DMA1, USART_RX_DMA_CHANNEL) != serialPort.rxPos);
}

char serialRead(void)
{
    serialPort_t *s = &serialPort;

    char ch = s->rxBuf[SERIAL_RX_BUFSIZE - s->rxPos];

    if (--s->rxPos == 0) {
        s->rxPos = SERIAL_RX_BUFSIZE;
    }

    return ch;
}

void serialPrint(const char *str)
{
    char ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(ch);
    }
}

void serialInit(void)
{
    serialPort_t *s = &serialPort;

    /* Enable GPIO clock */
    LL_AHB1_GRP1_EnableClock(USART_TX_GPIO_CLK | USART_RX_GPIO_CLK);

    /* Enable USART clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* Configure USART Tx and Rx as alternate function push-pull */
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;

    GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
    GPIO_InitStructure.Pin = USART_TX_PIN;
    LL_GPIO_Init(USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Alternate = LL_GPIO_AF_0;
    GPIO_InitStructure.Pin = USART_RX_PIN;
    LL_GPIO_Init(USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* USARTx configuration ----------------------------------------------------*/
    /* USARTx configured as follow:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - one Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    LL_USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
    USART_InitStructure.Parity = LL_USART_PARITY_NONE;    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStructure.TransferDirection = LL_USART_DIRECTION_RX | LL_USART_DIRECTION_TX;
    USART_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART, &USART_InitStructure);

    // Enable the DMA1_Channel2_3 global Interrupt
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    /* Clear DMA1 global flags */
    LL_DMA_ClearFlag_GI2(DMA1);
    LL_DMA_ClearFlag_GI3(DMA1);

    s->rxHead = s->rxTail = 0;
    s->txHead = s->txTail = 0;

    /* DMA Configuration -------------------------------------------------------*/
    LL_DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;

    /* DMA channel Rx of USART Configuration */
    LL_DMA_DeInit(DMA1, USART_RX_DMA_CHANNEL);
    DMA_InitStructure.PeriphOrM2MSrcAddress = USART_RDR_ADDRESS;
    DMA_InitStructure.NbData = SERIAL_RX_BUFSIZE;
    DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)s->rxBuf;
    DMA_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority = LL_DMA_PRIORITY_LOW;
    LL_DMA_Init(DMA1, USART_RX_DMA_CHANNEL, &DMA_InitStructure);
    /* Enable the DMA channel */
    LL_DMA_EnableChannel(DMA1, USART_RX_DMA_CHANNEL);

    /* Enable the USART Rx DMA request */
    LL_USART_EnableDMAReq_RX(USART);
    s->rxPos = LL_DMA_GetDataLength(DMA1, USART_RX_DMA_CHANNEL);

    /* DMA channel Tx of USART Configuration */
    LL_DMA_DeInit(DMA1, USART_TX_DMA_CHANNEL);
    DMA_InitStructure.PeriphOrM2MSrcAddress = USART_TDR_ADDRESS;
    DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
    LL_DMA_Init(DMA1, USART_TX_DMA_CHANNEL, &DMA_InitStructure);

    LL_DMA_EnableIT_TC(DMA1, USART_TX_DMA_CHANNEL);
    LL_DMA_SetDataLength(DMA1, USART_TX_DMA_CHANNEL, 0);

    LL_USART_EnableDMAReq_TX(USART);

    LL_USART_Enable(USART);
}

void DMA1_Channel2_3_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);
        LL_DMA_DisableChannel(DMA1, USART_TX_DMA_CHANNEL);

        if (serialPort.txHead != serialPort.txTail) {
            serialStartTxDMA();
        }
    }
}