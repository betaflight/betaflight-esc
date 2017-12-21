#include "include.h"

// 重定义fputc函数
int fputc(int ch, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    serialWrite(ch);
    return ch;
}

static serialPort_t serialPort;

// 启动串口发送DMA 将串口发送缓冲区的内容使用DMA发送
static void serialStartTxDMA(void)
{
    serialPort_t *s = &serialPort;

    USARTx_TX_DMA_CHANNEL->CMAR = (uint32_t)&s->txBuf[s->txTail];
    if (s->txHead > s->txTail) {
        USARTx_TX_DMA_CHANNEL->CNDTR = s->txHead - s->txTail;
        s->txTail = s->txHead;
    } else {
        USARTx_TX_DMA_CHANNEL->CNDTR = SERIAL_TX_BUFSIZE - s->txTail;
        s->txTail = 0;
    }

    DMA_Cmd(USARTx_TX_DMA_CHANNEL, ENABLE);
}

// 将要发送的内容写入串口发送缓冲区
void serialWrite(int ch)
{
    serialPort_t *s = &serialPort;

    s->txBuf[s->txHead] = ch;
    s->txHead = (s->txHead + 1) % SERIAL_TX_BUFSIZE;

    if (!(USARTx_TX_DMA_CHANNEL->CCR & 1))
        serialStartTxDMA();
}

// 判断是否有收到有效的数据
bool serialAvailable(void)
{
    return (USARTx_RX_DMA_CHANNEL->CNDTR != serialPort.rxPos);
}

// only call after a affirmative return from serialAvailable()
// 从串口缓冲区中读取数据
int serialRead(void)
{
    serialPort_t *s = &serialPort;

    int ch = s->rxBuf[SERIAL_RX_BUFSIZE - s->rxPos];

    if (--s->rxPos == 0) {
        s->rxPos = SERIAL_RX_BUFSIZE;
    }

    return ch;
}

// 串口打印字符串
void serialPrint(const char *str)
{
    while (*str) {
        serialWrite(*(str++));
    }
}

// 初始化串口
void serialInit(void)
{
    serialPort_t *s = &serialPort;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

    /* Enable USART clock */
    USARTx_APBPERIPHCLOCK(USARTx_CLK, ENABLE);

    /* Enable the DMA periph */
    RCC_AHBPeriphClockCmd(DMAx_CLK, ENABLE);

    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
    GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
    GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

    /* USARTx configuration ----------------------------------------------------*/
    /* USARTx configured as follow:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - one Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);

    // Enable the DMA1_Channel2_3 global Interrupt
    nvicEnableVector(DMA1_Channel2_3_IRQn, 1);

    /* Clear DMA1 global flags */
    DMA_ClearFlag(USARTx_TX_DMA_FLAG_GL);
    DMA_ClearFlag(USARTx_RX_DMA_FLAG_GL);

    //串口收发指针
    s->rxHead = s->rxTail = 0;
    s->txHead = s->txTail = 0;

    /* DMA Configuration -------------------------------------------------------*/
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /* DMA channel Rx of USART Configuration */
    DMA_DeInit(USARTx_RX_DMA_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = USARTx_RDR_ADDRESS;
    DMA_InitStructure.DMA_BufferSize = SERIAL_RX_BUFSIZE;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)s->rxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_Init(USARTx_RX_DMA_CHANNEL, &DMA_InitStructure);
    /* Enable the DMA channel */
    DMA_Cmd(USARTx_RX_DMA_CHANNEL, ENABLE);

    /* Enable the USART Rx DMA request */
    USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
    s->rxPos = DMA_GetCurrDataCounter(USARTx_RX_DMA_CHANNEL);

    /* DMA channel Tx of USART Configuration */
    DMA_DeInit(USARTx_TX_DMA_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = USARTx_TDR_ADDRESS;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(USARTx_TX_DMA_CHANNEL, &DMA_InitStructure);

    DMA_ITConfig(USARTx_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);
    USARTx_TX_DMA_CHANNEL->CNDTR = 0;

    USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USARTx, ENABLE);
}

// USART tx DMA IRQ  发送DMA完成中断
void DMA1_Channel2_3_IRQHandler(void)
{
    DMA_ClearFlag(USARTx_TX_DMA_FLAG_TC);
    DMA_Cmd(USARTx_TX_DMA_CHANNEL, DISABLE);

    if (serialPort.txHead != serialPort.txTail) {// 判断是否有数据要发送出去
        serialStartTxDMA();
    }
}
