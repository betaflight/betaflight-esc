#pragma once

#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_6
#define USARTx_TX_GPIO_PORT              GPIOB
#define USARTx_TX_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define USARTx_TX_SOURCE                 GPIO_PinSource6
#define USARTx_TX_AF                     GPIO_AF_0

#define USARTx_RX_PIN                    GPIO_Pin_7
#define USARTx_RX_GPIO_PORT              GPIOB
#define USARTx_RX_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define USARTx_RX_SOURCE                 GPIO_PinSource7
#define USARTx_RX_AF                     GPIO_AF_0

#define DMAx_CLK                         RCC_AHBPeriph_DMA1

#define USARTx_TDR_ADDRESS               0x40013828
#define USARTx_RDR_ADDRESS               0x40013824

#define USARTx_TX_DMA_CHANNEL            DMA1_Channel2
#define USARTx_TX_DMA_FLAG_TC            DMA1_FLAG_TC2
#define USARTx_TX_DMA_FLAG_GL            DMA1_FLAG_GL2

#define USARTx_RX_DMA_CHANNEL            DMA1_Channel3
#define USARTx_RX_DMA_FLAG_TC            DMA1_FLAG_TC3
#define USARTx_RX_DMA_FLAG_GL            DMA1_FLAG_GL3

#define SERIAL_TX_BUFSIZE	256
#define SERIAL_RX_BUFSIZE	256

typedef struct {
    volatile unsigned char txBuf[SERIAL_TX_BUFSIZE];
    unsigned int txHead, txTail;

    volatile unsigned char rxBuf[SERIAL_RX_BUFSIZE];
    volatile unsigned int rxHead, rxTail;

    unsigned int rxPos;
} serialPort_t;

void serialInit(void);
void serialWrite(int ch);
void serialPrint(const char *str);
bool serialAvailable(void);
int serialRead(void);
