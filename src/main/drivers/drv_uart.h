#pragma once

#define USART                           USART1
#define USART_IRQn                      USART1_IRQn
#define USART_IRQHandler                USART1_IRQHandler

#define USART_TX_PIN                    LL_GPIO_PIN_6
#define USART_TX_GPIO_PORT              GPIOB
#define USART_TX_GPIO_CLK               LL_AHB1_GRP1_PERIPH_GPIOB
#define USART_TX_AF                     LL_GPIO_AF_0

#define USART_RX_PIN                    LL_GPIO_PIN_7
#define USART_RX_GPIO_PORT              GPIOB
#define USART_RX_GPIO_CLK               LL_AHB1_GRP1_PERIPH_GPIOB
#define USART_RX_AF                     LL_GPIO_AF_0

#define USART_TDR_ADDRESS               0x40013828
#define USART_RDR_ADDRESS               0x40013824

#define USART_TX_DMA_CHANNEL            LL_DMA_CHANNEL_2
#define USART_RX_DMA_CHANNEL            LL_DMA_CHANNEL_3

#define SERIAL_TX_BUFSIZE   256
#define SERIAL_RX_BUFSIZE   256

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
