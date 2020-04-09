#ifndef __uart4_H
#define __uart4_H

#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"

#define RX_BUFFER_SIZE 128

typedef struct {
    size_t length;
    uint8_t *addr;
}uart4_queue_buffer;

void Uart4Init(void);
void UART4_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
static void UartRxCheck(void);

// extern DMA_HandleTypeDef hdma_uart4_tx;
// extern DMA_HandleTypeDef hdma_uart4_rx;
// extern UART_HandleTypeDef huart4;

#endif /* __uart4_H */
