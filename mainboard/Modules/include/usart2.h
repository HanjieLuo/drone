#ifndef __usart2_H
#define __usart2_H

#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"

#define RX_BUFFER_SIZE 128

typedef struct {
    size_t length;
    uint8_t *addr;
}usart2_queue_buffer;

void Usart2Init(void);
void USART2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
static void UartRxCheck(void);
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* __usart2_H */
