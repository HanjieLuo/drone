#ifndef __uart4_H
#define __uart4_H

#include "usart.h"

#define RX_BUFFER_SIZE 128

static unsigned char rx_buffer[RX_BUFFER_SIZE];

void Uart4Init(void);
void UART4_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void UartRxCheck(void);
char* itoa(int value, char* result, int base);

#endif /* __uart4_H */
