#ifndef __uart4_H
#define __uart4_H

#include "usart.h"
#include "MAVLink/common/mavlink.h"

#define RX_BUFFER_SIZE 128

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static mavlink_message_t msg;
static mavlink_status_t status;
static int mavlink_comm = MAVLINK_COMM_0;

void Uart4Init(void);
void UART4_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void UartRxCheck(void);
void MavlinkParse(uint8_t *buffer, size_t len);

#endif /* __uart4_H */
