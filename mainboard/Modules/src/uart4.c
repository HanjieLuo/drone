#include "uart4.h"

void Uart4Init(void) {
    //使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    //使能接收中断
    HAL_UART_Receive_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE);
}

void UART4_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        UartRxCheck();
    }
}

void DMA1_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart4.hdmarx);
    // // char buffer[] = "DMA1_Stream2_IRQHandler\n";
    // // HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 1000);

    // if(__HAL_DMA_GET_FLAG(huart4.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(huart4.hdmarx))!= RESET) {
    //     __HAL_DMA_CLEAR_FLAG(huart4.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(huart4.hdmarx));

    //     char buffer[] = "HT\n";
    //     HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sizeof(buffer), 1000);
    // } else {
    //     HAL_DMA_IRQHandler(huart4.hdmarx);
    // }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
        UartRxCheck();
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
        UartRxCheck();
    }
}

void UartRxCheck(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);

    char buffer[10];
    // itoa(old_pos, buffer, 10);
    // HAL_UART_Transmit(&huart1, (uint8_t *)buffer, sizeof(buffer), 1000);

    itoa(pos, buffer, 10);
    printf("%s\n", buffer);
    // HAL_UART_Transmit(&huart1, (uint8_t *)buffer, sizeof(buffer), 1000);

    if (pos != old_pos) {    /* Check change in received data */
        if (pos > old_pos) { /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            // usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            // usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                // usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos; /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == RX_BUFFER_SIZE) {
        old_pos = 0;
    }
}

void MavlinkParse(uint8_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (mavlink_parse_char(mavlink_comm, buffer[i], &msg, &status)) {
            // switch (msg.msgid) {
            //     case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            //         Serial.println("MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE");
            //         break;
            //     }
            //     default: {
            //         break;
            //     }
            // }
            // Serial.println("=======");
            // Serial.println(msg.msgid);
            // Serial.println(msg.seq);
            // Serial.println(msg.compid);
            // Serial.println(msg.sysid);
            // Serial.println("=======");
        }
    }
}
