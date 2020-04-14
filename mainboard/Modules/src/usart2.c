#include "usart2.h"

static uint8_t rx_buffer[RX_BUFFER_SIZE];

xQueueHandle usart2_queue;
STATIC_MEM_QUEUE_ALLOC(usart2_queue, 5, sizeof(usart2_queue_buffer));

void Usart2Init(void) {
    usart2_queue = STATIC_MEM_QUEUE_CREATE(usart2_queue);

    //使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    //使能接收中断
    HAL_UART_Receive_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);
}

void USART2_IRQHandler(void) {
    // printf("UART4_IRQHandler\r\n");
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        UartRxCheck();
    } else {
        HAL_UART_IRQHandler(&huart2);
    }
}

void DMA1_Stream5_IRQHandler(void) {
    // printf("DMA1_Stream2_IRQHandler\r\n");
    HAL_DMA_IRQHandler(huart2.hdmarx);
}

void DMA1_Stream6_IRQHandler(void) {
    // printf("DMA1_Stream4_IRQHandler\r\n");
    HAL_DMA_IRQHandler(huart2.hdmatx);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        UartRxCheck();
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        UartRxCheck();
    }
}

void UartRxCheck(void) {
    static size_t old_pos;
    size_t pos;
    usart2_queue_buffer buffer;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Calculate current position in buffer */
    pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    // printf("%u\r\n", pos);

    if (pos != old_pos) {    /* Check change in received data */
        if (pos > old_pos) { /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            buffer.addr   = &rx_buffer[old_pos];
            buffer.length = pos - old_pos;
            xQueueSendFromISR(usart2_queue, &buffer, &xHigherPriorityTaskWoken);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            buffer.addr   = &rx_buffer[old_pos];
            buffer.length = RX_BUFFER_SIZE - old_pos;
            xQueueSendFromISR(usart2_queue, &buffer, &xHigherPriorityTaskWoken);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                buffer.addr   = &rx_buffer[0];
                buffer.length = pos;
                xQueueSendFromISR(usart2_queue, &buffer, &xHigherPriorityTaskWoken);
            }
        }
    }
    old_pos = pos; /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == RX_BUFFER_SIZE) {
        old_pos = 0;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//     printf("HAL_UART_TxCpltCallback\r\n");
//     if(huart->Instance == UART4){
//         huart->gState = HAL_UART_STATE_READY;
//     }
// }
