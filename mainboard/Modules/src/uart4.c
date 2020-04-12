#include "uart4.h"

static uint8_t rx_buffer[RX_BUFFER_SIZE];

xQueueHandle uart4_queue;
STATIC_MEM_QUEUE_ALLOC(uart4_queue, 5, sizeof(uart4_queue_buffer));

void Uart4Init(void) {
    printf("Uart4Init");
    uart4_queue = STATIC_MEM_QUEUE_CREATE(uart4_queue);

    //使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    //使能接收中断
    HAL_UART_Receive_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE);
}

void UART4_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        UartRxCheck();
    } else {
        HAL_UART_IRQHandler(&huart4);
    }
}

void DMA1_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart4.hdmarx);
}

void DMA1_Stream4_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart4.hdmatx);
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
    uart4_queue_buffer buffer;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Calculate current position in buffer */
    pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);

    printf("%u\r\n", pos);

    if (pos != old_pos) {    /* Check change in received data */
        if (pos > old_pos) { /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            buffer.addr   = &rx_buffer[old_pos];
            buffer.length = pos - old_pos;
            xQueueSendFromISR(uart4_queue, &buffer, &xHigherPriorityTaskWoken);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            buffer.addr   = &rx_buffer[old_pos];
            buffer.length = RX_BUFFER_SIZE - old_pos;
            xQueueSendFromISR(uart4_queue, &buffer, &xHigherPriorityTaskWoken);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                buffer.addr   = &rx_buffer[0];
                buffer.length = pos;
                xQueueSendFromISR(uart4_queue, &buffer, &xHigherPriorityTaskWoken);
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
