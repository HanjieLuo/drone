#include "systemview_task.h"

#define SYSVIEW_SINGLE_TX 256

extern TaskHandle_t system_view_notify;

STATIC_MEM_TASK_ALLOC(systemview_task, SYSTEMVIEW_TASK_STACKSIZE);
static void SystemViewTask(void *param);

uint8_t hello_message[32] = {
    'S', 'E', 'G', 'G', 'E', 'R', ' ',
    'S', 'y', 's', 't', 'e', 'm', 'V', 'i', 'e', 'w',
    ' ', 'V', '0' + SEGGER_SYSVIEW_MAJOR,
    '.', '0' + (SEGGER_SYSVIEW_MINOR / 10),
    '0' + (SEGGER_SYSVIEW_MINOR % 10),
    '.', '0' + (SEGGER_SYSVIEW_REV / 10),
    '0' + (SEGGER_SYSVIEW_REV % 10),
    '\0', 0, 0, 0, 0, 0};

void SystemViewLaunch(void) {
    // uint8_t buffer[] = "Hello world\r\n";
    // HAL_UART_Transmit_DMA(&huart1, buffer, sizeof(buffer));

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    SEGGER_SYSVIEW_Conf();

    STATIC_MEM_TASK_CREATE(systemview_task, SystemViewTask, "SystemViewTask", NULL, SYSTEMVIEW_TASK_PRI);
}

void SystemViewTask(void *param) {
    system_view_notify = xTaskGetCurrentTaskHandle();

    // 获取Channel ID
    int channel_id = SEGGER_SYSVIEW_GetChannelID();

    //发送HELLO包
    HAL_UART_Transmit_DMA(&huart1, hello_message, 32);

    uint8_t rx_buf;
    uint8_t tx_buf[SYSVIEW_SINGLE_TX];
    uint32_t notify_flag;
    bool dma_in_progress = true;
    TickType_t prev_send;

    //启动记录
    SEGGER_SYSVIEW_Start();

    while (1) {
        if (xTaskNotifyWait(0x00, 0x03, &notify_flag, pdMS_TO_TICKS(400)) == pdTRUE) {
            if (notify_flag & 0x01) {
                SEGGER_RTT_WriteDownBufferNoLock(channel_id, &rx_buf, 0x01);
                HAL_UART_Receive_IT(&huart1, &rx_buf, 0x01);
            } else if (notify_flag & 0x02) {
                if (dma_in_progress) {
                    dma_in_progress = false;
                    prev_send       = xTaskGetTickCount();
                }
            }
        }

        if (dma_in_progress == false && xTaskGetTickCount() - prev_send >= pdMS_TO_TICKS(400)) {
            unsigned int tx_length = SEGGER_RTT_GetBytesInBuffer(channel_id);
            if (tx_length >= SYSVIEW_SINGLE_TX) {
                uint32_t num = SEGGER_RTT_ReadUpBufferNoLock(channel_id, tx_buf, SYSVIEW_SINGLE_TX);
                HAL_UART_Transmit_DMA(&huart1, tx_buf, num);
            } else if (tx_length != 0) {
                uint32_t num = SEGGER_RTT_ReadUpBufferNoLock(channel_id, tx_buf, tx_length);
                HAL_UART_Transmit_DMA(&huart1, tx_buf, num);
            }
            dma_in_progress = true;
        }
        // HAL_UART_Transmit_DMA(&huart1, buffer, sizeof(buffer));
        // HAL_UART_Transmit_DMA(&huart1, hello_message, 32);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
