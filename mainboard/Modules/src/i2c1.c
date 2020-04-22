#include "i2c1.h"

#define I2C_MAX_WAIT pdMS_TO_TICKS(1000UL)

static TaskHandle_t i2c1_notify = NULL;
static u_int32_t notify_value;

void I2C1Init(void) {

}

bool I2C1Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
    i2c1_notify = xTaskGetCurrentTaskHandle();
    if(HAL_I2C_Mem_Read_DMA(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size) == HAL_OK) {
        // printf("ulTaskNotifyTake\r\n");
        notify_value = ulTaskNotifyTake(pdTRUE, I2C_MAX_WAIT);
        if (notify_value == 1) {
            return true;
        }
    } 
    return false;
}

bool I2C1Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
    i2c1_notify = xTaskGetCurrentTaskHandle();
    if(HAL_I2C_Mem_Write_DMA(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size) == HAL_OK) {
        // printf("ulTaskNotifyTake\r\n");
        notify_value = ulTaskNotifyTake(pdTRUE, I2C_MAX_WAIT);
        if (notify_value == 1) {
            return true;
        }
    } 
    return false;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // printf("MemRxCpltCallback\r\n");
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(i2c1_notify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    printf("MemTxCpltCallback\r\n");
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(i2c1_notify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}