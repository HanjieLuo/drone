#include "i2c1.h"

#define I2C_MAX_WAIT pdMS_TO_TICKS(1000UL)

static TaskHandle_t i2c1_notify = NULL;
static u_int32_t notify_value = 0;

bool I2C1MenRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
    i2c1_notify = xTaskGetCurrentTaskHandle();
    if(HAL_I2C_Mem_Read_DMA(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size) == HAL_OK) {
        // printf("ulTaskNotifyTake\r\n");
        notify_value = ulTaskNotifyTake(pdTRUE, I2C_MAX_WAIT);
        // printf("%lu\r\n", notify_value);
        if (notify_value == 1) {
            return true;
        }
    } 
    return false;
}

bool I2C1MenWrite(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
    i2c1_notify = xTaskGetCurrentTaskHandle();
    if(HAL_I2C_Mem_Write_DMA(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size) == HAL_OK) {
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // // printf("ulTaskNotifyTake\r\n");
        notify_value = ulTaskNotifyTake(pdTRUE, I2C_MAX_WAIT);
        // // printf("notify_value:%lu\r\n", notify_value);
        if (notify_value == 1) {
            return true;
        }
    } 
    return false;
}

bool I2C1Read(uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
    i2c1_notify = xTaskGetCurrentTaskHandle();
    if(HAL_I2C_Master_Receive_DMA(&hi2c1, DevAddress, pData, Size) == HAL_OK) {
        // printf("ulTaskNotifyTake\r\n");
        notify_value = ulTaskNotifyTake(pdTRUE, I2C_MAX_WAIT);
        // printf("%lu\r\n", notify_value);
        if (notify_value == 1) {
            return true;
        }
    } 
    return false;
}

bool I2C1Write(uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
    i2c1_notify = xTaskGetCurrentTaskHandle();
    if(HAL_I2C_Master_Transmit_DMA(&hi2c1, DevAddress, pData, Size) == HAL_OK) {
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
    i2c1_notify = NULL;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // printf("MemTxCpltCallback\r\n");
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(i2c1_notify, &xHigherPriorityTaskWoken);
    i2c1_notify = NULL;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // printf("HAL_I2C_MasterTxCpltCallback\r\n");
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(i2c1_notify, &xHigherPriorityTaskWoken);
    i2c1_notify = NULL;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // printf("HAL_I2C_MasterRxCpltCallback\r\n");
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(i2c1_notify, &xHigherPriorityTaskWoken);
    i2c1_notify = NULL;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================== Test =================================
void TestI2C1MenRead() {
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("===========TestI2C1MenRead===========\r\n");
    uint8_t i2c_data = 0;

    // printf("Test HAL_I2C_Mem_Read for MPU6050\r\n");
    // if(HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x75, 1, &i2c_data, 1, 1000) == HAL_OK) {
    //     printf("Get data: %x\r\n", i2c_data);
    // } else {
    //     printf("Get not get data!\r\n");
    // }

    // vTaskDelay(pdMS_TO_TICKS(100));

    // i2c_data = 0;
    // printf("\r\nTest I2C1MenRead for MPU6050\r\n");
    // if(I2C1MenRead(0xD0, 0x75, 1, &i2c_data, 1)) {
    //     printf("Get data: %x\r\n", i2c_data);
    // } else {
    //     printf("Get not get data!\r\n");
    // }

    if(HAL_I2C_Mem_Read_DMA(&hi2c1, 0xD0, 0x75, 1, &i2c_data, 1) == HAL_OK) {
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        printf("Get data: %x\r\n", i2c_data);
    } else {
        printf("Get not get data!\r\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_data = 0X80;
    printf("\r\nTest HAL_I2C_Mem_Write_DMA\r\n");
    if(HAL_I2C_Mem_Write_DMA(&hi2c1, 0xD0, 0x6B, 1, &i2c_data, 1) == HAL_OK) {
        printf("Get data!\r\n");
    } else {
        printf("Get not get data!\r\n");
    }

    printf("=====================================\r\n\r\n");



}