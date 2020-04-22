#ifndef __I2C1_H
#define __I2C1_H

#include "i2c.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"

void I2C1Init(void);
bool I2C1Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
bool I2C1Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif /* __I2C1_H */
