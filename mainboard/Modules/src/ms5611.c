#include "ms5611.h"

static uint16_t prom[6];

bool MS5611Init(void) {
    // 读取 MS5611 传感器，大致分为四步骤:
    // 1. RESET 芯片(delay 10ms)
    // 2. 读取 PROM 中的校准数据(（0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xAC,0xAE）共 8 x 16bit 的数据, delay 10ms)
    // 3. 启动 AD 读取温度和压力的 RAW 数据
    // 4. 根据校准值、温度和压力的 RAW 值计算出真实压力和温度值。

    if(HAL_I2C_IsDeviceReady(&hi2c1, MS5611_ADDR, 10, 1000) != HAL_OK) { return false;}

    //RESET 芯片(delay 10ms)
    uint8_t command = 0x1e;
    if (!I2C1Write(MS5611_ADDR, &command, 1)) {return false; }
	vTaskDelay(pdMS_TO_TICKS(10));

    // printf("MS5611Init\r\n");

    //读取 PROM 中的校准数据(（0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xAC,0xAE）共 8 x 16bit 的数据, delay 10ms)
    uint8_t prom_raw[2] = {0};
    for(uint8_t i = 0; i < 6; i++) {
        command = MS5611_PROM_C1 + i * 2;
        if (!I2C1MenRead(MS5611_ADDR, command, 1, prom_raw, 2)) {return false;}
        prom[i] = (int16_t)(prom_raw[0] << 8 | prom_raw[1]);
        printf("%u: %u\r\n", i, prom[i]);
    }


    // command = MS5611_PROM_C1;
    // if (!I2C1MenRead(MS5611_ADDR, command, 1, prom_raw, 2)) { printf("I2C1MenRead error\r\n"); return false; }
    // printf("%u: %u\r\n", prom_raw[0], prom_raw[1]);

    // if (!I2C1Write(MS5611_ADDR, MS5611_RESET, 1)) { return false; }
    // prom_raw[0] = MS5611_RESET;
    // vTaskDelay(pdMS_TO_TICKS(100));
    // uint8_t RESET_MS5611 = 0x1E;;
    // while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &RESET_MS5611, 1, 1000)!= HAL_OK);
    // printf("MS5611Init\r\n");
    // vTaskDelay(pdMS_TO_TICKS(10));

    // printf("MS5611Init\r\n");

    // uint8_t PROM_READ_C1_MS5611 = 0xAA;
    // uint8_t prom_raw[2] = {0};
    // while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &PROM_READ_C1_MS5611, 1, 1000) != HAL_OK);
	// while (HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, prom_raw, 2, 1000) != HAL_OK);

    // HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, MS5611_PROM_C1, 1, prom_raw, 2, 1000);
    // if (!I2C1MenRead(MS5611_ADDR, MS5611_PROM_C1, 1, prom_raw, 2)) { printf("I2C1MenRead error\r\n"); return false; }
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, MS5611_PROM_C2, 1, prom_raw, 2, 1000);
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, MS5611_PROM_C3, 1, prom_raw, 2, 1000);
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, MS5611_PROM_C4, 1, prom_raw, 2, 1000);
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, MS5611_PROM_C5, 1, prom_raw, 2, 1000);
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDR, MS5611_PROM_C6, 1, prom_raw, 2, 1000);
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // if (!I2C1MenRead(MS5611_ADDR, MS5611_PROM_C2, 1, prom_raw, 2)) { printf("I2C1MenRead error\r\n"); return false; }
    // printf("%u: %u\r\n", prom[0], prom[1]);

    // if (!I2C1MenRead(MS5611_ADDR, MS5611_PROM_C3, 1, prom_raw, 2)) { printf("I2C1MenRead error\r\n"); return false; }
    // printf("%u: %u\r\n", prom[0], prom[1]);




    return true;
}