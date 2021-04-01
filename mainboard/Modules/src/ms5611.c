#include "ms5611.h"

// 读取 MS5611 传感器，大致分为四步骤:
// 1. RESET 芯片(delay 10ms)
// 2. 读取 PROM 中的校准数据(（0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xAC,0xAE）共 8 x 16bit 的数据, delay 10ms)
// 3. 启动 AD 读取温度和压力的 RAW 数据
// 4. 根据校准值、温度和压力的 RAW 值计算出真实压力和温度值。

static uint8_t command;
static uint8_t buffer[3];

// static uint16_t SENS_t1;   // Pressure sensitivity
// static uint16_t OFF_t1;    // Pressure offset
static int64_t SENS_t2;    // Pressure sensitivity
static int64_t OFF_t2;     // Pressure offset
static uint16_t TCS;       // Temperature coefficient of pressure sensitivity
static uint16_t TCO;       // Temperature coefficient of pressure offset
static uint16_t TREF;      // Reference temperature
static uint16_t TEMPSENS;  // Temperature coefficient of the temperature

// static int32_t temperature;  //° *100
// static int32_t pressure;     //pa *100
// static float altitude;       //m

bool MS5611Init(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, MS5611_ADDR, 10, 1000) != HAL_OK) {
        return false;
    }

    //RESET 芯片(delay 10ms)
    command = 0x1e;
    // I2C1Write(MS5611_ADDR, &command, 1);
    if (!I2C1Write(MS5611_ADDR, &command, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // printf("MS5611Init\r\n");

    //读取 PROM 中的校准数据(（0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xAC,0xAE）共 8 x 16bit 的数据, delay 10ms)
    uint8_t prom_raw[2] = {0};
    uint16_t prom[6];
    for (uint8_t i = 0; i < 6; i++) {
        command = MS5611_PROM_C1 + i * 2;
        // I2C1MenRead(MS5611_ADDR, command, 1, prom_raw, 2);
        if (!I2C1MenRead(MS5611_ADDR, command, 1, prom_raw, 2)) {
            return false;
        }
        prom[i] = (int16_t)(prom_raw[0] << 8 | prom_raw[1]);
        // printf("%u: %u\r\n", i, prom[i]);
    }

    SENS_t2  = (int64_t)prom[0] << 15;
    OFF_t2   = (int64_t)prom[1] << 16;
    TCS      = prom[2];
    TCO      = prom[3];
    TREF     = prom[4];
    TEMPSENS = prom[5];

    vTaskDelay(pdMS_TO_TICKS(10));

    return true;
}

bool MS5611ReadRawTemp(uint32_t *value) {
    command = MS5611_D2_OSR_4096;
    // I2C1Write(MS5611_ADDR, &command, 1);
    if (!I2C1Write(MS5611_ADDR, &command, 1)) {return false;}

    vTaskDelay(pdMS_TO_TICKS(MS5611_CONVERSION_OSR_4096));

    command = MS5611_OSR_256;
    // I2C1Write(MS5611_ADDR, &command, 1);
    // I2C1Read(MS5611_ADDR, buffer, 3);
    if (!I2C1Write(MS5611_ADDR, &command, 1)) { return false;}
    if (!I2C1Read(MS5611_ADDR, buffer, 3)) { return false;}

    *value = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    return true;
}

bool MS5611ReadRawPress(uint32_t *value) {
    command = MS5611_D1_OSR_4096;
    // I2C1Write(MS5611_ADDR, &command, 1);
    if (!I2C1Write(MS5611_ADDR, &command, 1)) {return false;}

    vTaskDelay(pdMS_TO_TICKS(MS5611_CONVERSION_OSR_4096));

    command = MS5611_OSR_256;
    // I2C1Write(MS5611_ADDR, &command, 1);
    // I2C1Read(MS5611_ADDR, buffer, 3);
    if (!I2C1Write(MS5611_ADDR, &command, 1)) {return false;}
    if (!I2C1Read(MS5611_ADDR, buffer, 3)) {return false;}

    *value = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    return true;
}

bool MS5611ReadAlt(float* alt) {
    int32_t dT;
    int64_t TEMP, OFF, SENS;
    uint32_t D1, D2;

    // MS5611ReadRawPress(&D1);
    // MS5611ReadRawTemp(&D2);
    if (!MS5611ReadRawPress(&D1)) {return false;}
    if (!MS5611ReadRawTemp(&D2)) {return false;}

    dT   = D2 - ((int32_t)TREF << 8);                         //实际和参考温度之间的差
    TEMP = 2000 + (((int64_t)dT * (int64_t)TEMPSENS) >> 23);  //实际温度, ° *100
    OFF  = OFF_t2 + (((int64_t)TCO * (int64_t)dT) >> 7);      //实际温度的零偏
    SENS = SENS_t2 + (((int64_t)TCS * (int64_t)dT) >> 8);     //实际温度的灵敏度

    //二阶温度补偿参考MS5611数据手册的第9页
    if (TEMP < 2000) {  //当温度小于20°时二阶温度补偿
        int64_t T2    = ((int64_t)dT * (int64_t)dT) >> 31;
        int64_t OFF2  = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
        int64_t SENS2 = OFF2 >> 1;
        if (TEMP < -1500)  //当温度小于-15°时
        {
            OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += (11 * (TEMP + 1500) * (TEMP + 1500)) >> 1;
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    // temperature = TEMP; //° *100
    float pressure    = (float)( (((D1 * SENS) >> 21) - OFF) >> 15 );  //单位为Pa, pa *100

	*alt = 44330.77f * (1.0f - pow((pressure/101325.0f), 0.1902632f));
    return true;
}