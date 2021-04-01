#ifndef __MS5611_H
#define __MS5611_H

#include "i2c1.h"
#include "utils.h"

//定义器件在IIC总线中的从地址,根据CSB引脚不同修改
// MS5611挂在MPU5060的从I2C接口上。MS5611的I2C地址为0b111011Cx，其中C比特位由CSB引脚决定，为CSB引脚的补码值（取反）。
// GY-86上 MS5611的CSB引脚接地，所以CSB引脚值为0，8位I2C地址为0b1110111x（0xEE），7位I2C地址为 0b1110111（0x77）。
//#define MS561101BA_ADDR  0xec   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS5611_ADDR 0xee  //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

#define MS5611_RESET 0x1E

// #define MS5611_PROM_BASE_ADDR 0xA0  // by adding ints from 0 to 6 we can read all the prom configuration values.
#define MS5611_PROM_C1 0xA2
#define MS5611_PROM_C2 0xA4
#define MS5611_PROM_C3 0xA6
#define MS5611_PROM_C4 0xA8
#define MS5611_PROM_C5 0xAA
#define MS5611_PROM_C6 0xAC

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08

//pressure
#define MS5611_D1_OSR_256 0x40
#define MS5611_D1_OSR_512 0x42
#define MS5611_D1_OSR_1024 0x44
#define MS5611_D1_OSR_2048 0x46
#define MS5611_D1_OSR_4096 0x48

//temperature
#define MS5611_D2_OSR_256 0x50
#define MS5611_D2_OSR_512 0x52
#define MS5611_D2_OSR_1024 0x54
#define MS5611_D2_OSR_2048 0x56
#define MS5611_D2_OSR_4096 0x58

#define MS5611_CONVERSION_OSR_256 1    // 0.60 mSec conversion time (1666.67 Hz)
#define MS5611_CONVERSION_OSR_512 2    // 1.17 mSec conversion time ( 854.70 Hz)
#define MS5611_CONVERSION_OSR_1024 3   // 2.28 mSec conversion time ( 357.14 Hz)
#define MS5611_CONVERSION_OSR_2048 5   // 4.54 mSec conversion time ( 220.26 Hz)
#define MS5611_CONVERSION_OSR_4096 10  // 9.04 mSec conversion time ( 110.62 Hz)

// 定义MS561101BA内部地址
// registers of the device
// #define MS5611_D1 0x40
// #define MS5611_D2 0x50

// D1 and D2 result size (bytes)
// #define MS5611_D1D2_SIZE 3

// C1 will be at 0xA2 and all the subsequent are multiples of 2
// #define MS5611_PROM_REG_COUNT 6  // number of registers in the PROM
// #define MS5611_PROM_REG_SIZE 2   // size in bytes of a prom registry.

bool MS5611Init(void);
bool MS5611ReadRawTemp(uint32_t *value);
bool MS5611ReadRawPress(uint32_t *value);
bool MS5611ReadAlt(float *alt);

// bool MS5611ReadPress();
// bool MS5611ReadTemp(float *celsius);

#endif /* __MS5611_H */
