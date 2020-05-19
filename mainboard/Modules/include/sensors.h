#ifndef __SENSORS_H
#define __SENSORS_H

#include "FreeRTOS.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "queue.h"
#include "utils.h"

#define SensorsReadAccelRaw MPU6050ReadAccelRaw
#define SensorsReadGyroRaw MPU6050ReadGyroRaw
#define SensorsReadMagRaw HMC5883LReadMagRaw


// Acc Calibration params
const float Ta00 = 1, Ta01 = 9.20578e-05, Ta02 = -0.00458463;
const float Ta10 = 0, Ta11 = 1, Ta12 = -0.00704855;
const float Ta20 = 0, Ta21 = 0, Ta22 = 1;
const Sa_x = 0.999055, Sa_y = 0.996069, Sa_z = 0.995043;
const Ba_x = -0.563108, Ba_y = -0.0599016, Ba_z = -0.251484;

// Gyro Calibration params
const float Tg00 = 1, Tg01 = -0.000635797, Tg02 = 0.00508567;
const float Tg10 = -0.00272401, Tg11 = 1, Tg12 = 0.00577775;
const float Tg20 = 0.0082499, Tg21 = 0.012745, Tg22 = 1;
const Sg_x = 0.997496, Sg_y = 0.992593, Sg_z = 1.00596;
const Bg_x = 0.0250904, Bg_y = 0.00282511, Bg_z = -0.00949097;

void SensorsInit(void);
void SensorsReadTask(void *param);

void IMUCalibration(const int16_t *acc_raw, const int16_t *gyro_raw, const int16_t *mag_raw, float *acc, float *gyro, float *mag);

#endif /* __SENSORS_H */
