#ifndef __SENSORS_H
#define __SENSORS_H

// #define ARM_MATH_CM4
// #include "arm_math.h"

#include "FreeRTOS.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "queue.h"
#include "utils.h"
#include "filter.h"
#include "system_task.h"

#define SensorsReadAccelRaw MPU6050ReadAccelRaw
#define SensorsReadGyroRaw MPU6050ReadGyroRaw
#define SensorsReadMagRaw HMC5883LReadMagRaw

bool SensorsInit(void);
void SensorsTask(void *param);

void CalibrateIMU(const int16_t *acc_raw, const int16_t *gyro_raw, float *acc, float *gyro);
void CalibrateMag(const int16_t *mag_raw, float *mag);

#endif /* __SENSORS_H */
