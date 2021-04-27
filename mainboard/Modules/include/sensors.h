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
#define SensorsReadAlt MS5611ReadAlt
#define SensorsReadMagRaw HMC5883LReadMagRaw

bool SensorsInit(void);
void SensorsReadIMU(uint64_t *timestamp, float *acc, float *gyro);


#endif /* __SENSORS_H */
