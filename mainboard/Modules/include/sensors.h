#ifndef __GY_86_H
#define __GY_86_H

#include "utils.h"
#include "mpu6050.h"
#include "ms5611.h"

void SensorsInit(void);
void MPU6050Task(void *param);
void MS5611Task(void *param);

#endif /* __GY_86_H */
