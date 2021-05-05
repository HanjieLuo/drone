#ifndef __SENSORS_TASK_H
#define __SENSORS_TASK_H

// #define ARM_MATH_CM4
// #include "arm_math.h"

#include "FreeRTOS.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "queue.h"
#include "utils/utils.h"
#include "filter.h"
#include "sensors.h"
#include "system_task.h"

void SensorsLaunch(void);
void SensorsTask(void *param);

#endif /* __SENSORS_TASK_H */
