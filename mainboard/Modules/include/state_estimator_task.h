#ifndef __STATE_ESTIMATOR_TASK_H
#define __STATE_ESTIMATOR_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "utils/utils.h"
#include "sensors.h"
#include "eskf.h"

void StateEstimatorLaunch(void);
void StateEstimatorTask(void *param);

#endif /* __STATE_ESTIMATOR_TASK_H */
