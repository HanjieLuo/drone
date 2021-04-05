#ifndef __SYSTEM_TASK_H
#define __SYSTEM_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "config.h"
#include "utils.h"
#include "sensors_task.h"
#include "state_estimator_task.h"

void SystemLaunch(void);
// This permits to guarantee that the system task is initialized before other tasks waits for the start event.
void SystemWaitStart(void);

void SystemInitiate(void);


#endif /* __SYSTEM_TASK_H */
