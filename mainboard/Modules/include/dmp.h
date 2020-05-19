#ifndef __DMP_H
#define __DMP_H

#include "FreeRTOS.h"
#include "queue.h"
#include "sensors.h"
#include "utils.h"

void DmpInit(void);
void DmpTask(void *param);

#endif /* __DMP_H */
