#ifndef __mavlink_task_H
#define __mavlink_task_H

#include "uart4.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "MAVLink/common/mavlink.h"

void MavlinkInit(void);
static void MavlinkTask(void *param);
static void MavlinkParse(uint8_t *buffer, size_t len);

#endif /* __mavlink_task_H */
