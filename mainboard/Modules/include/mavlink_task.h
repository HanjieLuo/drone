#ifndef __mavlink_task_H
#define __mavlink_task_H

#include "FreeRTOS.h"
#include "queue.h"
#include "MAVLink/common/mavlink.h"

#include "usart2.h"
#include "utils.h"
#include "motor.h"

void MavlinkInit(void);
static void MavlinkTask(void *param);
static void MavlinkProcessMsg(mavlink_message_t *msg);
// static void MavlinkParse(uint8_t *buffer, size_t len);


#endif /* __mavlink_task_H */
