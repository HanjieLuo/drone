#ifndef __mavlink_task_H
#define __mavlink_task_H

#include "FreeRTOS.h"
#include "queue.h"
#include "MAVLink/common/mavlink.h"

#include "utils/utils.h"
#include "usart_com.h"
#include "motor.h"
#include "sensors_task.h"

void MavlinkInit(void);

#endif /* __mavlink_task_H */
