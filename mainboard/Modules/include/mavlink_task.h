#ifndef __mavlink_task_H
#define __mavlink_task_H

#include "FreeRTOS.h"
#include "queue.h"
#include "MAVLink/common/mavlink.h"

#include "usart2.h"
#include "utils.h"
#include "motor.h"
#include "sensors.h"

void MavlinkInit(void);

#endif /* __mavlink_task_H */
