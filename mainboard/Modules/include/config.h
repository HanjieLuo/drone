#ifndef __CONFIG_H
#define __CONFIG_H

#define USE_SISTEMVEIW 1

// Task priorities. Higher number higher priority
#define SYSTEMVIEW_TASK_PRI     6
#define SENSORS_TASK_PRI        4
#define SYSTEM_TASK_PRI         2



//Task stack sizes
#define SYSTEM_TASK_STACKSIZE         (2 * configMINIMAL_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE        (2 * configMINIMAL_STACK_SIZE)
#define SYSTEMVIEW_TASK_STACKSIZE     (2 * configMINIMAL_STACK_SIZE)

#endif /* __CONFIG_H */

