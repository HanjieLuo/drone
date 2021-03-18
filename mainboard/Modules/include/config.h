#ifndef __CONFIG_H
#define __CONFIG_H

// Task priorities. Higher number higher priority
#define SENSORS_TASK_PRI        4
#define SYSTEM_TASK_PRI         2


//Task stack sizes
#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE        (2 * configMINIMAL_STACK_SIZE)

#endif /* __CONFIG_H */

