#include "system_task.h"

static bool is_init = false;

STATIC_MEM_TASK_ALLOC(system_task, SYSTEM_TASK_STACKSIZE);
static void SystemTask(void *param);


void SystemLaunch(void) {
    STATIC_MEM_TASK_CREATE(system_task, SystemTask, "SystemTask", NULL, SYSTEM_TASK_PRI);
}

void SystemTask(void *param) {
    SystemInitiate();
}

void SystemInitiate(void) {
    if(is_init) return;
    
    bool is_ok = SensorsInit();

    is_init = is_ok;
    printf("SystemInitiate: %u\r\n\r\n", is_init);
}

void SystemWaitStart(void) {
    if(!is_init) {
        vTaskDelay(2);
    }
}