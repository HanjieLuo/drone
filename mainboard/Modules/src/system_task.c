#include "system_task.h"

static bool is_init = false;

STATIC_MEM_TASK_ALLOC(system_task, SYSTEM_TASK_STACKSIZE);
static void SystemTask(void *param);


void SystemLaunch(void) {
    STATIC_MEM_TASK_CREATE(system_task, SystemTask, "SystemTask", NULL, SYSTEM_TASK_PRI);
}

void SystemTask(void *param) {
    SystemInitiate();

    while(1) {
        vTaskDelay(portMAX_DELAY);
    }


    // uint8_t buffer[] = "Hello world\r\n";
    // while(1) {
    //     HAL_UART_Transmit_DMA(&huart1, buffer, sizeof(buffer));
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}

void SystemInitiate(void) {
    if(is_init) return;
    
    bool is_ok = SensorsInit();

    is_init = is_ok;
    printf("SystemInitiate: %u\r\n\r\n", is_init);
}


void SystemWaitStart(void) {
    if(!is_init) {
        // printf("wait\r\n");
        vTaskDelay(2);
    }
}