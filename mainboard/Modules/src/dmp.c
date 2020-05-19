#include "dmp.h"

extern xQueueHandle acc_raw_queue;
extern xQueueHandle gyro_raw_queue;
extern xQueueHandle mag_raw_queue;

static TaskHandle_t dmp_task_handle;
STATIC_MEM_TASK_ALLOC(dmp_task, configMINIMAL_STACK_SIZE * 2);

void DmpInit(void) {
    dmp_task_handle = STATIC_MEM_TASK_CREATE(dmp_task, DmpTask, "DmpTask", NULL, 4);
}

void DmpTask(void *param) {
    // int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
    // float acc[3], gyro[3], mag[3];

    // const TickType_t wait_time = pdMS_TO_TICKS(20);
    // portTickType last_wait_time = xTaskGetTickCount();
    // for (;;) {
    //     xQueueReceive(acc_raw_queue, acc_raw, portMAX_DELAY);
    //     xQueueReceive(gyro_raw_queue, gyro_raw, portMAX_DELAY);
    //     xQueueReceive(mag_raw_queue, mag_raw, portMAX_DELAY);

    //     // usecTimestamp

    //     // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
    //     // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    //     // printf("Mag: %d, %d, %d\r\n\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);


    //     vTaskDelayUntil(&last_wait_time, wait_time);
    // }
}