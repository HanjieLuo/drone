#include "state_estimator_task.h"

extern xQueueHandle acc_queue;
extern xQueueHandle gyro_queue;
extern xQueueHandle mag_queue;
extern xQueueHandle alt_queue;

static TaskHandle_t state_estimator_task_handle;
STATIC_MEM_TASK_ALLOC(state_estimator_task, STATE_ESTIMATOR_TASK_STACKSIZE);

void StateEstimatorLaunch(void) {
    // EskfInit();

    state_estimator_task_handle = STATIC_MEM_TASK_CREATE(state_estimator_task, StateEstimatorTask, "StateEstimatorTask", NULL, STATE_ESTIMATOR_TASK_PRI);
}

void StateEstimatorTask(void *param) {
    // int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
    float acc[3], gyro[3], mag[3], alt;

    const TickType_t wait_time = pdMS_TO_TICKS(1000);
    portTickType last_wait_time = xTaskGetTickCount();
    for (;;) {
        xQueuePeek(acc_queue, acc, portMAX_DELAY);
        xQueuePeek(gyro_queue, gyro, portMAX_DELAY);
        xQueuePeek(mag_queue, mag, portMAX_DELAY);
        xQueuePeek(alt_queue, &alt, portMAX_DELAY);

        printf("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], alt); 
        
    //     // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
    //     // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    //     // printf("Mag: %d, %d, %d\r\n\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);


        vTaskDelayUntil(&last_wait_time, wait_time);
    }
}