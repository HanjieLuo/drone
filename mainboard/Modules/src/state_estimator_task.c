#include "state_estimator_task.h"

extern xQueueHandle acc_queue;
extern xQueueHandle gyro_queue;
extern xQueueHandle mag_queue;
extern xQueueHandle alt_queue;

static TaskHandle_t state_estimator_task_handle;
STATIC_MEM_TASK_ALLOC(state_estimator_task, STATE_ESTIMATOR_TASK_STACKSIZE);

void StateEstimatorLaunch(void) {
    state_estimator_task_handle = STATIC_MEM_TASK_CREATE(state_estimator_task, StateEstimatorTask, "StateEstimatorTask", NULL, STATE_ESTIMATOR_TASK_PRI);
}

void StateEstimatorTask(void *param) {
    uint64_t timestamp;
    float acc[3], gyro[3], mag[3], alt;

    const TickType_t wait_time = pdMS_TO_TICKS(1);
    portTickType last_wait_time = xTaskGetTickCount();
    for (;;) {
        SensorsReadIMU(&timestamp, acc, gyro);
        
        EskfPredict(&timestamp, acc, gyro);

        // xQueuePeek(mag_queue, mag, portMAX_DELAY);
        // xQueuePeek(alt_queue, &alt, portMAX_DELAY);

        // printf("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], alt); 
        
    //     // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
    //     // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    //     // printf("Mag: %d, %d, %d\r\n\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);
        // printf("%llu, %f, %f, %f, %f, %f, %f\r\n", timestamp, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]); 
        // printf("%llu\r\n", timestamp); 

        vTaskDelayUntil(&last_wait_time, wait_time);
    }
}