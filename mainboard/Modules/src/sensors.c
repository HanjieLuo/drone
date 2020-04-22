#include "sensors.h"


static TaskHandle_t mpu6050_task_handle;
STATIC_MEM_TASK_ALLOC(mpu6050_task, configMINIMAL_STACK_SIZE * 3);

void SensorsInit(void) {
    mpu6050_task_handle = STATIC_MEM_TASK_CREATE(mpu6050_task, MPU6050Task, "MPU6050Task", NULL, 1);
}

void MPU6050Task(void *param) {
    MPU6050Init();
    for(;;) {

    }
}