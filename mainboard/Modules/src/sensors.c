#include "sensors.h"

static TaskHandle_t mpu6050_task_handle;
STATIC_MEM_TASK_ALLOC(mpu6050_task, configMINIMAL_STACK_SIZE * 5);

static TaskHandle_t ms5611_task_handle;
STATIC_MEM_TASK_ALLOC(ms5611_task, configMINIMAL_STACK_SIZE *5);

void SensorsInit(void) {
    // mpu6050_task_handle = STATIC_MEM_TASK_CREATE(mpu6050_task, MPU6050Task, "MPU6050Task", NULL, 1);
    ms5611_task_handle = STATIC_MEM_TASK_CREATE(ms5611_task, MS5611Task, "MS5611Task", NULL, 1);
}

void MPU6050Task(void *param) {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float celsius;

    if(!MPU6050Init()) return;

    for(;;) {
        printf("================\r\n");
        if(MPU6050ReadAccel(&accel_x, &accel_y, &accel_z)) {
            printf("Accel: %f, %f, %f\r\n", accel_x, accel_y, accel_z);
        }

        if(MPU6050ReadGyro(&gyro_x, &gyro_y, &gyro_z)) {
            printf("Gyro: %f, %f, %f\r\n", gyro_x, gyro_y, gyro_z);
        }

        if(MPU6050ReadTemp(&celsius)) {
            printf("Temp: %f\r\n", celsius);
        }
        printf("================\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void MS5611Task(void *param) {
    // if(!MPU6050Init()) return;

    if(!MS5611Init()) return;

    for(;;) {

    }

}