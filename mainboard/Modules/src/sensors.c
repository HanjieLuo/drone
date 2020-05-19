#include "sensors.h"

xQueueHandle acc_raw_queue;
STATIC_MEM_QUEUE_ALLOC(acc_raw_queue, 1, sizeof(float) * 3);
xQueueHandle gyro_raw_queue;
STATIC_MEM_QUEUE_ALLOC(gyro_raw_queue, 1, sizeof(float) * 3);
xQueueHandle mag_raw_queue;
STATIC_MEM_QUEUE_ALLOC(mag_raw_queue, 1, sizeof(float) * 3);

static TaskHandle_t sensors_read_task_handle;
STATIC_MEM_TASK_ALLOC(sensors_read_task, configMINIMAL_STACK_SIZE * 3);

const TickType_t sensors_read_task_wait = pdMS_TO_TICKS(1); 

// void MPU6050Task(void *param);
// void MS5611Task(void *param);
// void HMC5883LTask(void *param);
// static TaskHandle_t mpu6050_task_handle;
// STATIC_MEM_TASK_ALLOC(mpu6050_task, configMINIMAL_STACK_SIZE * 5);
// static TaskHandle_t ms5611_task_handle;
// STATIC_MEM_TASK_ALLOC(ms5611_task, configMINIMAL_STACK_SIZE *5);
// static TaskHandle_t hmc5883l_task_handle;
// STATIC_MEM_TASK_ALLOC(hmc5883l_task, configMINIMAL_STACK_SIZE *5);

void SensorsInit(void) {
    acc_raw_queue = STATIC_MEM_QUEUE_CREATE(acc_raw_queue);
    gyro_raw_queue = STATIC_MEM_QUEUE_CREATE(gyro_raw_queue);
    mag_raw_queue = STATIC_MEM_QUEUE_CREATE(mag_raw_queue);

    sensors_read_task_handle = STATIC_MEM_TASK_CREATE(sensors_read_task, SensorsReadTask, "SensorsReadTask", NULL, 5);
    // mpu6050_task_handle = STATIC_MEM_TASK_CREATE(mpu6050_task, MPU6050Task, "MPU6050Task", NULL, 1);
    // ms5611_task_handle = STATIC_MEM_TASK_CREATE(ms5611_task, MS5611Task, "MS5611Task", NULL, 1);
    // hmc5883l_task_handle = STATIC_MEM_TASK_CREATE(hmc5883l_task, HMC5883LTask, "HMC5883LTask", NULL, 1);
}

void SensorsReadTask(void *param) {
    bool IS_SENSORS_OK = MPU6050Init();
    IS_SENSORS_OK &= HMC5883LInit();
    IS_SENSORS_OK &= MS5611Init();
    if (!IS_SENSORS_OK) return;

    int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
    float acc[3], gyro[3], mag[3];

    portTickType last_wait_time = xTaskGetTickCount();
    for(;;) {
        MPU6050ReadAccelRaw(acc_raw);
        MPU6050ReadGyroRaw(gyro_raw);
        HMC5883LReadMagRaw(mag_raw);

        IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);

        // xQueueOverwrite(acc_raw_queue, acc_raw);
        // xQueueOverwrite(gyro_raw_queue, gyro_raw);
        // xQueueOverwrite(mag_raw_queue, mag_raw);

        // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
        // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        // printf("Mag: %d, %d, %d\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);

        vTaskDelayUntil(&last_wait_time, sensors_read_task_wait);
    }

    // int16_t* acc_raw  = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* gyr_raw = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* mag_raw = pvPortMalloc(sizeof(int16_t) * 3);
}

void IMUCalibration(const int16_t *acc_raw, const int16_t *gyro_raw, const int16_t *mag_raw, float *acc, float *gyro, float *mag) {


}

/*
void MPU6050Task(void *param) {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float celsius;

    if(!MPU6050Init()) return;

    for(;;) {
        printf("================\r\n");
        MPU6050ReadAccel(&accel_x, &accel_y, &accel_z);
        printf("Accel: %f, %f, %f\r\n", accel_x, accel_y, accel_z);

        MPU6050ReadGyro(&gyro_x, &gyro_y, &gyro_z);
        printf("Gyro: %f, %f, %f\r\n", gyro_x, gyro_y, gyro_z);
       
        MPU6050ReadTemp(&celsius);
        printf("Temp: %f\r\n", celsius);
        printf("================\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
*/

/*
void MS5611Task(void *param) {
    // if(!MPU6050Init()) return;
    // uint32_t temp_raw, press_raw;
    float alt;

    if(!MS5611Init()) return;

    for(;;) {
        printf("================\r\n");
        MS5611Calculate();
        MS5611ReadAlt(&alt);
        printf("Alt: %f\r\n", alt);

        // MS5611ReadRawTemp(&temp_raw);
        // printf("Temp: %lu\r\n", temp_raw);

        // MS5611ReadRawPress(&press_raw);
        // printf("Press: %lu\r\n", press_raw);
        printf("================\r\n");
        // vTaskDelay(pdMS_TO_TICKS(1));      
    }
}
*/

/*
void HMC5883LTask(void *param) {
    float mx, my, mz;

    if(!MPU6050Init()) return;
    if(!HMC5883LInit()) return;

    for(;;) { 
        printf("================\r\n");
        HMC5883LReadMag(&mx, &my, &mz);
        printf("Mag: %f, %f, %f\r\n", mx, my, mz);
        printf("================\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
*/