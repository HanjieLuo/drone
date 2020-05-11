#include "sensors.h"

void MPU6050Task(void *param);
void MS5611Task(void *param);
void HMC5883LTask(void *param);

// static TaskHandle_t mpu6050_task_handle;
// STATIC_MEM_TASK_ALLOC(mpu6050_task, configMINIMAL_STACK_SIZE * 5);

// static TaskHandle_t ms5611_task_handle;
// STATIC_MEM_TASK_ALLOC(ms5611_task, configMINIMAL_STACK_SIZE *5);

// static TaskHandle_t hmc5883l_task_handle;
// STATIC_MEM_TASK_ALLOC(hmc5883l_task, configMINIMAL_STACK_SIZE *5);

bool IS_SENSORS_OK = false;
const TickType_t snsors_read_task_wait = pdMS_TO_TICKS(10); 

bool SensorsInit(void) {
    // mpu6050_task_handle = STATIC_MEM_TASK_CREATE(mpu6050_task, MPU6050Task, "MPU6050Task", NULL, 1);
    // ms5611_task_handle = STATIC_MEM_TASK_CREATE(ms5611_task, MS5611Task, "MS5611Task", NULL, 1);
    // hmc5883l_task_handle = STATIC_MEM_TASK_CREATE(hmc5883l_task, HMC5883LTask, "HMC5883LTask", NULL, 1);

    bool flag_MPU6050 = MPU6050Init();
    bool flag_HMC5883L = HMC5883LInit();
    bool flag_MS5611 = MS5611Init();
    
    IS_SENSORS_OK = flag_MPU6050 & flag_HMC5883L & flag_MS5611;
    return IS_SENSORS_OK;
}

void SnsorsReadTask(void) {
    if (!IS_SENSORS_OK) return;

    int16_t acc_raw[3];
    int16_t gyr_raw[3];
    int16_t mag_raw[3];

    // int16_t* acc_raw  = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* gyr_raw = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* mag_raw = pvPortMalloc(sizeof(int16_t) * 3);

    portTickType last_wait_time = xTaskGetTickCount();
    for(;;) {
        MPU6050ReadAccelRaw(acc_raw);
        MPU6050ReadGyroRaw(gyr_raw);
        HMC5883LReadMagRaw(mag_raw);

        vTaskDelayUntil(&last_wait_time, snsors_read_task_wait);
    }
}

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