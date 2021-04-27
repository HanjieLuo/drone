#include "sensors_task.h"



extern xQueueHandle acc_queue;
extern xQueueHandle gyro_queue;
extern xQueueHandle mag_queue;
extern xQueueHandle alt_queue;

static TaskHandle_t sensors_task_handle;
STATIC_MEM_TASK_ALLOC(sensors_task, SENSORS_TASK_STACKSIZE);

const TickType_t sensors_task_wait = pdMS_TO_TICKS(1000.0 / IMU_SAMPLE_FREQ);
const static uint8_t mag_meas_delay = IMU_SAMPLE_FREQ / MAG_SAMPLE_FREQ;
const static uint8_t alt_meas_delay = IMU_SAMPLE_FREQ / ALT_SAMPLE_FREQ;

static Lowpass2Data acc_lpf[3];
static Lowpass2Data gyro_lpf[3];

void SensorsLaunch(void) {
    for(unsigned int i = 0; i < 3; i++) {
        Lowpass2Init(&acc_lpf[i], IMU_SAMPLE_FREQ, ACC_LPF_CUTOFF_FREQ);
        Lowpass2Init(&gyro_lpf[i], IMU_SAMPLE_FREQ, GYRO_LPF_CUTOFF_FREQ);
    }

    sensors_task_handle = STATIC_MEM_TASK_CREATE(sensors_task, SensorsTask, "SensorsTask", NULL, SENSORS_TASK_PRI);
}


void SensorsTask(void *param) {
    // SystemWaitStart();

    int16_t acc_raw[3] = {0}, gyro_raw[3] = {0}, mag_raw[3] = {0};
    float alt = 0.f, acc[3] = {0}, gyro[3] = {0}, mag[3] = {0};

    uint8_t mag_meas_count = mag_meas_delay;
    uint8_t alt_meas_count = alt_meas_delay;

    portTickType last_wait_time = xTaskGetTickCount();

    while(1) {
        SensorsReadAccelRaw(acc_raw);
        SensorsReadGyroRaw(gyro_raw);
      
        CalibrateIMU(acc_raw, gyro_raw, acc, gyro);

        Lowpass2Apply3Axis((Lowpass2Data*)(&acc_lpf), acc);
        Lowpass2Apply3Axis((Lowpass2Data*)(&gyro_lpf), gyro);

        xQueueOverwrite(acc_queue, acc);
        xQueueOverwrite(gyro_queue, gyro);

        if (--mag_meas_count == 0) {
            SensorsReadMagRaw(mag_raw);
            CalibrateMag(mag_raw, mag);
            xQueueOverwrite(mag_queue, mag);
            mag_meas_count = mag_meas_delay;
        }

        if (--alt_meas_count == 0) {
            SensorsReadAlt(&alt);
            xQueueOverwrite(alt_queue, &alt);
            alt_meas_count = alt_meas_delay;
        } 

        // printf("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], alt); 
        // printf("%lu\n", (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount())); 
        vTaskDelayUntil(&last_wait_time, sensors_task_wait);

        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

        // vTaskDelay(pdMS_TO_TICKS(5));

        // TickType_t xTimeBefore, xTotalTimeSuspended;
        // xTimeBefore = xTaskGetTickCount();
        // xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
        // printf("%lu\r\n", xTotalTimeSuspended);


        // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
        // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        // printf("Mag: %d, %d, %d\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);

        // printf("Accel: %f %f %f\r\n", acc[0], acc[1], acc[2]);
        // printf("Gyro : %f %f %f\r\n", gyro[0], gyro[1], gyro[2]);
        // printf("Mag  : %f %f %f\r\n\r\n", mag[0], mag[1], mag[2]);

        // printf("%u,%d,%d,%d,%d,%d,%d\n", pdTICKS_TO_MS(xTaskGetTickCount()), acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]);

        // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (float)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]); 

        // printf("%f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (float)pdTICKS_TO_MS(xTaskGetTickCount()), acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2], mag_raw[0], mag_raw[1], mag_raw[2]);

    }

    // int16_t* acc_raw  = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* gyr_raw = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* mag_raw = pvPortMalloc(sizeof(int16_t) * 3);
}





