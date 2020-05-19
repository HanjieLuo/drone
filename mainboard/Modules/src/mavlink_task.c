#include "mavlink_task.h"

extern xQueueHandle usart2_queue;
// extern xQueueHandle acc_raw_queue;
// extern xQueueHandle gyr_raw_queue;
// extern xQueueHandle mag_raw_queue;

static TaskHandle_t mavlink_task_handle;
STATIC_MEM_TASK_ALLOC(mavlink_task, configMINIMAL_STACK_SIZE * 3);

static mavlink_message_t msg;
static mavlink_status_t status;
static int mavlink_comm = MAVLINK_COMM_0;
static mavlink_rc_channels_override_t rc_channels_override;

static void MavlinkTask(void *param);
static void MavlinkProcessMsg(mavlink_message_t *msg);
// static void MavlinkParse(uint8_t *buffer, size_t len);

void MavlinkInit(void) {
    Usart2Init();
    mavlink_task_handle = STATIC_MEM_TASK_CREATE(mavlink_task, MavlinkTask, "MavlinkTask", NULL, 1);
}

// extern TIM_HandleTypeDef htim4;

void MavlinkTask(void *param) {
    // // uint8_t buffer1[] = "Hello world\r\n";
    // // uint16_t compare = 0;

    // usart2_queue_buffer buffer;
    int16_t acc_raw[3], gyr_raw[3], mag_raw[3];
    uint16_t msg_size;
    uint8_t msg_buffer[128];

    const TickType_t wait_time = pdMS_TO_TICKS(10);
    portTickType last_wait_time = xTaskGetTickCount();
    for (;;) {
        // if (xQueueReceive(usart2_queue, &buffer, portMAX_DELAY) == pdTRUE) {
        //     for (size_t i = 0; i < buffer.length; i++) {
        //         if (mavlink_parse_char(mavlink_comm, buffer.addr[i], &msg, &status)) {
        //             MavlinkProcessMsg(&msg);
        //         }
        //     }
        // }

        // xQueueReceive(acc_raw_queue, acc_raw, wait_time);
        // xQueueReceive(gyr_raw_queue, gyr_raw, wait_time);
        // xQueueReceive(mag_raw_queue, mag_raw, wait_time);

        // mavlink_msg_raw_imu_pack(1, 200, &msg, pdTICKS_TO_US(xTaskGetTickCount()),
        //                          acc_raw[0], acc_raw[1], acc_raw[2],
        //                          gyr_raw[0], gyr_raw[1], gyr_raw[2],
        //                          mag_raw[0], mag_raw[1], mag_raw[2], 0, 0);
        
        // // 37bytes
        // msg_size = mavlink_msg_to_send_buffer(msg_buffer, &msg);

        // // HAL_UART_Transmit_DMA(&huart2, msg_buffer, msg_size);
        // HAL_UART_Transmit_DMA(&huart1, msg_buffer, msg_size);

        // printf("%ld\r\n", (long)pdTICKS_TO_US(xTaskGetTickCount()));
        // printf("%u\r\n", msg_size);

        // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
        // printf("Gyro: %d, %d, %d\r\n", gyr_raw[0], gyr_raw[1], gyr_raw[2]);
        // printf("Mag: %d, %d, %d\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);

        // vTaskDelayUntil(&last_wait_time, wait_time);
    }
}

void MavlinkProcessMsg(mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_msg_rc_channels_override_decode(msg, &rc_channels_override);
            SetMotor(rc_channels_override.chan1_raw, rc_channels_override.chan2_raw, rc_channels_override.chan3_raw, rc_channels_override.chan4_raw);
            printf("===============\r\n");
            printf("MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:%u\r\n", msg->seq);
            printf("RC channel 1: %u\r\n", rc_channels_override.chan1_raw);
            printf("RC channel 2: %u\r\n", rc_channels_override.chan2_raw);
            printf("RC channel 3: %u\r\n", rc_channels_override.chan3_raw);
            printf("RC channel 4: %u\r\n", rc_channels_override.chan4_raw);
            printf("===============\r\n");
            break;
        }
        default: {
            break;
        }
    }
    // printf("===============\r\n");
    // printf("%u\r\n", msg->msgid);
    // printf("%u\r\n", msg->seq);
    // printf("%u\r\n", msg->compid);
    // printf("%u\r\n", msg->sysid);
    // printf("===============\r\n");
}
