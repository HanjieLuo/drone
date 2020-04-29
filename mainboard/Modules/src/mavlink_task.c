#include "mavlink_task.h"

extern xQueueHandle usart2_queue;
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
    // // const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    // // uint16_t compare = 0;
    usart2_queue_buffer buffer;
    for(;;) {
        if (xQueueReceive(usart2_queue, &buffer, portMAX_DELAY) == pdTRUE) {
            for (size_t i = 0; i < buffer.length; i++) {
                if (mavlink_parse_char(mavlink_comm, buffer.addr[i], &msg, &status)) {
                    MavlinkProcessMsg(&msg);
                }
            }
        }
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



