#include "mavlink_task.h"

extern xQueueHandle uart4_queue;
static TaskHandle_t mavlink_task_handle;
STATIC_MEM_TASK_ALLOC(mavlink_task, configMINIMAL_STACK_SIZE * 3);

static mavlink_message_t msg;
static mavlink_status_t status;
static int mavlink_comm = MAVLINK_COMM_0;

void MavlinkInit(void) {
    printf("MavlinkInit");
    Uart4Init();
    mavlink_task_handle = STATIC_MEM_TASK_CREATE(mavlink_task, MavlinkTask, "MavlinkTask", NULL, 1);
}

void MavlinkTask(void *param) {
    uart4_queue_buffer buffer;
    for (;;) {
        if (xQueueReceive(uart4_queue, &buffer, portMAX_DELAY) == pdTRUE) {
            MavlinkParse(buffer.addr, buffer.length);
        }
    }
}

void MavlinkParse(uint8_t *buffer, size_t len) {
    // printf("%u\r\n", len);
    // printf("%.*s\r\n", len, buffer);
    for (size_t i = 0; i < len; i++) {
        // printf("%o", buffer[i]);
        // printf("%c\r\n", buffer[i]);
        if (mavlink_parse_char(mavlink_comm, buffer[i], &msg, &status)) {
            MavlinkParsePayload(&msg);
        }
    }
    printf("%d\r\n", (int)status.parse_state);
    printf("\r\n");
}

void MavlinkParsePayload(mavlink_message_t *msg) {
    printf("===============\r\n");
    printf("%u\r\n", msg->msgid);
    printf("%u\r\n", msg->seq);
    printf("%u\r\n", msg->compid);
    printf("%u\r\n", msg->sysid);
    printf("===============\r\n");
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_rc_channels_override_t rc_channels_override;
			mavlink_msg_rc_channels_override_decode(msg, &rc_channels_override);
            printf("===============\r\n");
            fprintf("RC channel 1: %u\r\n", rc_channels_override.chan1_raw);
            fprintf("RC channel 2: %u\r\n", rc_channels_override.chan2_raw);
            fprintf("RC channel 3: %u\r\n", rc_channels_override.chan3_raw);
            fprintf("RC channel 4: %u\r\n", rc_channels_override.chan4_raw);
            printf("===============\r\n");
            // printf("MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE\r\n");
            break;
        }
        default: {
            break;
        }
    }
}