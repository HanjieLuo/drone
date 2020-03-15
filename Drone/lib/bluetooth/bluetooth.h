#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <Arduino.h>
#include <common/mavlink.h>
// #include <STM32FreeRTOS.h>

#define PIN_SERIAL4_RX PA1
#define PIN_SERIAL4_TX PA0


void mavlink_receive_task(void *params) {
    // mavlink_message_t msg;
    // mavlink_status_t status;
    // int mavlink_comm = MAVLINK_COMM_0;
    const TickType_t delay_10ms = pdMS_TO_TICKS(10);
    Serial.println("In");
    while (1) {
        // while (Serial4.available()) {
        //     uint8_t data = (uint8_t)Serial4.read();
        //     Serial.print(data);
        //     // if (mavlink_parse_char(mavlink_comm, data, &msg, &status)) {
        //     //     switch (msg.msgid) {
        //     //         case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
        //     //             Serial.print("\n");
        //     //             Serial.println(msg.msgid);
        //     //         } break;

        //     //         default: {
        //     //         } break;
        //     //     }
        //     // }
        // }
        Serial.println(configCPU_CLOCK_HZ);
        vTaskDelay(delay_10ms);
    }
}


// void serialEvent4() {
//     mavlink_message_t msg;
//     mavlink_status_t status;
//     int mavlink_comm = MAVLINK_COMM_0;

//     // Serial.println("*");
//     while (Serial4.available()) {
//         // Serial.println("+");
//         uint8_t data = (uint8_t)Serial4.read();
//         // unsigned long time1 = millis();
//         if (mavlink_parse_char(mavlink_comm, data, &msg, &status)) {
//             switch (msg.msgid) {
//                 case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
//                     Serial.println("MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE");
//                     break;
//                 }
//                 default: {
//                     break;
//                 }
//             }
//             // Serial.println("=======");
//             // Serial.println(msg.msgid);
//             // Serial.println(msg.seq);
//             // Serial.println(msg.compid);
//             // Serial.println(msg.sysid);
//             // Serial.println("=======");
//         }
//         // unsigned long time2 = millis();
//         // Serial.println(time2 - time1);
//     }
//     // Serial.print("#");
// }


#endif  // __BLUETOOTH_H__