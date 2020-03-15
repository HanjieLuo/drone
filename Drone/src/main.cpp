// #include "drone.h"=
#include <Arduino.h>
#include <STM32FreeRTOS.h>


char ptrTaskList[250];

static void MyTask1(void* pvParameters) {
    for( ;; ) {
        Serial.println("Task1 Running");
        // delay(2000);
        vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
    }
}

// static void MyTask2(void* pvParameters) {
//     for( ;; ) {
//         Serial.println("Task2 Running");
//         // delay(2000);
//         vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
//     }
// }

// TaskHandle_t TaskHandle_1;

void setup() {
    Serial.begin(9600);
    // Serial4.begin(115200);
    xTaskCreate(MyTask1, "Task1", 120, NULL, 2, NULL);
    // xTaskCreate(MyTask2, "Task2", 120, NULL, 2, NULL);
    // BaseType_t xReturned = xTaskCreate(MyTask1, "Task1", 120, NULL, 1, NULL);
    // if( xReturned != pdPASS )
    // {
    //     Serial.println("ok");
    // }
    vTaskStartScheduler();
    while(1);
}

void loop() {
    // vTaskList(ptrTaskList);
    // Serial.println("**********************************");
    // Serial.println("Task  State   Prio    Stack    Num");
    // Serial.println("**********************************");
    // Serial.print(ptrTaskList);
    // Serial.println("**********************************");
    // delay(1000);
}

// #define PIN_SERIAL4_RX PA1
// #define PIN_SERIAL4_TX PA0

// //                      RX    TX
// HardwareSerial Serial4(PA1, PA0);

// String inputString  = "";     // a String to hold incoming data
// bool stringComplete = false;  // whether the string is complete
// inputString.reserve(200);

// void serialEvent4() {
//     while (Serial4.available()) {
//         // get the new byte:
//         char inChar = (char)Serial4.read();
//         // add it to the inputString:
//         inputString += inChar;
//         // if the incoming character is a newline, set a flag so the main loop can
//         // do something about it:
//         if (inChar == '\n') {
//             stringComplete = true;
//         }
//     }
// }

// void loop() {
// while (Serial4.available()) {
//     char inChar = (char)Serial4.read();
//     Serial.println(inChar);
// }

// if (stringComplete) {
//     Serial.println(inputString);
//     // clear the string:
//     inputString    = "";
//     stringComplete = false;
// }
// Serial4.println("OK");
// Serial.println("wait");
// delay(100);
// Serial.println(configMAX_SYSCALL_INTERRUPT_PRIORITY);
// delay(1000);
// }
