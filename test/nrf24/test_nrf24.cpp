#include <Arduino.h>
#include <unity.h>
#include "nrf24.h"

#define LED_PIN PC13  // 最小系统板板上LED连接的是PB0

NRF24 nrf24;

void test_write() {
    char const msg[] = "Hello World\n";

    nrf24.Init();
    nrf24.PrintDetails();

    for (int i = 0; i < 10; i++) {
        nrf24.Write(msg, sizeof(msg));
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }
}

void test_read() {
    char *buf;
    uint8_t buf_len;

    nrf24.Init(false);
    nrf24.PrintDetails();

    for (int i = 0; i < 10;) {
        bool flag = nrf24.Read(buf, buf_len);

        if (flag) {
            Serial.print("[");
            Serial.print(buf_len);
            Serial.print("]");
            Serial.println(buf);
            i++;
        }
    }
}

void test_irq_read() {
    nrf24.Init(true);
    nrf24.PrintDetails();

    for (int i = 0; i < 120; i++) {
        delay(1000);
    }
}

void setup() {
    // give `pio test` some time to connect to serial
    delay(2000);
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);

    UNITY_BEGIN();  // IMPORTANT LINE!
    // RUN_TEST(test_write);
    // RUN_TEST(test_read);
    RUN_TEST(test_irq_read);
    UNITY_END();
}

void loop() {
}
