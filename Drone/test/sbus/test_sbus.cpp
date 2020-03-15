#include <Arduino.h>
#include <unity.h>
#include "sbus.h"

SBUS sbus;

void test_parse() {
    volatile char *payload;
    char data[] = {0x0F, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x00, 0x00};
    sbus.Parse(data, 25);
    payload = sbus.GetPayload();
    for(int i = 0; i < 23; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(",");
    }
    Serial.println("");
    Serial.println("test_Parse");
}

void test_all() {
    uint16_t channels[16] = {512, 2047, 0, 10, 20, 30, 40, 500, 600, 700, 800, 900, 1000, 1100, 1200, 0};
    char packet[25];
    sbus.Write(channels, packet);
    for(int i = 0; i < 25; i++) {
        Serial.print(packet[i], HEX);
        Serial.print(",");
    }
    Serial.println("");

    uint16_t channels2[16];
    sbus.Parse(packet, 25);
    sbus.Read(channels2);
    for(int i = 0; i < 16; i++) {
        Serial.print(channels[i]);
        Serial.print(",");
    }
    Serial.println("");
    for(int i = 0; i < 16; i++) {
        Serial.print(channels2[i]);
        Serial.print(",");
    }
    Serial.println("");
}


void setup() {
    // give `pio test` some time to connect to serial
    delay(2000);
    Serial.begin(9600);
    UNITY_BEGIN();    // IMPORTANT LINE!
    // RUN_TEST(test_parse);
    // RUN_TEST(test_all);
    UNITY_END();
}

void loop() {
}
