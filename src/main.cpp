#include <Arduino.h>
#include "nrf24.h"
#include "sbus.h"

#define LED_PIN PC13  // 最小系统板板上LED连接的是PB0

NRF24 nrf24;
// char buf[32];
char *buf;
uint8_t buf_len;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);

    nrf24.Init();
    nrf24.PrintDetails();
}

void loop() {

}

// #define NRF24_IRQ PC1

// void IrqReceiveHandler(void) {  // define global handler
//     Serial.println("Get");
// }

// void setup() {
//     Serial.begin(9600);

//     pinMode(NRF24_IRQ, INPUT_PULLDOWN);
//     attachInterrupt(digitalPinToInterrupt(NRF24_IRQ), IrqReceiveHandler, FALLING);  // Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
// }

// void loop() {
//     // Serial.println("LOOP");
//     // delay(1000);
// }




// #define LED_PIN PC13  // 最小系统板板上LED连接的是PB0

// NRF24 nrf24;
// // char buf[32];
// char *buf;
// uint8_t buf_len;

// void setup() {
//     pinMode(LED_PIN, OUTPUT);
//     Serial.begin(9600);

//     nrf24.Init();
//     nrf24.PrintDetails();
// }

// void loop() {
//     // bool flag = nrf24.Read(buf, buf_len);
//     // if (flag) {
//     //     Serial.print("[");
//     //     Serial.print(buf_len);
//     //     Serial.print("]");
//     //     Serial.println(buf);
//     // }
//     // }else{
//     //     char const msg[] = "PC\n";
//     //     nrf24.Write(&msg, sizeof(msg));
//     //     digitalWrite(LED_PIN, HIGH);
//     //     delay(1000);
//     //     digitalWrite(LED_PIN, LOW);
//     // }
//     // delay(1000);
// }






// #include <SPI.h>
// #include "RF24.h"
// #include "nRF24L01.h"

// #define LED_PIN PC13  // 最小系统板板上LED连接的是PB0

// //create an RF24 object
// RF24 radio(PB0, PB1);  // CE, CSN

// // const byte address[6] = "00001";
// // const byte address[6] = "1Node";
// // const uint64_t address = 0xFFFFFFFFFFLL;   // Radio pipe addresses for the 2 nodes to communicate.
// uint8_t TARGET_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  //Destination address(USB Adapter)
// uint8_t LOCAL_ADDRESS[]  = {0xF0, 0xF0, 0xF0, 0xF0, 0xE1};  //Set arduino address

// void setup() {
//     pinMode(LED_PIN, OUTPUT);
//     Serial.begin(9600);

//     radio.begin();  // Setup and configure rf radio
//     // radio.openWritingPipe(address);
//     radio.openWritingPipe(TARGET_ADDRESS);
//     radio.openReadingPipe(1, LOCAL_ADDRESS);
//     radio.setChannel(110);
//     radio.setDataRate(RF24_250KBPS);
//     radio.setPALevel(RF24_PA_MAX);    // If you want to save power use "RF24_PA_MIN" but keep in mind that reduces the module's range
//     radio.setCRCLength(RF24_CRC_16);  // Use 8-bit CRC for performance
//     // radio.setAutoAck(0);                     // Ensure autoACK is enabled
//     radio.setRetries(15, 15);  // Optionally, increase the delay between retries & # of retries

//     radio.stopListening();
//     radio.printDetails();   // Dump the configuration of the rf unit for debugging

//     // Serial.println(F("\n\rRF24/examples/Transfer/"));
// }

// void loop() {
//     char packet[32];
//     char const msg[] = "Hello, World!";
//     packet[0]        = sizeof(msg);
//     memcpy(packet + 1, msg, sizeof(msg));
//     // const char text[32] = "11";
//     radio.write(&packet, sizeof(packet));
//     // Serial.print(flag);
//     digitalWrite(LED_PIN, HIGH);
//     delay(1000);
//     digitalWrite(LED_PIN, LOW);
// }

// // pins used for the connection with the sensor
// // the other you need are controlled by the SPI library):
// const int chipSelectPin = PB1;

// #define LED_PIN PC13 // 最小系统板板上LED连接的是PB0

// void setup() {
//     pinMode(LED_PIN, OUTPUT);
//     // start the SPI library:
//     SPI.begin();

//     // initalize the  data ready and chip select pins:
//     pinMode(chipSelectPin, OUTPUT);
// }

// void loop() {
//     pinMode(LED_PIN, OUTPUT);

//     // take the chip select low to select the device:
//     digitalWrite(chipSelectPin, LOW);

//     SPI.transfer(123); //Send register location

//     // take the chip select high to de-select:
//     digitalWrite(chipSelectPin, HIGH);

//     digitalWrite(LED_PIN, LOW);

//     delay(1000);
// }

// #define LED_PIN PC13 // 最小系统板板上LED连接的是PB0
// #define MOTOR_PIN PC8

// void setup() {
//     pinMode(LED_PIN, OUTPUT);
//     Serial.begin(9600);
//     digitalWrite(LED_PIN, LOW);
//     int speed = 35;
//     analogWrite(MOTOR_PIN, speed);
// }

// void loop() {
// digitalWrite(LED_PIN, LOW);
// analogWrite(MOTOR_PIN, 0);
// if (Serial.available()) {
//     char ch = Serial.read();

//     if(isDigit(ch)) {
//         int speed = map(ch, '0', '9', 0, 255);
//         analogWrite(MOTOR_PIN, speed);
//         Serial.println(speed);
//     } else {
//         Serial.println("ERROR");
//     }
// }
// }

// void setup() {
//     // pinMode(LED_PIN, OUTPUT);
//     Serial.begin(9600);
// }

// void loop() {

//     // digitalWrite(LED_PIN, HIGH);
//     // Serial.println("Fuck you");
//     // delay(100);
//     // digitalWrite(LED_PIN, LOW);
//     // Serial.println("Love you");
//     // delay(100 );
// }