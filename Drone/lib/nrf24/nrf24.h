#ifndef __NRF24_H__
#define __NRF24_H__

#include <Arduino.h>
#include "RF24.h"
#include "nRF24L01.h"

#include "sbus.h"

#define NRF24_CE PB0
#define NRF24_CSN PB1
#define NRF24_IRQ PC1
#define NRF24_DST_ADDR 0xD2F0F0F0F0LL    //Destination address(USB Adapter) {0xF0, 0xF0, 0xF0, 0xF0, 0xD2}
#define NRF24_LOCAL_ADDR 0xE1F0F0F0ABLL  //Set arduino address {0xAB, 0xF0, 0xF0, 0xF0, 0xE1}
#define NRF24_DATA_RATE RF24_250KBPS
#define NRF24_PA_LEVEL RF24_PA_MAX
#define NRF24_CRC_LENGTH RF24_CRC_16
#define NRF24_CHANNEL 111

class NRF24 {
   public:
    NRF24();
    ~NRF24();

    void Init(bool enable_irq = true);
    bool Write(const char* buf, uint8_t buf_len);
    bool Read(char*& buf, uint8_t& buf_len);
    void IrqReceive();

    void PrintDetails();
   private:
    RF24* radio_;
    char packet_[32];

    // SBUS sbus_;
};

#endif  // __NRF24_H__