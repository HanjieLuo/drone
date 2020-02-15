#include "nrf24.h"

NRF24 *nrf24_instance;

static void IrqReceiveHandler(void) {  // define global handler
    nrf24_instance->IrqReceive();      // calls class member handler
}

NRF24::NRF24() {
    nrf24_instance = this;
    radio_         = new RF24(NRF24_CE, NRF24_CSN);  // CE, CSN
}

NRF24::~NRF24() {
    delete radio_;
}

void NRF24::Init(bool enable_irq) {
    radio_->begin();
    radio_->openWritingPipe(NRF24_DST_ADDR);
    radio_->openReadingPipe(1, NRF24_LOCAL_ADDR);  //設定一對一通訊時，建議使用通道1，而通道0在ESB（增強型突發協定）中，預設當作「接收端傳回收到訊息的回應」之用。
    radio_->setChannel(NRF24_CHANNEL);
    radio_->setDataRate(NRF24_DATA_RATE);
    radio_->setPALevel(NRF24_PA_LEVEL);      // If you want to save power use "RF24_PA_MIN" but keep in mind that reduces the module's range
    radio_->setCRCLength(NRF24_CRC_LENGTH);  // Use 8-bit CRC for performance
    radio_->setRetries(15, 15);              // Optionally, increase the delay between retries & # of retries
    radio_->startListening();
    if (enable_irq) {
        pinMode(NRF24_IRQ, INPUT_PULLDOWN);
        radio_->maskIRQ(1, 1, 0);                                                       // mask all IRQ triggers except for receive (1 is mask, 0 is no mask)
        attachInterrupt(digitalPinToInterrupt(NRF24_IRQ), IrqReceiveHandler, FALLING);  // Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
    }
}

void NRF24::IrqReceive() {
    while (radio_->available()) {  //check receive data
        radio_->read(&packet_, 32);
        Serial.println(packet_ + 1);
    }
}

bool NRF24::Write(const char *buf, uint8_t buf_len) {
    if (buf_len > 31) return false;

    radio_->stopListening();
    packet_[0] = buf_len;
    memcpy(packet_ + 1, buf, buf_len);
    bool flag = radio_->write(&packet_, buf_len + 1);
    radio_->startListening();

    return flag;
}

bool NRF24::Read(char *&buf, uint8_t &buf_len) {
    if (radio_->available()) {  //check receive data
        // memset(packet_, 0, 32);
        radio_->read(&packet_, 32);
        buf_len = packet_[0];
        buf     = packet_ + 1;
        return true;
    }

    return false;
}

void NRF24::PrintDetails() {
    radio_->printDetails();  // Dump the configuration of the rf unit for debugging
}
