#ifndef __SBUS_H__
#define __SBUS_H__

#include <Arduino.h>

/*
The SBUS protocol uses inverted serial logic with a baud rate of 100000, 8 data bits, even parity bit, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:

Byte[0]: SBUS Header, 0x0F
Byte[1-22]: 16 servo channels, 11 bits per servo channel
Byte[23]:
Bit 7: digital channel 17 (0x80)
Bit 6: digital channel 18 (0x40)
Bit 5: frame lost (0x20)
Bit 4: failsafe activated (0x10)
Bit 0 - 3: n/a
Byte[24]: SBUS End Byte, 0x00
A table mapping bytes[1-22] to servo channels is included

Note that lost frame is true if the frame was lost and failsafe is true if the receiver has entered failsafe mode.
Lost frame is typically used as part of a counter to collect lost frames data for evaluating receiver performance. 
*/

#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

class SBUS {
   public:
    SBUS();
    ~SBUS();

    bool Parse(const char* buf, uint8_t buf_len);

    // void SetChannel(unsigned char ch, unsigned int data);
    // char* GetData();

   private:
    uint8_t status_;
    char cur_byte_;
    char pre_byte_;
    char payload_[23];
    uint8_t payload_idx_;
};

#endif  // __SBUS_H__