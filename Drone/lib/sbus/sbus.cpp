#include "sbus.h"

SBUS::SBUS() {
    status_      = 0;
    payload_idx_ = 0;
    pre_byte_    = SBUS_FOOTER;
    memset((void *)payload_, 0, 23);
}

SBUS::~SBUS() {
}

void SBUS::Parse(const char* buf, uint8_t buf_len) {
    for (uint8_t i = 0; i < buf_len; i++) {
        cur_byte_ = buf[i];
        if (status_ == 0) {
            if ((cur_byte_ == SBUS_HEADER) && (pre_byte_ == SBUS_FOOTER)) {
                status_      = 1;
                payload_idx_ = 0;
            }
        } else if (status_ == 1) {
            uint8_t len_buf_rest     = buf_len - i;
            uint8_t lenpayload__rest = 23 - payload_idx_;
            uint8_t len              = (len_buf_rest < lenpayload__rest) ? len_buf_rest : lenpayload__rest;
            memcpy((void*)(payload_ + payload_idx_), (void*)(buf + i), len);
            i += len;
            payload_idx_ += len;

            if (payload_idx_ == 23) {
                status_ = 2;
            }
        } else {
            if (cur_byte_ == SBUS_FOOTER) {
                status_      = 0;
                payload_idx_ = 0;
            }
        }
        pre_byte_ = cur_byte_;
    }
}

void SBUS::Read(uint16_t* channels, bool* lost_frame, bool* fail_safe) {
    // 16 channels of 11 bit data
    channels[0]  = (uint16_t)((payload_[0] | payload_[1] << 8) & 0x07FF);
    channels[1]  = (uint16_t)((payload_[1] >> 3 | payload_[2] << 5) & 0x07FF);
    channels[2]  = (uint16_t)((payload_[2] >> 6 | payload_[3] << 2 | payload_[4] << 10) & 0x07FF);
    channels[3]  = (uint16_t)((payload_[4] >> 1 | payload_[5] << 7) & 0x07FF);
    channels[4]  = (uint16_t)((payload_[5] >> 4 | payload_[6] << 4) & 0x07FF);
    channels[5]  = (uint16_t)((payload_[6] >> 7 | payload_[7] << 1 | payload_[8] << 9) & 0x07FF);
    channels[6]  = (uint16_t)((payload_[8] >> 2 | payload_[9] << 6) & 0x07FF);
    channels[7]  = (uint16_t)((payload_[9] >> 5 | payload_[10] << 3) & 0x07FF);
    channels[8]  = (uint16_t)((payload_[11] | payload_[12] << 8) & 0x07FF);
    channels[9]  = (uint16_t)((payload_[12] >> 3 | payload_[13] << 5) & 0x07FF);
    channels[10] = (uint16_t)((payload_[13] >> 6 | payload_[14] << 2 | payload_[15] << 10) & 0x07FF);
    channels[11] = (uint16_t)((payload_[15] >> 1 | payload_[16] << 7) & 0x07FF);
    channels[12] = (uint16_t)((payload_[16] >> 4 | payload_[17] << 4) & 0x07FF);
    channels[13] = (uint16_t)((payload_[17] >> 7 | payload_[18] << 1 | payload_[19] << 9) & 0x07FF);
    channels[14] = (uint16_t)((payload_[19] >> 2 | payload_[20] << 6) & 0x07FF);
    channels[15] = (uint16_t)((payload_[20] >> 5 | payload_[21] << 3) & 0x07FF);

    if (lost_frame) {
        *lost_frame = payload_[22] & SBUS_LOSTFRAME;
    }

    if (fail_safe) {
        *fail_safe = payload_[22] & SBUS_FAILSAFE;
    }
}

void SBUS::Write(const uint16_t* channels, char* packet) {
    packet[0] = SBUS_HEADER;
    // 16 channels of 11 bit data
    packet[1]  = (char)((channels[0] & 0x07FF));
    packet[2]  = (char)((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
    packet[3]  = (char)((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
    packet[4]  = (char)((channels[2] & 0x07FF) >> 2);
    packet[5]  = (char)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
    packet[6]  = (char)((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
    packet[7]  = (char)((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
    packet[8]  = (char)((channels[5] & 0x07FF) >> 1);
    packet[9]  = (char)((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
    packet[10] = (char)((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
    packet[11] = (char)((channels[7] & 0x07FF) >> 3);
    packet[12] = (char)((channels[8] & 0x07FF));
    packet[13] = (char)((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
    packet[14] = (char)((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
    packet[15] = (char)((channels[10] & 0x07FF) >> 2);
    packet[16] = (char)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
    packet[17] = (char)((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
    packet[18] = (char)((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
    packet[19] = (char)((channels[13] & 0x07FF) >> 1);
    packet[20] = (char)((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
    packet[21] = (char)((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
    packet[22] = (char)((channels[15] & 0x07FF) >> 3);

    // flags
    packet[23] = 0x00;
    // footer
    packet[24] = SBUS_FOOTER;
}

char* SBUS::GetPayload() {
    return payload_;
}