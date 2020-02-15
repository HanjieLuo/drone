#include "sbus.h"

SBUS::SBUS() {
    status_   = 0;
    payload_idx_ = 0;
    pre_byte_ = SBUS_FOOTER;
}

SBUS::~SBUS() {
}

bool SBUS::Parse(const char* buf, uint8_t buf_len) {
    bool full_packet = false;
    for (uint8_t i = 0; i < buf_len; i++) {
        cur_byte_ = buf[i];
        if (status_ == 0) {
            if ((cur_byte_ == SBUS_HEADER) && (pre_byte_ == SBUS_FOOTER)) {
                status_ = 1;
                payload_idx_ = 0;
            }
        } else if (status_ == 1){
            uint8_t len_buf_rest = buf_len - i;
            uint8_t len_payload_rest = 23 - payload_idx_;
            uint8_t len = (len_buf_rest < len_payload_rest) ? len_buf_rest : len_payload_rest;
            memcpy((void *)(buf + i), (void *)(payload_ + payload_idx_), len);
            i += len;
            payload_idx_ += len;
            
            if (payload_idx_ == 23) {
                status_ = 2;
            }
        } else {
            if (cur_byte_ == SBUS_FOOTER) {
                status_ = 0;
                payload_idx_ = 0;
                full_packet = true;
            }
        }
        pre_byte_ = cur_byte_;
    }

    return full_packet;
}