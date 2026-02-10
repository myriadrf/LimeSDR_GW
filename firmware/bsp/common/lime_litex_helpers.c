//
// Created by ts on 7/4/25.
//

#include "lime_litex_helpers.h"

void helper_csr_update_byte(const uint32_t addr, const uint8_t val, const uint8_t byte_index) {
    // If the byte index is invalid, return without doing anything
    // to avoid buffer overflow
    if (byte_index > 3) {
        return;
    };
    uint32_t buf;
    uint8_t* buf_point = (uint8_t*)&buf;
    buf = csr_read_simple(addr);
    buf_point[byte_index] = val;
    csr_write_simple(buf, addr);
}

void helper_csr_read_byte(const uint32_t addr, uint8_t *val, const uint8_t byte_index) {
    // If the byte index is invalid, return without doing anything
    // to avoid buffer overflow
    if (byte_index > 3) {
        return;
    };
    uint32_t buf;
    uint8_t* buf_point = (uint8_t*)&buf;
    buf = csr_read_simple(addr);
    *val = buf_point[byte_index];
}
