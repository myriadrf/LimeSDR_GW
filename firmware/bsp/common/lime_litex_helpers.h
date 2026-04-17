//
// Created by ts on 7/4/25.
//

#ifndef HELPERS_H
#define HELPERS_H

#include <generated/csr.h>
#include <stdint.h>

void helper_csr_update_byte(uint32_t addr, uint8_t val, uint8_t byte_index);

void helper_csr_read_byte(uint32_t addr, uint8_t *val, uint8_t byte_index);

#endif // HELPERS_H