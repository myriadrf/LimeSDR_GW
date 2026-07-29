//
// Created by lab on 5/8/24.
//

#ifndef FIRMWARE_LMS7002M_H
#define FIRMWARE_LMS7002M_H

#include "stdint.h"
#include <generated/csr.h>
#include "spimaster.h"

void lms_spi_write(uint16_t addr, uint16_t val, uint32_t cs);

uint16_t lms_spi_read(uint16_t addr, uint32_t cs);

#endif // FIRMWARE_LMS7002M_H