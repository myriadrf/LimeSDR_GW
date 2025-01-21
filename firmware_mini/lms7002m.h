//
// Created by lab on 5/8/24.
//

#ifndef FIRMWARE_LMS7002M_H
#define FIRMWARE_LMS7002M_H

#include <generated/csr.h>
#include "stdint.h"


#define SPI_CS_HIGH (0 << 0)
#define SPI_CS_LOW  (1 << 0)
#define SPI_START   (1 << 0)
#define SPI_DONE    (1 << 0)
#define SPI_LENGTH  (1 << 8)

void lms_spi_write(uint16_t addr, uint16_t val);

uint16_t lms_spi_read(uint16_t addr);

#endif //FIRMWARE_LMS7002M_H
