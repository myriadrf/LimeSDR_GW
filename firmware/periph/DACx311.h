//
// Created by ts on 2/24/26.
//

#ifndef FIRMWARE_DAC5311_H
#define FIRMWARE_DAC5311_H

#include <generated/csr.h>
#include "stdint.h"

static void dacx311_spi_write(const uint16_t write_data, uint32_t spi_cs);

#endif //FIRMWARE_DAC5311_H