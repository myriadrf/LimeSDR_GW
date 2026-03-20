//
// Created by ts on 2/24/26.
//

#ifndef FIRMWARE_DAC5311_H
#define FIRMWARE_DAC5311_H

#include <generated/csr.h>
#include "stdint.h"

#define DAC_MODEL_5311 5
#define DAC_MODEL_6311 6
#define DAC_MODEL_7311 7

void dacx311_spi_write(uint16_t write_data, uint32_t spi_cs);
uint8_t dacx311_write_value(uint16_t value, uint32_t spi_cs, uint8_t dac_model);

#endif //FIRMWARE_DAC5311_H