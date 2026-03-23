//
// Created by ts on 1/21/26.
//

#ifndef ADF4002
#define ADF4002

#include "stdint.h"

void Control_TCXO_ADF(uint8_t spi_master, uint8_t spi_cs, uint8_t oe, uint8_t *data);

void adf_pack(uint8_t *in_buf, uint8_t *out_buf);

void Control_TCXO_ADF_packed(uint8_t spi_master, uint8_t spi_cs, uint8_t oe, uint8_t *data);

#endif // ADF4002