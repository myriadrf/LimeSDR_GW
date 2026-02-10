//
// Created by lab on 5/8/24.
//

#ifndef AFE7901
#define AFE7901

#include <generated/csr.h>
#include "stdint.h"


#define SPI_CS_HIGH (0 << 0)
#define SPI_CS_LOW  (1 << 0)
#define SPI_START   (1 << 0)
#define SPI_DONE    (1 << 0)
#define SPI_LENGTH  (1 << 8)

void AFE7901_spi_write(uint16_t addr, uint8_t val, uint32_t cs);

uint8_t AFE7901_spi_read(uint16_t addr, uint32_t cs);

void dev_spi_write(uint16_t addr, uint8_t val);

int dev_spi_read(uint16_t addr);

void xil_printf(const char *fmt, ...);

void delaySec(int seconds);

void AFE7901_bringup(void);

#endif //AFE7901
