#ifndef SPIMASTER_H
#define SPIMASTER_H

#include <stdint.h>

#define SPI_START             (1 << 0)
#define SPI_DONE              (1 << 0)
#define SPI_LENGTH            (1 << 8)

typedef struct {
    uint32_t control_addr;
    uint32_t status_addr;
    uint32_t mosi_addr;
    uint32_t miso_addr;
    uint32_t cs_addr;
} spimaster_regs;

uint8_t spimaster_transfer(const spimaster_regs *regs, uint8_t cs, const uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata);

#endif // SPIMASTER_H
