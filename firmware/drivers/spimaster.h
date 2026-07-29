#ifndef SPIMASTER_H
#define SPIMASTER_H

#include <stdint.h>

#define SPI_START             (1 << 0)
#define SPI_DONE              (1 << 0)
#define SPI_LENGTH            (1 << 8)

// SPI clock modes (CPOL is fixed at 0 in the LiteX SPIMaster core).
// Selecting the mode requires the runtime-CPHA gateware patch
// (tools/spi_cpha_patch.py), which exposes a "<name>_phase" CSR whose
// address must be provided via spimaster_regs.phase_addr.
#define SPI_MODE0             0   // CPHA=0: MOSI launched on falling edge, MISO sampled on rising edge.
#define SPI_MODE1             1   // CPHA=1: MOSI launched on rising edge, MISO sampled on falling edge.

typedef struct {
    uint32_t control_addr;
    uint32_t status_addr;
    uint32_t mosi_addr;
    uint32_t miso_addr;
    uint32_t cs_addr;
    uint32_t phase_addr;
} spimaster_regs;

uint8_t spimaster_transfer(const spimaster_regs *regs, uint8_t cs, const uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata);

// Select the SPI clock mode (SPI_MODE0 / SPI_MODE1) for the given master by
// writing its clock-phase (CPHA) CSR. Requires regs->phase_addr to be set to
// the master's "<name>_phase" CSR address.
void spimaster_set_mode(const spimaster_regs *regs, uint8_t mode);

#endif // SPIMASTER_H
