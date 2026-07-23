#include "spimaster.h"
#include <generated/csr.h>

// cdelay is typically defined in the LiteX bios or board support code
extern void cdelay(int count);

void spimaster_set_mode(const spimaster_regs *regs, uint8_t mode)
{
    // CPOL is fixed at 0 in the LiteX SPIMaster core; only the clock phase (CPHA)
    // is selectable. SPI_MODE0 -> CPHA=0, SPI_MODE1 -> CPHA=1. The phase CSR is
    // provided by the runtime-CPHA gateware patch (tools/spi_cpha_patch.py).
    csr_write_simple((mode & 0x1) ? 1 : 0, regs->phase_addr);
}

uint8_t spimaster_transfer(const spimaster_regs *regs, uint8_t cs, const uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata)
{
    uint32_t recv_val = 0;
    uint32_t bits     = transfer_len * 8;
    uint32_t cs_mask  = 1 << cs;
    uint32_t timeout  = 1000000;

    if (transfer_len == 0 || transfer_len > 4)
        return 1;

    // Pack mosidata MSB-first into a 32-bit register
    uint32_t packed_mosi = 0;
    for (uint32_t i = 0; i < transfer_len; i++) {
        packed_mosi = (packed_mosi << 8) | (uint32_t)mosidata[i];
    }

    // LiteX SPIMaster in 'raw' mode (default) shifts out from the MSB of its data_width.
    // We must left-align our data to the core's width (32 bits).
    packed_mosi <<= (4 - transfer_len) * 8;

    csr_write_simple(cs_mask, regs->cs_addr);
    cdelay(1);

    // Wait for core to be ready
    while (!(csr_read_simple(regs->status_addr) & SPI_DONE) && timeout--) {
    }

    if (timeout == 0) return 1;

    csr_write_simple(packed_mosi, regs->mosi_addr);
    csr_write_simple(bits * SPI_LENGTH | SPI_START, regs->control_addr);

    // Wait for transfer to finish
    timeout = 1000000;
    while (!(csr_read_simple(regs->status_addr) & SPI_DONE) && timeout--) {
    }

    if (timeout == 0) return 1;

    recv_val = csr_read_simple(regs->miso_addr);

    // LiteX SPIMaster captures MISO into the LSBs of the register.
    // If we want 'recv_data_len' bytes, they are in recv_val[recv_data_len*8-1:0].
    if (misodata && recv_data_len > 0) {
        for (int i = recv_data_len - 1; i >= 0; i--) {
            misodata[i] = recv_val & 0xFF;
            recv_val >>= 8;
        }
    }

    return 0;
}
