//
// Created by ts on 2/24/26.
//

#include "DACx311.h"

static void dacx311_spi_write(const uint16_t write_data, uint32_t spi_cs) {
    uint32_t cmd;

    /* Prepare command (16-bit data shifted left by 16 bits to align with 32-bit register). */
    cmd = ((uint32_t) write_data) << 16;

    /* Wait for master to be ready. */
    while ((spimaster_status_read() & (1 << CSR_SPIMASTER_STATUS_DONE_OFFSET)) == 0);

    /* Set CS */
    spimaster_cs_write(spi_cs);

    /* Do transfer. */
    spimaster_mosi_write(cmd);
    spimaster_control_write(
        (1 << CSR_SPIMASTER_CONTROL_START_OFFSET) |
        (16 << CSR_SPIMASTER_CONTROL_LENGTH_OFFSET)
    );

    /* Wait for transfer to complete. */
    while ((spimaster_status_read() & (1 << CSR_SPIMASTER_STATUS_DONE_OFFSET)) == 0);
}