//
// Created by ts on 2/24/26.
//

#include "DACx311.h"

void dacx311_spi_write(const uint16_t write_data, const uint32_t spi_cs)
{
    /* Prepare command (16-bit data shifted left by 16 bits to align with the 32-bit register). */
    const uint32_t cmd = ((uint32_t)write_data) << 16;

    /* Wait for the master to be ready. */
    while ((spimaster_status_read() & (1 << CSR_SPIMASTER_STATUS_DONE_OFFSET)) == 0);

    /* Set CS */
    spimaster_cs_write(spi_cs);

    /* Do transfer. */
    spimaster_mosi_write(cmd);
    spimaster_control_write(
        (1 << CSR_SPIMASTER_CONTROL_START_OFFSET) |
        (16 << CSR_SPIMASTER_CONTROL_LENGTH_OFFSET)
    );

    /* Wait for the transfer to complete. */
    while ((spimaster_status_read() & (1 << CSR_SPIMASTER_STATUS_DONE_OFFSET)) == 0);
}

uint8_t dacx311_write_value(const uint16_t value, const uint32_t spi_cs, const uint8_t dac_model)
{
    uint8_t data_width;
    uint16_t data_mask;
    uint16_t spi_val = 0;
    switch (dac_model)
    {
    case DAC_MODEL_5311:
        data_width = 8;
        data_mask = 0xFF;
        break;

    case DAC_MODEL_6311:
        data_width = 10;
        data_mask = 0x3FF;
        break;

    case DAC_MODEL_7311:
        data_width = 12;
        data_mask = 0xFFF;
        break;

    default:
        // Invalid value
        return 1;
    }
    // Mask value
    spi_val = (value & data_mask);
    // Shift up
    spi_val = spi_val << (14 - data_width);
    // Mode bits are already 00 implicitly, no need to set them
    // Write to the device
    dacx311_spi_write(spi_val, spi_cs);
    return 0;
}
