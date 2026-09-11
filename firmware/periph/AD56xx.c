//
// Created by ts on 7/21/26.
//

#include "AD56xx.h"
#include "bsp.h"
#include <stddef.h>

/**
 * @brief Writes to the AD5601/11/21/62 DAC, setting both value and power mode.
 * @param spi_master SPI master index.
 * @param spi_cs Chip select index.
 * @param value DAC value (8, 10, 12, or 16-bit).
 * @param model DAC model (AD56XX_MODEL_AD5601, AD56XX_MODEL_AD5611, AD56XX_MODEL_AD5621, AD56XX_MODEL_AD5662).
 * @param mode Power mode selection.
 */
void ad56xx_write(uint8_t spi_master, uint8_t spi_cs, uint16_t value, uint8_t model, AD56XX_PowerMode mode)
{
    if (model == AD56XX_MODEL_AD5662) {
        // The AD5662 word format (24-bit):
        // [23:18] : Don't care bits (0)
        // [17:16] : PD1, PD0 (Power-down modes)
        // [15:0]  : 16-bit Data (D15:D0) in normal mode, or don't care in power-down
        uint8_t data[3];
        data[0] = (uint8_t)mode & 0x03;
        if (mode == AD56XX_PWR_NORMAL) {
            data[1] = (value >> 8) & 0xFF;
            data[2] = value & 0xFF;
        } else {
            data[1] = 0;
            data[2] = 0;
        }
        bsp_spi_transfer(spi_master, spi_cs, data, 3, 0, NULL);
    } else {
        uint16_t spi_word = 0;
        uint8_t data[2];

        // The AD5601/11/21 family word format (16-bit):
        // [15:14] : PD1, PD0 (Power-down modes)
        // [13:0]  : Data (Left-aligned) and don't care bits

        // Power-down bits [15:14]
        spi_word = ((uint16_t)mode & 0x03) << 14;

        // If normal mode, add data bits [13:0] (Left-aligned)
        if (mode == AD56XX_PWR_NORMAL) {
            if (model == AD56XX_MODEL_AD5601) { // 8-bit
                spi_word |= (value & 0xFF) << 6;
            } else if (model == AD56XX_MODEL_AD5611) { // 10-bit
                spi_word |= (value & 0x3FF) << 4;
            } else if (model == AD56XX_MODEL_AD5621) { // 12-bit
                spi_word |= (value & 0xFFF) << 2;
            }
        }

        data[0] = (spi_word >> 8) & 0xFF;
        data[1] = spi_word & 0xFF;

        bsp_spi_transfer(spi_master, spi_cs, data, 2, 0, NULL);
    }
}
