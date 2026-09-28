//
// Created by ts on 7/21/26.
//

#ifndef FIRMWARE_AD56XX_H
#define FIRMWARE_AD56XX_H

#include "stdint.h"

#define AD56XX_MODEL_AD5601 8
#define AD56XX_MODEL_AD5611 10
#define AD56XX_MODEL_AD5621 12

typedef enum {
    AD56XX_PWR_NORMAL = 0,      // Normal operation
    AD56XX_PWR_1K_TO_GND = 1,   // 1 kOhm to GND
    AD56XX_PWR_100K_TO_GND = 2, // 100 kOhm to GND
    AD56XX_PWR_THREE_STATE = 3  // Three-state
} AD56XX_PowerMode;

/**
 * @brief Writes to the AD5601/11/21 DAC, setting both value and power mode.
 * @param spi_master SPI master index.
 * @param spi_cs Chip select index.
 * @param value DAC value (8, 10, or 12-bit).
 * @param model DAC model (AD56XX_MODEL_AD5601, AD56XX_MODEL_AD5611, AD56XX_MODEL_AD5621).
 * @param mode Power mode selection.
 */
void ad56xx_write(uint8_t spi_master, uint8_t spi_cs, uint16_t value, uint8_t model, AD56XX_PowerMode mode);

#endif // FIRMWARE_AD56XX_H
