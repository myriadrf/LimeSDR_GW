//
// Created by ts on 6/16/25.
//

#ifndef TMP114_H
#define TMP114_H
#include "litei2c.h"
#include <stdbool.h>
#include <stdint.h>

#define TMP114A_ADDR  0x48
#define TMP114B_ADDR  0x49
#define TMP114C_ADDR  0x4A
#define TMP114D_ADDR  0x4B
#define TMP114ND_ADDR 0x4C
#define TMP114NC_ADDR 0x4D
#define TMP114NB_ADDR 0x4E
#define TMP114NA_ADDR 0x4F

/**
 * @brief Configures the TMP114 sensor for continuous conversion mode.
 *
 * This function enables or disables continuous conversion mode for the TMP114 sensor.
 * When enabled, the sensor will continuously measure and update temperature readings.
 *
 * @param i2c_regs Pointer to the litei2c_regs structure containing the registers for I2C communication.
 * @param addr I2C address of the TMP114 sensor.
 * @param enable Boolean flag to enable or disable continuous conversion mode. Set to true to enable, or false to
 * disable.
 */
void TMP114_Continuous_Conversion(const litei2c_regs *i2c_regs, uint8_t addr, bool enable);

/**
 * @brief Reads the temperature from the TMP114 temperature sensor.
 *
 * The function communicates with the TMP114 sensor using the provided
 * I2C registers and the specific device address to retrieve the temperature
 * data. The data returned is in the form of raw 16-bit temperature value.
 *
 * @param i2c_regs Pointer to the Lite I2C register structure used for I2C communication.
 * @param addr The I2C address of the TMP114 sensor.
 * @return uint16_t Raw 16-bit temperature value retrieved from the TMP114 sensor.
 */
uint16_t TMP114_Read_Temp(const litei2c_regs *i2c_regs, uint8_t addr);

/**
 * @brief Reads a single temperature measurement in one-shot mode from a TMP114 sensor.
 *
 * This function interfaces with the TMP114 temperature sensor using the provided
 * I2C register mappings to request and retrieve a single temperature measurement.
 * The sensor will perform the conversion and return the temperature data
 * in the form of a raw 16-bit temperature value.
 *
 * NOTE!!: Triggering a OneShot conversion will DISABLE continuous conversion, and the
 * TMP device will enter 'shutdown' mode
 *
 * @param i2c_regs Pointer to the lite-i2c register structure that provides access
 *                 to the I2C peripheral's memory space.
 * @param addr     7-bit I2C address of the TMP114 sensor.
 *
 * @return A 16-bit unsigned integer representing the measured temperature value
 *         retrieved from the TMP114 sensor. The unit and format of this data
 *         follow the TMP114 datasheet.
 */
uint16_t TMP114_Read_Temp_OneShot(const litei2c_regs *i2c_regs, uint8_t addr);

/**
 * @brief Converts a raw temperature reading from the TMP114 sensor to a temperature value.
 *
 * This function processes the raw temperature data provided by the TMP114
 * sensor and converts it into a usable temperature value in 0.1C.
 *
 * @param raw_temp The raw temperature data obtained from the TMP114 sensor.
 *                  This value is a 16-bit number representing the
 *                  temperature measurement as returned by the sensor.
 *
 * @return A 16-bit temperature value in multiples of 0.1 Celsius,
 * which is the processed and scaled interpretation of the raw sensor data.
 */
uint16_t TMP114_Convert_Temp(uint16_t raw_temp);

#endif // TMP114_H