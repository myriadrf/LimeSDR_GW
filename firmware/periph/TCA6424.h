//
// Created by ts on 6/25/25.
//

#ifndef TCA6424_H
#define TCA6424_H

#include "litei2c.h"

// TCA6424 devices can have one of two addresses
// depending on pin connections
#define TCA6424_ADR0 0x22
#define TCA6424_ADR1 0x23
// Input port register addresses
#define TCA6424_IN_PORT0_ADR 0x0
#define TCA6424_IN_PORT1_ADR 0x1
#define TCA6424_IN_PORT2_ADR 0x2
// Output port register addresses
#define TCA6424_OUT_PORT0_ADR 0x4
#define TCA6424_OUT_PORT1_ADR 0x5
#define TCA6424_OUT_PORT2_ADR 0x6
// Polarity inversion register addresses
// (input only, should not affect outputs)
#define TCA6424_POL_INV_PORT0_ADR 0x8
#define TCA6424_POL_INV_PORT1_ADR 0x9
#define TCA6424_POL_INV_PORT2_ADR 0xA
// Configuration register addresses
#define TCA6424_CONFIG_PORT0_ADR 0xC
#define TCA6424_CONFIG_PORT1_ADR 0xD
#define TCA6424_CONFIG_PORT2_ADR 0xE

/**
 * @brief Set the direction of pins in a specific port on the TCA6424.
 *
 * This function writes to the I2C address specified by 'addr' and sets
 * the direction of the pins in the port, according to the dir value.
 *
 * The dir parameter is interpreted as a bit-array, where each bit corresponds to a specific pin.
 *
 * If an invalid 'port' value is provided, this function will return without
 * writing to the I2C address.
 *
 * @param i2c_regs   Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr       Address of the TCA6424 to write to.
 * @param port       Index of the port (0, 1, or 2).
 * @param dir        Bit pattern for setting the pin direction (high = input, low = output).
 */
void TCA6424_SetPortDirection(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t dir);

/**
 * @brief Read the direction of pins in a specific port on the TCA6424.
 *
 * This function reads from the I2C address specified by 'addr' and retrieves
 * the direction of the pins in the port, according to the dir value.
 *
 * The value stored in 'dir' is interpreted as a bit-array, where each bit corresponds to a specific pin.
 *
 * If an invalid 'port' value is provided, this function will return without
 * reading from the I2C address.
 *
 * @param i2c_regs   Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr       Address of the TCA6424 to read from.
 * @param port       Index of the port (0, 1, or 2).
 * @param dir        Output pointer to store the bit pattern indicating the pin direction.*/

void TCA6424_ReadPortDirection(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t* dir);

/**
 * @brief Sets the polarity inversion for pins in a specific port on the TCA6424.
 *
 * This function writes to the I2C address specified by 'addr' and sets or resets
 * input polarity of the pins in the port, according to the pol value.
 *
 * The pol parameter is interpreted as a bit-array, where each bit corresponds to a specific pin.
 *
 * If an invalid 'port' value is provided, this function will return without
 * writing to the I2C address.
 *
 * @param i2c_regs Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr Address of the TCA6424 to write to.
 * @param port Index of the port (0, 1, or 2).
 * @param pol Bit pattern for input pin polarity inversion.
 */
void TCA6424_SetPortPolarityInversion(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t pol);

/**
 * @brief Read the polarity inversion of pins in a specific port on the TCA6424.
 *
 * This function reads from the I2C address specified by 'addr' and retrieves
 * the polarity inversion status of the pins in the port.
 *
 * The value stored in 'pol' is interpreted as a bit-array, where each bit corresponds to a specific pin.
 *
 * If an invalid 'port' value is provided, this function will return without
 * writing to the I2C address.
 * @param i2c_regs   Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr       Address of the TCA6424 to read from.
 * @param port       Index of the port (0, 1, or 2).
 * @param pol        Output pointer to store the bit pattern indicating the polarity inversion settings.
 */
void TCA6424_ReadPortPolarityInversion(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t* pol);

/**
 * @brief Sets the output value for pins in a specific port on the TCA6424.
 *
 * This function sets the output value for pins in a specific port (port 0, 1, or 2) on the
 * TCA6424 I2C device.
 *
 * The val parameter is interpreted as a bit-array, where each bit corresponds to a specific pin.
 *
 * If an invalid 'port' value is provided, this function will return without
 * writing to the I2C address.
 *
 * @param i2c_regs Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr Address of the TCA6424 to write to.
 * @param port Index of the port (0, 1, or 2).
 * @param val Bit pattern for pin output values.
 *
 * @return None
 */
void TCA6424_SetPortOutputValues(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t val);

/**
 * @brief Read the output values of pins in a specific port on the TCA6424.
 *
 * This function reads from the I2C address specified by 'addr' and retrieves
 * the output values of the pins in the port.
 *
 * The value stored in 'val' is interpreted as a bit-array, where each bit corresponds
 * to a specific pin.
 *
 * If no valid 'port' value is provided, this function will return without
 * reading from the I2C address.
 *
 * @param i2c_regs   Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr       Address of the TCA6424 to read from.
 * @param port       Index of the port (0, 1, or 2).
 * @param val        Bit pattern for retrieving pin output values (high = output, low = input).
 */
void TCA6424_ReadPortOutputValues(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t* val);

/**
 * Reads pin input values from a specific port on the TCA6424.
 *
 * This function reads the input values of pins in a specific port (port 0, 1, or 2) on the
 * TCA6424 I2C device.
 *
 * The value stored in 'val' is interpreted as a bit-array, where each bit corresponds to a specific pin.
 *
 * If an invalid 'port' value is provided, this function will return without
 * writing to the I2C address.
 *
 * @param i2c_regs Pointer to a litei2c_regs structure containing I2C register addresses.
 * @param addr Address of the TCA6424 to read from.
 * @param port Index of the port (0, 1, or 2).
 * @param val Output pointer to store bit pattern corresponding to input values.
 */
void TCA6424_ReadPortInputValues(const litei2c_regs* i2c_regs, uint8_t addr, uint8_t port, uint8_t* val);

#endif //TCA6424_H
