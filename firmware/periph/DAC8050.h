//
// Created by ts on 7/2/25.
//

#ifndef DAC8050_H
#define DAC8050_H

#include "litei2c.h"

//addresses
#define DAC8050_ADDR_A0_AGND 0x48
#define DAC8050_ADDR_A0_VDD 0x49
#define DAC8050_ADDR_A0_SDA 0x4A
#define DAC8050_ADDR_A0_SCL 0x4B
//commands
#define DAC8050_NOOP 0x0
#define DAC8050_DEVID 0x1
#define DAC8050_SYNC 0x2
#define DAC8050_CONFIG 0x3
#define DAC8050_GAIN 0x4
#define DAC8050_TRIGGER 0x5
#define DAC8050_STATUS 0x7
#define DAC8050_DAC_DATA 0x8

/**
 * Issues a write command to the DAC8050 via the I2C interface using the specified parameters.
 *
 * @param regs A pointer to the litei2c_regs structure which contains the base addresses
 *             of the I2C controller registers required for communication.
 * @param i2c_addr The 7-bit I2C address of the DAC8050 device.
 * @param cmd The command to be written to on the DAC8050.
 * @param val The 16-bit value to be written to the specified command.
 * @return Returns 0 on success, or a negative value indicating an error during the I2C communication.
 */
int8_t DAC8050_Write_Command(const litei2c_regs* regs, uint8_t i2c_addr, uint8_t cmd, uint16_t val);

/**
 * Reads a 16-bit value from the DAC8050 device by sending a specific command.
 *
 * @param regs Pointer to the litei2c_regs structure containing the I2C hardware registers configuration.
 * @param i2c_addr The 7-bit I2C address of the DAC8050 device.
 * @param cmd The command byte to specify the register or operation to perform on the DAC8050 device.
 * @param val Pointer to a 16-bit variable where the read value will be stored.
 * @return Returns 0 on success, or a negative value indicating an error during the I2C communication.
 */
int8_t DAC8050_Read_Command(const litei2c_regs* regs, uint8_t i2c_addr, uint8_t cmd, uint16_t* val);

/**
 * Writes DAC output value to the DAC8050 device using I2C communication.
 *
 * @param regs Pointer to the litei2c_regs structure containing the I2C hardware registers configuration.
 * @param i2c_addr The 7-bit I2C address of the DAC8050 device.
 * @param val The 16-bit value to write to the DAC8050.
 *
 * @return Returns 0 on success, or a negative value indicating an error during the I2C communication.
 */
int8_t DAC8050_Write_Value(const litei2c_regs* regs, uint8_t i2c_addr, uint16_t val);

/**
 * Reads the current value from the DAC8050 device.
 *
 * Reads DAC output value from the DAC8050 device using I2C communication.
 *
 * @param regs Pointer to the litei2c_regs structure containing the I2C hardware registers configuration.
 * @param i2c_addr The 7-bit I2C address of the DAC8050 device.
 * @param val Pointer to an uint16_t variable where the read value
 *            from the DAC register will be stored.
 * @return Returns 0 on success, or a negative value indicating an error during the I2C communication.
 */
int8_t DAC8050_Read_Value(const litei2c_regs* regs, uint8_t i2c_addr, uint16_t* val);

#endif //DAC8050_H
