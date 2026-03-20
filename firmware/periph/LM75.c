//
// Created by tomass on 3/6/26.
//

#include "LM75.h"

/**
 * @brief Reads a 16-bit register from the LM75B.
 * @param I2C_REGS Pointer to I2C controller registers.
 * @param I2C_ADDR_LSB The LSB bits of the I2C address (A2, A1, A0).
 * @param reg_addr The 8-bit register address.
 * @param data Pointer to store the 16-bit result.
 * @return 0 on success, non-zero on error.
 */
int8_t LM75_Read_Register(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB, uint8_t reg_addr, uint16_t *data)
{
    uint8_t full_addr = LM75_I2C_ADDR | (I2C_ADDR_LSB & 0x07);
    return litei2c_a8d16_read_register(I2C_REGS, full_addr, reg_addr, data);
}

/**
 * @brief Writes a 16-bit register to the LM75B.
 * @param I2C_REGS Pointer to I2C controller registers.
 * @param I2C_ADDR_LSB The LSB bits of the I2C address (A2, A1, A0).
 * @param reg_addr The 8-bit register address.
 * @param data The 16-bit value to write.
 * @return 0 on success, non-zero on error.
 */
int8_t LM75_Write_Register(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB, uint8_t reg_addr, uint16_t data)
{
    uint8_t full_addr = LM75_I2C_ADDR | (I2C_ADDR_LSB & 0x07);
    return litei2c_a8d16_write_register(I2C_REGS, full_addr, reg_addr, data);
}

/**
 * @brief Reads the temperature and converts it to an integer (decidegrees).
 * @param I2C_REGS Pointer to I2C controller registers.
 * @param I2C_ADDR_LSB The LSB bits of the I2C address (A2, A1, A0).
 * @return Temperature in 0.1°C units (e.g., 255 = 25.5°C).
 */
int16_t LM75_Read_Temperature(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB)
{
    uint16_t raw_data;
    int16_t converted_value;
    uint8_t spirez;

    // Read 16-bit Temperature Register (Pointer 0x00)
    // litei2c_a8d16_read_register stores:
    // First byte received (MSB) into low byte (raw_data & 0xFF)
    // Second byte received (LSB) into high byte (raw_data >> 8)
    LM75_Read_Register(I2C_REGS, I2C_ADDR_LSB, 0x00, &raw_data);

    uint8_t i2c_rdata_0 = (uint8_t)(raw_data & 0xFF); // MSB
    uint8_t i2c_rdata_1 = (uint8_t)(raw_data >> 8);   // LSB

    // Exact logic from the provided snippet:
    converted_value = (signed short int)i2c_rdata_0;
    converted_value = converted_value << 8;
    converted_value = 10 * (converted_value >> 8);
    spirez          = i2c_rdata_1;
    if (spirez & 0x80)
        converted_value = converted_value + 5;

    return converted_value;
}