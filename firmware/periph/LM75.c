//
// Created by tomass on 3/6/26.
//

#include "LM75.h"

#include <stdio.h>

// LM75 Register addresses
#define LM75_REG_TEMP   0x00
#define LM75_REG_CONF   0x01
#define LM75_REG_THYST  0x02
#define LM75_REG_TOS    0x03

#define LM75_CONF_OS_POL_HIGH   0x04u

/* Note:
 * LM75 threshold registers:
 *   45.0°C = bytes {0x2D, 0x00} -> value 0x002D
 *   55.0°C = bytes {0x37, 0x00} -> value 0x0037
 */
#define LM75_THYST_45C          0x2D00u
#define LM75_TOS_55C            0x3700u

static uint8_t lm75_get_addr(uint8_t I2C_ADDR_LSB)
{
    return BSP_I2C_LM75_ADDR | (I2C_ADDR_LSB & 0x07);
}

static int8_t LM75_Read_Register8(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB, uint8_t reg_addr, uint8_t *data)
{
    return litei2c_a8d8_read_register(I2C_REGS, lm75_get_addr(I2C_ADDR_LSB), reg_addr, data);
}

static int8_t LM75_Write_Register8(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB, uint8_t reg_addr, uint8_t data)
{
    return litei2c_a8d8_write_register(I2C_REGS, lm75_get_addr(I2C_ADDR_LSB), reg_addr, data);
}

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
    uint8_t full_addr = BSP_I2C_LM75_ADDR | (I2C_ADDR_LSB & 0x07);
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
    uint8_t full_addr = BSP_I2C_LM75_ADDR | (I2C_ADDR_LSB & 0x07);
    return litei2c_a8d16_write_register(I2C_REGS, full_addr, reg_addr, data);
}

/**
 * @brief Initializes the LM75.
 *
 * Configuration performed:
 *   - CONFIG = 0x04  (OS polarity = 1, comparator mode, normal operation)
 *   - THYST  = 45.0°C
 *   - TOS    = 55.0°C
 *
 * @param I2C_REGS Pointer to I2C controller registers.
 * @param I2C_ADDR_LSB The LSB bits of the I2C address (A2, A1, A0).
 * @return 0 on success, non-zero on error.
 */
int8_t LM75_Init(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB)
{
    int8_t ret;
    uint8_t cfg;
    uint16_t reg16;

    /* CONFIG register */
    ret = LM75_Write_Register8(I2C_REGS, I2C_ADDR_LSB, LM75_REG_CONF, LM75_CONF_OS_POL_HIGH);
    //printf("LM75_Init: write CONF=0x%02x ret=%d\n", LM75_CONF_OS_POL_HIGH, ret);
    if (ret)
        return ret;

    /* THYST register = 45°C */
    ret = LM75_Write_Register(I2C_REGS, I2C_ADDR_LSB, LM75_REG_THYST, LM75_THYST_45C);
    //printf("LM75_Init: write THYST=0x%04x ret=%d\n", LM75_THYST_45C, ret);
    if (ret)
        return ret;

    /* TOS register = 55°C */
    ret = LM75_Write_Register(I2C_REGS, I2C_ADDR_LSB, LM75_REG_TOS, LM75_TOS_55C);
    //printf("LM75_Init: write TOS=0x%04x ret=%d\n", LM75_TOS_55C, ret);
    if (ret)
        return ret;

    //printf("LM75_Init: success\n");
    return 0;
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

    uint8_t i2c_rdata_1 = (uint8_t)(raw_data & 0xFF); // MSB
    uint8_t i2c_rdata_0 = (uint8_t)(raw_data >> 8);   // LSB

    // Exact logic from the provided snippet:
    converted_value = (signed short int)i2c_rdata_0;
    converted_value = converted_value << 8;
    converted_value = 10 * (converted_value >> 8);
    spirez          = i2c_rdata_1;
    if (spirez & 0x80)
        converted_value = converted_value + 5;

    return converted_value;
}
