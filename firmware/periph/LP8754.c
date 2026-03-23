#include "LP8754.h"
#include "bsp.h"
#include "litei2c.h"
#include <stdint.h>
#include <stdio.h> // For debug/logging (optional)

#ifndef BSP_I2C_LP8754_ADDR
#    error "BSP_I2C_LP8754_ADDR is not defined. Please define it before including this file."
#endif

uint32_t LP8754_config[] = {
    // TODO: apply valid config
    // 0xREGADDR[7:0] REGVAL[7:0]
    /*
                0x00BC,
                0x063F,
                0x0791,
                0x0891,
                0x0991,
                0x0A91,
                0x0B91,
                0x0C91,
                0x0D00,
                0x0E00,
                0x0FF8,
                0x1001,
                0x1100,
                0x1200,
                0x1884,
                0x1936,
                0x1F42,
                0x2100,
                0x2200,
                0x2E0B,
                */

    0x00A8,

    // ... add rest of entries
};

#define LP8754_CONFIG_LEN (sizeof(LP8754_config) / sizeof(LP8754_config[0]))

// Stub: Board-specific initialization
bool LP8754_init(litei2c_regs *litei2c_regs)
{
    LP8754_read_registers(litei2c_regs);
    // TODO: uncomment folowing lines when you have valid config
    bool status = LP8754_apply_config(litei2c_regs);
    // bool status = true;
    LP8754_read_registers(litei2c_regs);
    return status;
}

// Function to write a 16-bit register address and 8-bit data
bool LP8754_write_reg(litei2c_regs *litei2c_regs, uint8_t reg_addr, uint8_t value)
{
    return litei2c_a8d8_write_register(litei2c_regs, BSP_I2C_LP8754_ADDR, reg_addr, value) == 0;
}

bool LP8754_apply_config(litei2c_regs *litei2c_regs)
{
    for (size_t i = 0; i < LP8754_CONFIG_LEN; ++i) {
        uint16_t entry = LP8754_config[i];
        uint8_t reg    = (entry >> 8) & 0xFF;
        uint8_t val    = entry & 0xFF;
        if (!LP8754_write_reg(litei2c_regs, reg, val)) {
            return false; // Stop on first error
            printf("[LP8754] Error: cfg fail");
        }
    }
    return true;
}

// Read a single 8-bit register from LP8754
bool LP8754_read_reg(litei2c_regs *litei2c_regs, uint8_t reg_addr, uint8_t *reg_value)
{
    return litei2c_a8d8_read_register(litei2c_regs, BSP_I2C_LP8754_ADDR, reg_addr, reg_value) == 0;
}

// Function to read a 8-bit register address and 8-bit data (usefull for testing only)
void LP8754_read_registers(litei2c_regs *litei2c_regs)
{
    const uint8_t i2c_addr = BSP_I2C_LP8754_ADDR; // LP8754 I2C address (example)

    // Define the register addresses you want to read from
    uint8_t reg_addrs[] = {
        0x00, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
        0x0F, 0x10, 0x11, 0x12, 0x18, 0x19, 0x1F, 0x21, 0x22, 0x2E,
    };
    const int num_regs = sizeof(reg_addrs) / sizeof(reg_addrs[0]);

    // Array to store the read data
    uint8_t reg_values[num_regs];
    printf("[LP8754] Read reg dump:\n");
    // Read from each register
    for (int i = 0; i < num_regs; i++) {
        const int8_t retval = litei2c_a8d8_read_register(litei2c_regs, i2c_addr, reg_addrs[i], &reg_values[i]);
        if (retval == 0) {
            printf("[LP8754] Read reg 0x%04X = 0x%02X\n", reg_addrs[i], reg_values[i]);
        } else {
            printf("[LP8754] Failed to read reg 0x%04X\n", reg_addrs[i]);
        }
    }
}