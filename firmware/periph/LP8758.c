#include "bsp.h"
#include "LP8758.h"
#include "litei2c.h"
#include <stdint.h>
#include <stdio.h>  // For debug/logging (optional)

#ifndef I2C_LP8758_ADDR
#error "I2C_LP8758_ADDR is not defined. Please define it before including this file."
#endif

//TODO: replace magic number with defines and control fields
// ==========================================
// 1. REGISTER DEFINITIONS (Enum)
// ==========================================
typedef enum {
    LP8758_REG_DEV_REV          = 0x00,
    LP8758_REG_OTP_REV          = 0x01,
    LP8758_REG_BUCK0_CTRL1      = 0x02,
	LP8758_REG_BUCK0_VOUT       = 0x0A,
} LP8758_Reg_t;

//LP8758_REG_BUCK0_CTRL1 fields
#define LP8758_EN_BUCK0         (1U << 7)
#define LP8758_EN_PIN_CTRL0     (1U << 6)

#define LP8758_BUCK0_CTRL1_DEFFAULT 0xC8


uint32_t LP8758_config[] = {
		//TODO: apply valid config
		//0xREGADDR[7:0] REGVAL[7:0]
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

		0x0ACF, //BUCK0_VOUT=2.4V
		0x0288, //BUCK0_CTRL= EN_BUCK0=1, EN_PIN_CTRL0=0



    // ... add rest of entries
};

#define LP8758_CONFIG_LEN (sizeof(LP8758_config) / sizeof(LP8758_config[0]))

// Stub: Board-specific initialization
bool LP8758_init(litei2c_regs *litei2c_regs) {
    LP8758_read_registers(litei2c_regs);
	//TODO: uncomment folowing lines when you have valid config
    bool status = LP8758_apply_config(litei2c_regs);
	//bool status = true;
    LP8758_read_registers(litei2c_regs);
    return status;
}

// Function to write a 16-bit register address and 8-bit data
bool LP8758_write_reg(litei2c_regs *litei2c_regs, uint8_t reg_addr, uint8_t value) {
	return litei2c_a8d8_write_register(litei2c_regs, I2C_LP8758_ADDR, reg_addr, value) == 0;
}

bool LP8758_apply_config(litei2c_regs *litei2c_regs) {
    for (size_t i = 0; i < LP8758_CONFIG_LEN; ++i) {
        uint16_t entry = LP8758_config[i];
        uint8_t reg = (entry >> 8) & 0xFF;
        uint8_t val = entry & 0xFF;
        if (!LP8758_write_reg(litei2c_regs, reg, val)) {
            return false;  // Stop on first error
            printf("[LP8758] Error: cfg fail");
        }
    }
    return true;
}

// Read a single 8-bit register from LP8758
bool LP8758_read_reg(litei2c_regs *litei2c_regs, uint8_t reg_addr, uint8_t *reg_value) {
	return litei2c_a8d8_read_register(litei2c_regs, I2C_LP8758_ADDR, reg_addr, reg_value) == 0;
}


// Function to read a 8-bit register address and 8-bit data (usefull for testing only)
void LP8758_read_registers(litei2c_regs *litei2c_regs) {
    const uint8_t i2c_addr = I2C_LP8758_ADDR;  // LP8758 I2C address (example)

    // Define the register addresses you want to read from
    uint8_t reg_addrs[] = {
    		0x01,
    		0x02,
    		0x03,
    		0x05,
    		0x07,
    		0x09,
    		0x0A,
                0x0B,
                0x12,
    		0x16,
    		0x17,
    		0x18,
    		0x19,
    		0x1A,
    		0x1B,
    		0x1C,
    		0x1D,
    		0x1E,
    		0x1F,
    		0x20,
    		0x21,
    		0x22,
    		0x23,
    };
    const int num_regs = sizeof(reg_addrs) / sizeof(reg_addrs[0]);

    // Array to store the read data
    uint8_t reg_values[num_regs];
    printf("[LP8758] Read reg dump:\n");
    // Read from each register
    for (int i = 0; i < num_regs; i++) {
    	const int8_t retval = litei2c_a8d8_read_register(litei2c_regs, i2c_addr, reg_addrs[i], &reg_values[i]);
    	if (retval == 0) {
            printf("[LP8758] Read reg 0x%04X = 0x%02X\n", reg_addrs[i], reg_values[i]);
        } else {
            printf("[LP8758] Failed to read reg 0x%04X\n", reg_addrs[i]);
        }
    }
}



