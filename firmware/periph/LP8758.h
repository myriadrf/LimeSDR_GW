#ifndef LP8758_H
#define LP8758_H

#include <stdbool.h>
#include <stdint.h>

#define REG_FLAGS_0 0x0D

#define FLAGS_0_nPG_B0_BIT (1U << 2)

// Initialize board-specific hardware
bool LP8758_init(litei2c_regs *litei2c_regs);

bool LP8758_write_reg(litei2c_regs *litei2c_regs, uint8_t reg_addr, uint8_t value);

bool LP8758_apply_config(litei2c_regs *litei2c_regs);

bool LP8758_read_reg(litei2c_regs *litei2c_regs, uint8_t reg_addr, uint8_t *reg_value);

void LP8758_read_registers(litei2c_regs *litei2c_regs);

#endif // LP8758_H