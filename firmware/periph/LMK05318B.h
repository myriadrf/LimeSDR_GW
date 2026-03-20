#ifndef LMK05318B_H
#define LMK05318B_H

#include <stdbool.h>
#include <stdint.h>

//Register addresses
#define DEV_CTL 				0x0C
#define INT_FLAG0 				0x13
#define INT_FLAG1 				0x14
#define OUT_MUTE				0x19
#define PRODID 					0x02
#define OUTCTL_1 				0x34
#define OUTCTL_2 				0x36
#define OUTCTL_6 				0x3D
#define BAW_LOCKDET_PPM_MAX_BY1 0x50


//READ only values
#define LMK05318B_PRODID 0x35


// Initialize board-specific hardware
bool LMK05318B_init(litei2c_regs *litei2c_regs);

bool LMK05318B_write_reg(litei2c_regs *litei2c_regs, uint16_t reg_addr, uint8_t value);

bool LMK05318B_apply_config(litei2c_regs *litei2c_regs);

void LMK05318B_read_registers(litei2c_regs *litei2c_regs);
bool LMK05318B_read_reg(litei2c_regs* i2c_regs, uint16_t reg_addr, uint8_t *rdata);

bool LMK05318B_check_ID(litei2c_regs *litei2c_regs);

void LMK05318B_check_lock(litei2c_regs *litei2c_reg);

void LMK05318B_dump_registers(litei2c_regs *litei2c_regs);

bool LMK05318B_verify_config(litei2c_regs *litei2c_regs);

void LMK05318B_wait_for_lock(litei2c_regs *i2c_regs);


#endif // LMK05318B_H
