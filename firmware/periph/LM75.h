//
// Created by tomass on 3/6/26.
//

#ifndef LIMESDR_GW_LM75_H
#define LIMESDR_GW_LM75_H

#include "litei2c.h"

#define BSP_I2C_LM75_ADDR 0x48

int8_t LM75_Read_Register(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB, uint8_t reg_addr, uint16_t *data);

int8_t LM75_Write_Register(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB, uint8_t reg_addr, uint16_t data);

int16_t LM75_Read_Temperature(const litei2c_regs *I2C_REGS, uint8_t I2C_ADDR_LSB);

#endif // LIMESDR_GW_LM75_H