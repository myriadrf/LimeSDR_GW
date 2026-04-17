//
// Created by ts on 7/2/25.
//

#include "DAC8050.h"

int8_t DAC8050_Write_Command(const litei2c_regs *regs, const uint8_t i2c_addr, const uint8_t cmd, const uint16_t val)
{
    return litei2c_a8d16_write_register(regs, i2c_addr, cmd, val);
}

int8_t DAC8050_Read_Command(const litei2c_regs *regs, const uint8_t i2c_addr, const uint8_t cmd, uint16_t *val)
{
    return litei2c_a8d16_read_register(regs, i2c_addr, cmd, val);
}

int8_t DAC8050_Write_Value(const litei2c_regs *regs, const uint8_t i2c_addr, const uint16_t val)
{
    return DAC8050_Write_Command(regs, i2c_addr, DAC8050_DAC_DATA, val);
}

int8_t DAC8050_Read_Value(const litei2c_regs *regs, const uint8_t i2c_addr, uint16_t *val)
{
    return DAC8050_Read_Command(regs, i2c_addr, DAC8050_DAC_DATA, val);
}