//
// Created by ts on 6/25/25.
//

#include "TCA6424.h"

void TCA6424_SetPortDirection(const litei2c_regs *i2c_regs, const uint8_t addr, const uint8_t port, const uint8_t dir)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_CONFIG_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_CONFIG_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_CONFIG_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_write_register(i2c_regs, addr, reg_adr, dir);
}

void TCA6424_ReadPortDirection(const litei2c_regs *i2c_regs, const uint8_t addr, const uint8_t port, uint8_t *dir)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_CONFIG_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_CONFIG_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_CONFIG_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_read_register(i2c_regs, addr, reg_adr, dir);
}

void TCA6424_SetPortPolarityInversion(const litei2c_regs *i2c_regs,
                                      const uint8_t addr,
                                      const uint8_t port,
                                      const uint8_t pol)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_POL_INV_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_POL_INV_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_POL_INV_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_write_register(i2c_regs, addr, reg_adr, pol);
}

void TCA6424_ReadPortPolarityInversion(const litei2c_regs *i2c_regs,
                                       const uint8_t addr,
                                       const uint8_t port,
                                       uint8_t *pol)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_POL_INV_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_POL_INV_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_POL_INV_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_read_register(i2c_regs, addr, reg_adr, pol);
}

void TCA6424_SetPortOutputValues(const litei2c_regs *i2c_regs, const uint8_t addr, const uint8_t port, const uint8_t val)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_OUT_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_OUT_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_OUT_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_write_register(i2c_regs, addr, reg_adr, val);
}

void TCA6424_ReadPortOutputValues(const litei2c_regs *i2c_regs, const uint8_t addr, const uint8_t port, uint8_t *val)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_OUT_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_OUT_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_OUT_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_read_register(i2c_regs, addr, reg_adr, val);
}

void TCA6424_ReadPortInputValues(const litei2c_regs *i2c_regs, const uint8_t addr, const uint8_t port, uint8_t *val)
{
    uint8_t reg_adr;
    // Choose a proper address to write to
    // Return without writing if an invalid value is provided
    // To avoid damage due to incorrect values
    switch (port) {
    case 0:
        reg_adr = TCA6424_IN_PORT0_ADR;
        break;
    case 1:
        reg_adr = TCA6424_IN_PORT1_ADR;
        break;
    case 2:
        reg_adr = TCA6424_IN_PORT2_ADR;
        break;
    default:
        return;
    }
    litei2c_a8d8_read_register(i2c_regs, addr, reg_adr, val);
}