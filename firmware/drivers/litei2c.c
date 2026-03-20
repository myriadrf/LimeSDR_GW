//
// Created by ts on 6/17/25.
//

#include "litei2c.h"

#include <generated/csr.h>
#include <stdio.h>

int8_t litei2c_transfer(
    const litei2c_regs *regs, const uint8_t I2C_addr, uint32_t *buf, const uint8_t txlen, const uint8_t rxlen)
{
    // Write active bit
    csr_write_simple(1, regs->master_active_addr);
    // Write I2C address
    csr_write_simple(I2C_addr, regs->master_addr_addr);
    // Write txlen, rxlen and recover = 0
    uint32_t settings = txlen | (rxlen << 8);
    csr_write_simple(settings, regs->master_settings_addr);
    // Wait for master to become ready
    // TODO: Implement timeout? Also reset everything if actually does timeout
    while (!(csr_read_simple(regs->master_status_addr) & (1 << MASTER_STATUS_TX_READY_OFFSET))) {
    }
    // Write buffer to core
    csr_write_simple(*buf, regs->master_rxtx_addr);
    // Wait for master to finish
    // TODO: Implement timeout? Also reset everything if actually does timeout
    while (!(csr_read_simple(regs->master_status_addr) & (1 << MASTER_STATUS_RX_READY_OFFSET))) {
    }
    // Check for NACK
    if (csr_read_simple(regs->master_status_addr) & (1 << MASTER_STATUS_NACK_OFFSET)) {
        return -1;
    }
    // Read buffer from the core
    *buf = csr_read_simple(regs->master_rxtx_addr);
    // Write active bit
    csr_write_simple(0, regs->master_active_addr);
    return 0;
}

int8_t litei2c_transfer_reordered(
    const litei2c_regs *regs, const uint8_t I2C_addr, uint32_t *buf, const uint8_t txlen, const uint8_t rxlen)
{
    uint32_t buf_reordered = 0;
    uint8_t *buf8_pointer  = (uint8_t *)buf;

    switch (txlen) {
    case 4:
        buf_reordered |= buf8_pointer[0] << 24;
        buf_reordered |= buf8_pointer[1] << 16;
        buf_reordered |= buf8_pointer[2] << 8;
        buf_reordered |= buf8_pointer[3];
        break;
    case 3:
        buf_reordered |= buf8_pointer[0] << 16;
        buf_reordered |= buf8_pointer[1] << 8;
        buf_reordered |= buf8_pointer[2];
        break;
    case 2:
        buf_reordered |= buf8_pointer[0] << 8;
        buf_reordered |= buf8_pointer[1];
        break;
    case 1:
        buf_reordered |= buf8_pointer[0];
        break;
    default:
        // Transmits longer than 4 aren't supported
        return -1;
    }

    const int8_t retval = litei2c_transfer(regs, I2C_addr, &buf_reordered, txlen, rxlen);

    switch (rxlen) {
    case 5:
    case 4:
        buf8_pointer[0] = (buf_reordered >> 24) & 0xff;
        buf8_pointer[1] = (buf_reordered >> 16) & 0xff;
        buf8_pointer[2] = (buf_reordered >> 8) & 0xff;
        buf8_pointer[3] = (buf_reordered) & 0xff;
        break;
    case 3:
        buf8_pointer[0] = (buf_reordered) >> 16 & 0xff;
        buf8_pointer[1] = (buf_reordered) >> 8 & 0xff;
        buf8_pointer[2] = (buf_reordered) & 0xff;
        break;
    case 2:
        buf8_pointer[0] = (buf_reordered) >> 8 & 0xff;
        buf8_pointer[1] = (buf_reordered) & 0xff;
        break;
    case 1:
        buf8_pointer[0] = (buf_reordered) & 0xff;
        break;
    default:
        // Transmits longer than 4 aren't supported
        return -1;
    }
    return retval;
}

int8_t litei2c_recover_bus(const litei2c_regs *regs)
{
    csr_write_simple(1, regs->master_active_addr);
    // Set recovery bit
    csr_write_simple(1 << 16, regs->master_settings_addr);
    // Wait for master to become ready
    // TODO: Implement timeout? Also reset everything if actually does timeout
    while (!(csr_read_simple(regs->master_status_addr) & (1 << MASTER_STATUS_TX_READY_OFFSET))) {
    }
    csr_write_simple(0, regs->master_rxtx_addr);
    // Wait for master to finish
    // TODO: Implement timeout? Also reset everything if actually does timeout
    while (!(csr_read_simple(regs->master_status_addr) & (1 << MASTER_STATUS_RX_READY_OFFSET))) {
    }
    (void)csr_read_simple(regs->master_rxtx_addr);
    csr_write_simple(0, regs->master_active_addr);
    return 0;
}

int8_t
litei2c_a8d8_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint8_t reg_addr, uint8_t *reg_val)
{
    uint32_t buf        = reg_addr;
    uint8_t *bufreg     = (uint8_t *)&buf;
    const int8_t retval = litei2c_transfer(regs, I2C_addr, &buf, 1, 1);
    *reg_val            = bufreg[0];
    return retval;
}

int8_t litei2c_a8d8_write_register(const litei2c_regs *regs,
                                   const uint8_t I2C_addr,
                                   const uint8_t reg_addr,
                                   const uint8_t reg_val)
{
    uint32_t buf        = reg_val | (reg_addr << 8);
    const int8_t retval = litei2c_transfer(regs, I2C_addr, &buf, 2, 0);
    return retval;
}

int8_t
litei2c_a16d8_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint16_t reg_addr, uint8_t *reg_val)
{
    uint32_t buf        = 0;
    uint8_t *buf_point  = (uint8_t *)&buf;
    uint8_t *adr_point  = (uint8_t *)&reg_addr;
    buf_point[0]        = adr_point[0];
    buf_point[1]        = adr_point[1];
    const int8_t retval = litei2c_transfer(regs, I2C_addr, &buf, 2, 1);
    *reg_val            = (uint8_t)buf & 0xFF;
    return retval;
}

int8_t litei2c_a16d8_write_register(const litei2c_regs *regs,
                                    const uint8_t I2C_addr,
                                    const uint16_t reg_addr,
                                    const uint8_t reg_val)
{
    uint32_t buf       = 0;
    uint8_t *buf_point = (uint8_t *)&buf;
    uint8_t *adr_point = (uint8_t *)&reg_addr;
    // buf_point[2] will be the first byte to be sent
    // buf_point[0] will be the last byte to be sent
    buf_point[0] = reg_val;
    buf_point[1] = adr_point[0];
    buf_point[2] = adr_point[1];
    return litei2c_transfer(regs, I2C_addr, &buf, 3, 0);
}

int8_t
litei2c_a8d16_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint8_t reg_addr, uint16_t *reg_val)
{
    uint32_t buf        = 0;
    uint8_t *buf_point  = (uint8_t *)&buf;
    uint8_t *val_point  = (uint8_t *)reg_val;
    buf_point[0]        = reg_addr;
    const int8_t retval = litei2c_transfer(regs, I2C_addr, &buf, 1, 2);
    val_point[1]        = buf_point[1];
    val_point[0]        = buf_point[0];
    return retval;
}

int8_t litei2c_a8d16_write_register(const litei2c_regs *regs,
                                    const uint8_t I2C_addr,
                                    const uint8_t reg_addr,
                                    const uint16_t reg_val)
{
    uint32_t buf       = 0;
    uint8_t *buf_point = (uint8_t *)&buf;
    uint8_t *val_point = (uint8_t *)&reg_val;
    buf_point[2]       = reg_addr;
    buf_point[1]       = val_point[1];
    buf_point[0]       = val_point[0];
    return litei2c_transfer(regs, I2C_addr, &buf, 3, 0);
}

int8_t litei2c_a16d16_read_register(const litei2c_regs *regs,
                                    const uint8_t I2C_addr,
                                    const uint16_t reg_addr,
                                    uint16_t *reg_val)
{
    uint32_t buf        = 0;
    uint8_t *buf_point  = (uint8_t *)&buf;
    uint8_t *adr_point  = (uint8_t *)&reg_addr;
    uint8_t *val_point  = (uint8_t *)reg_val;
    buf_point[0]        = adr_point[0];
    buf_point[1]        = adr_point[1];
    const int8_t retval = litei2c_transfer(regs, I2C_addr, &buf, 2, 2);
    val_point[1]        = buf_point[1];
    val_point[0]        = buf_point[0];
    return retval;
}

int8_t litei2c_a16d16_write_register(const litei2c_regs *regs,
                                     const uint8_t I2C_addr,
                                     const uint16_t reg_addr,
                                     const uint16_t reg_val)
{
    uint32_t buf       = 0;
    uint8_t *buf_point = (uint8_t *)&buf;
    uint8_t *adr_point = (uint8_t *)&reg_addr;
    uint8_t *val_point = (uint8_t *)&reg_val;
    buf_point[0]       = val_point[0];
    buf_point[1]       = val_point[1];
    buf_point[2]       = adr_point[0];
    buf_point[3]       = adr_point[1];
    return litei2c_transfer(regs, I2C_addr, &buf, 4, 0);
}
