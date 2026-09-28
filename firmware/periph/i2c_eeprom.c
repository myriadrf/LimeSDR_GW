//
// Created by Junie.
//

#include "i2c_eeprom.h"
#include "litei2c.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initializes the I2C EEPROM device structure.
 *
 * @param dev Pointer to the I2C EEPROM device structure to initialize.
 * @param regs Pointer to the LiteI2C controller registers.
 * @param addr The 7-bit I2C address of the EEPROM.
 * @param asize Address size (8-bit or 16-bit).
 * @param amode Addressing mode (Standard or Extended I2C).
 * @param psize Page size in bytes (e.g., 8, 16, 32, 64, 128).
 * @param total_size Total capacity of the EEPROM in bytes.
 * @return 0 on success, -1 on invalid parameters.
 */
int8_t I2C_EEPROM_Init(i2c_eeprom_t *dev, const litei2c_regs *regs, uint8_t addr, i2c_eeprom_addr_size_t asize, i2c_eeprom_addr_mode_t amode, uint16_t psize, uint32_t total_size)
{
    if (dev == NULL || regs == NULL) {
        return -1;
    }
    dev->i2c_regs = regs;
    dev->i2c_addr = addr;
    dev->addr_size = asize;
    dev->addr_mode = amode;
    dev->page_size = psize;
    dev->total_size = total_size;
    return 0;
}

/**
 * @brief Checks if the I2C EEPROM is busy (ACK polling).
 *
 * This function attempts a 0-byte I2C transfer to the device. If the device
 * NACKs the address, it is considered busy (e.g., performing an internal write cycle).
 *
 * @param dev Pointer to the I2C EEPROM device structure.
 * @return 1 if busy, 0 if ready, -1 on other I2C error.
 */
int8_t I2C_EEPROM_IsBusy(const i2c_eeprom_t *dev)
{
    if (dev == NULL) {
        return -1;
    }
    uint32_t buf = 0;
    // Attempt a 0-byte transfer to check for ACK
    int8_t ret = litei2c_transfer(dev->i2c_regs, dev->i2c_addr, &buf, 0, 0);
    if (ret == -1) {
        return 1; // NACK received, device is busy
    }
    return ret; // 0 if ACK received, or other error
}

/**
 * @brief Reads data from the I2C EEPROM.
 *
 * Supports multi-byte reads. The function breaks large requests into smaller chunks
 * compatible with the LiteI2C controller's transfer limits.
 *
 * @param dev Pointer to the I2C EEPROM device structure.
 * @param offset Starting memory address to read from.
 * @param data Pointer to the buffer where read data will be stored.
 * @param len Number of bytes to read.
 * @return 0 on success, -1 on error.
 */
int8_t I2C_EEPROM_Read(const i2c_eeprom_t *dev, uint32_t offset, uint8_t *data, uint32_t len)
{
    if (dev == NULL || data == NULL || (offset + len > dev->total_size)) {
        return -1;
    }

    while (len > 0) {
        uint8_t i2c_addr = dev->i2c_addr;
        uint32_t val = 0;
        uint8_t *ptr = (uint8_t *)&val;
        uint8_t txlen = 0;
        uint32_t current_offset = offset;

        // Handle extended addressing where bits 10:8 of address are in I2C dev addr
        if (dev->addr_mode == I2C_EEPROM_ADDR_MODE_EXTENDED_I2C) {
            i2c_addr |= (uint8_t)((current_offset >> 8) & 0x07);
        }

        // Prepare memory address bytes (MSB first)
        if (dev->addr_size == I2C_EEPROM_ADDR_16BIT) {
            ptr[0] = (uint8_t)((current_offset >> 8) & 0xFF);
            ptr[1] = (uint8_t)(current_offset & 0xFF);
            txlen = 2;
        } else {
            ptr[0] = (uint8_t)(current_offset & 0xFF);
            txlen = 1;
        }

        // LiteI2C is typically limited to 4 bytes per transaction.
        // We use up to 4 bytes for RX.
        uint8_t chunk = (len > 4) ? 4 : (uint8_t)len;

        if (litei2c_transfer_reordered(dev->i2c_regs, i2c_addr, &val, txlen, chunk) != 0) {
            return -1;
        }

        // Extract received data from reordered buffer
        for (uint8_t i = 0; i < chunk; i++) {
            data[i] = ptr[i];
        }

        data += chunk;
        len -= chunk;
        offset += chunk;
    }
    return 0;
}

/**
 * @brief Writes data to the I2C EEPROM.
 *
 * Handles page boundaries and performs ACK polling after each write transaction
 * to ensure the EEPROM has finished its internal write cycle before the next operation.
 *
 * @param dev Pointer to the I2C EEPROM device structure.
 * @param offset Starting memory address to write to.
 * @param data Pointer to the buffer containing data to be written.
 * @param len Number of bytes to write.
 * @return 0 on success, -1 on error.
 */
int8_t I2C_EEPROM_Write(const i2c_eeprom_t *dev, uint32_t offset, const uint8_t *data, uint32_t len)
{
    if (dev == NULL || data == NULL || (offset + len > dev->total_size)) {
        return -1;
    }

    while (len > 0) {
        // Ensure device is ready for a new write cycle
        while (I2C_EEPROM_IsBusy(dev) == 1);

        uint8_t i2c_addr = dev->i2c_addr;
        uint32_t val = 0;
        uint8_t *ptr = (uint8_t *)&val;
        uint8_t txlen = 0;
        uint32_t current_offset = offset;

        // Handle extended addressing
        if (dev->addr_mode == I2C_EEPROM_ADDR_MODE_EXTENDED_I2C) {
            i2c_addr |= (uint8_t)((current_offset >> 8) & 0x07);
        }

        // Prepare memory address bytes
        if (dev->addr_size == I2C_EEPROM_ADDR_16BIT) {
            ptr[0] = (uint8_t)((current_offset >> 8) & 0xFF);
            ptr[1] = (uint8_t)(current_offset & 0xFF);
            txlen = 2;
        } else {
            ptr[0] = (uint8_t)(current_offset & 0xFF);
            txlen = 1;
        }

        // Calculate available space in transaction (LiteI2C 4-byte limit)
        uint8_t max_payload = 4 - txlen;
        uint8_t chunk = (len > max_payload) ? max_payload : (uint8_t)len;

        // Respect EEPROM page boundary
        uint32_t space_in_page = dev->page_size - (current_offset % dev->page_size);
        if (chunk > space_in_page) {
            chunk = (uint8_t)space_in_page;
        }

        if (chunk > 0) {
            // Append data to the buffer after the address
            for (uint8_t i = 0; i < chunk; i++) {
                ptr[txlen + i] = data[i];
            }

            if (litei2c_transfer_reordered(dev->i2c_regs, i2c_addr, &val, txlen + chunk, 0) != 0) {
                return -1;
            }

            data += chunk;
            len -= chunk;
            offset += chunk;
        }
    }

    // Wait for the final write cycle to complete before returning
    while (I2C_EEPROM_IsBusy(dev) == 1);

    return 0;
}
