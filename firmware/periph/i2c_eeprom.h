//
// Created by Junie.
//

#ifndef LIMESDR_GW_I2C_EEPROM_H
#define LIMESDR_GW_I2C_EEPROM_H

#include "litei2c.h"
#include <stdint.h>

typedef enum {
    I2C_EEPROM_ADDR_8BIT,
    I2C_EEPROM_ADDR_16BIT
} i2c_eeprom_addr_size_t;

typedef enum {
    I2C_EEPROM_ADDR_MODE_STANDARD,      // Address is entirely in data payload
    I2C_EEPROM_ADDR_MODE_EXTENDED_I2C, // Address bits 10:8 are in I2C dev addr
} i2c_eeprom_addr_mode_t;

typedef struct {
    const litei2c_regs *i2c_regs;
    uint8_t i2c_addr;
    i2c_eeprom_addr_size_t addr_size;
    i2c_eeprom_addr_mode_t addr_mode;
    uint16_t page_size;
    uint32_t total_size;
} i2c_eeprom_t;

int8_t I2C_EEPROM_Init(i2c_eeprom_t *dev, const litei2c_regs *regs, uint8_t addr, i2c_eeprom_addr_size_t asize, i2c_eeprom_addr_mode_t amode, uint16_t psize, uint32_t total_size);

int8_t I2C_EEPROM_Read(const i2c_eeprom_t *dev, uint32_t offset, uint8_t *data, uint32_t len);

int8_t I2C_EEPROM_Write(const i2c_eeprom_t *dev, uint32_t offset, const uint8_t *data, uint32_t len);

int8_t I2C_EEPROM_IsBusy(const i2c_eeprom_t *dev);

#endif // LIMESDR_GW_I2C_EEPROM_H
