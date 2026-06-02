#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "regremap.h"

// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-USB
void readCSR(uint8_t *address, uint8_t *regdata_array)
{
    uint16_t value = 0;
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];

    switch (addr) {
    // TODO: Implement register remapping
    default:
        break;
    }

    regdata_array[0] = (value >> 8) & 0xFF;
    regdata_array[1] = value & 0xFF;
}

// To write and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-USB
void writeCSR(uint8_t *address, uint8_t *wrdata_array)
{
    uint16_t value = ((uint16_t)wrdata_array[0] << 8) | wrdata_array[1];
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];

    switch (addr) {
    // TODO: Implement register remapping
    default:
        break;
    }
}
