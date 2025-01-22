/*
 * lms7002m.c
 *
 *  Created on: May 7, 2024
 *      Author: lab
 */

#include "lms7002m.h"

void lms_spi_write(uint16_t addr, uint16_t val) {
    uint16_t cmd;
    uint16_t dat;

    /* set cs */
    spimaster_cs_write(1);

    cmd = (1 << 15) | (addr & 0x7fff);
    dat = val & 0xffff;
    while ((spimaster_status_read() & 0x1) == 0);
    spimaster_mosi_write(cmd << 16 | dat);
    spimaster_control_write(32 * SPI_LENGTH | SPI_START);
}

uint16_t lms_spi_read(uint16_t addr) {
    uint16_t recv_val;

    /* set cs */
    spimaster_cs_write(1);

    uint16_t cmd;
    cmd = (0 << 15) | (addr & 0x7fff);

    spimaster_mosi_write(cmd << 16);
    spimaster_control_write(32 * SPI_LENGTH | SPI_START);
    while ((spimaster_status_read() & 0x1) == 0);
    recv_val = spimaster_miso_read() & 0xffff;
    return recv_val;
}
