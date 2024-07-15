/*
 * lms7002m.c
 *
 *  Created on: May 7, 2024
 *      Author: lab
 */

#include "lms7002m.h"

void lms_spi_write(uint16_t addr, uint16_t val)
{
	uint16_t cmd;
	uint16_t dat;
	cmd = (1 << 15) | (addr & 0x7fff);
	dat = val & 0xffff;
	lms_spi_mosi_write(cmd << 16 | dat);
    lms_spi_control_write(32*SPI_LENGTH | SPI_START);


}

uint16_t lms_spi_read(uint16_t addr)
{
	uint16_t recv_val;

    uint16_t cmd;
    cmd = (0 << 15) | (addr & 0x7fff);

    lms_spi_mosi_write(cmd << 16);
    lms_spi_control_write(32*SPI_LENGTH | SPI_START);
    while (lms_spi_status_read() == 0);
    recv_val = lms_spi_miso_read() & 0xffff;
    return recv_val;
}
