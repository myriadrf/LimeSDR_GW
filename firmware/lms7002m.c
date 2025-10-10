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

#ifndef LIMESDR_XTRX
    /* Set CS. */
    spimaster_cs_write(SPI_CS_LMS);
#endif

    /* Prepare command (write bit set) and data. */
    cmd = (1 << 15) | (addr & 0x7fff);
    dat = val & 0xffff;

    /* Wait for master to be ready. */
    while ((spimaster_status_read() & 0x1) == 0);

    /* Do transfer. */
    spimaster_mosi_write(cmd << 16 | dat);
    spimaster_control_write(32 * SPI_LENGTH | SPI_START);

#ifndef LIMESDR_XTRX
    /* Clear CS. */
    spimaster_cs_write(0);
#endif

}

uint16_t lms_spi_read(uint16_t addr) {
    uint16_t recv_val;

#ifndef LIMESDR_XTRX
    /* Set CS. */
    spimaster_cs_write(SPI_CS_LMS);
#endif

    /* Prepare command (read bit clear). */
    uint16_t cmd;
    cmd = (0 << 15) | (addr & 0x7fff);

    /* Do transfer. */
    spimaster_mosi_write(cmd << 16);
    spimaster_control_write(32 * SPI_LENGTH | SPI_START);

    /* Wait for master to complete. */
    while ((spimaster_status_read() & 0x1) == 0);

    /* Read received value. */
    recv_val = spimaster_miso_read() & 0xffff;
#ifndef LIMESDR_XTRX

    /* Clear CS. */
    spimaster_cs_write(0);
#endif
    return recv_val;
}
