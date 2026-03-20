/*
* AFE7901.c
 *
 *  Created on: May 7, 2024
 *      Author: lab
 */

#include "AFE7901.h"
#include <stdio.h>
#include <stdarg.h>

void AFE7901_spi_write(uint16_t addr, uint8_t val, uint32_t cs) {
    uint16_t cmd;
    uint8_t dat;

    /* set cs */
    spimaster1_cs_write(cs);
    cdelay(1);

    cmd = (0 << 15) | (addr & 0x7fff);
    dat = val & 0xff;
    while ((spimaster1_status_read() & 0x1) == 0);
    spimaster1_mosi_write(((uint32_t)cmd << 8) | dat);
    spimaster1_control_write(24 * SPI_LENGTH | SPI_START);
}

uint8_t AFE7901_spi_read(uint16_t addr, uint32_t cs) {
    uint8_t recv_val;

    /* set cs */
    spimaster1_cs_write(cs);

    uint16_t cmd;
    cmd = (1 << 15) | (addr & 0x7fff);

    spimaster1_mosi_write((uint32_t)cmd << 8);
    spimaster1_control_write(24 * SPI_LENGTH | SPI_START);
    while ((spimaster1_status_read() & 0x1) == 0);
    recv_val = spimaster1_miso_read() & 0xffff;
    return recv_val;
}


// Wrapper function to adapt AFE7901_spi_write for compatibility with APIs expecting dev_spi_write
void dev_spi_write(uint16_t addr, uint8_t val) {

	AFE7901_spi_write(addr, val, 0x1);

}


// Wrapper function to adapt AFE7901_spi_read for compatibility with APIs expecting dev_spi_read
int dev_spi_read(uint16_t addr) {

	return (int)AFE7901_spi_read(addr, 0x1);

}

void xil_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);  // Uses standard printf to output formatted string
    va_end(args);
}


// Simple calibrated busy-wait
void delaySec(int seconds) {
    // Estimate how many iterations needed per second of delay
    // TODO: Adjust multiplier based on actual CPU cycles per loop iteration
    const uint64_t loops_per_sec = 10000000;  // ≈10M iterations/sec for 100 MHz CPU

    uint64_t count = (uint64_t)(seconds * loops_per_sec);

    volatile uint64_t i;
    for (i = 0; i < count; i++) {
        __asm__ volatile("nop");
    }
}


void AFE7901_bringup(void) {
	int rdVal=0;
	int pollIter=0;
	int pollVal=0;


	//TODO: Not needed, just for testing
	rdVal = dev_spi_read(0x04);
	if ((rdVal&0x78) == 0x78) {
		xil_printf("AFE7901 detected CHIP_ID : 0x%x\n",rdVal);
		rdVal = dev_spi_read(0x06);
		xil_printf("AFE7901 detected CHIP_VER : 0x%x\n",rdVal);

	}
	delaySec(2);
}




