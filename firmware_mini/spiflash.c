// This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
// License: BSD

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <generated/csr.h>
#include <generated/mem.h>

#include "spiflash.h"

#if defined(CSR_SPIFLASH_CORE_BASE)

#ifdef CSR_SPIFLASH_CORE_MASTER_CS_ADDR

static void spiflash_len_mask_width_write(uint32_t len, uint32_t width, uint32_t mask)
{
	uint32_t tmp = len & ((1 <<  CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_LEN_SIZE) - 1);
	uint32_t word = tmp << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_LEN_OFFSET;
	tmp = width & ((1 << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_WIDTH_SIZE) - 1);
	word |= tmp << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_WIDTH_OFFSET;
	tmp = mask & ((1 <<  CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_MASK_SIZE) - 1);
	word |= tmp << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_MASK_OFFSET;
	spiflash_core_master_phyconfig_write(word);
}

static bool spiflash_tx_ready(void)
{
	return (spiflash_core_master_status_read() >> CSR_SPIFLASH_CORE_MASTER_STATUS_TX_READY_OFFSET) & 1;
}

static bool spiflash_rx_ready(void)
{
	return (spiflash_core_master_status_read() >> CSR_SPIFLASH_CORE_MASTER_STATUS_RX_READY_OFFSET) & 1;
}

static volatile uint8_t w_buf[SPI_FLASH_BLOCK_SIZE + 4];
static volatile uint8_t r_buf[SPI_FLASH_BLOCK_SIZE + 4];

static uint32_t transfer_byte(uint8_t b)
{
	/* wait for tx ready */
	while (!spiflash_tx_ready());

	spiflash_core_master_rxtx_write((uint32_t)b);

	/* wait for rx ready */
	while (!spiflash_rx_ready());

	return spiflash_core_master_rxtx_read();
}

static void transfer_cmd(volatile uint8_t *bs, volatile uint8_t *resp, int len)
{
	spiflash_len_mask_width_write(8, 1, 1);
	spiflash_core_master_cs_write(1);

	flush_cpu_dcache();
	for (int i=0; i < len; i++) {
		resp[i] = transfer_byte(bs[i]);
	}

	spiflash_core_master_cs_write(0);
	flush_cpu_dcache();
}

uint32_t spiflash_read_status_register(void)
{
	volatile uint8_t buf[4];
	w_buf[0] = 0x05;
	w_buf[1] = 0x00;
	transfer_cmd(w_buf, buf, 4);

	/* FIXME normally the status should be in buf[1],
	   but we have to read it a few more times to be
	   stable for unknown reasons */
	return buf[3];
}

uint32_t spiflash_read_id_register(uint8_t *buf)
{
	w_buf[0] = 0x9F;
	w_buf[1] = 0x00;
	transfer_cmd(w_buf, buf, 6);

	/* FIXME normally the status should be in buf[1],
	   but we have to read it a few more times to be
	   stable for unknown reasons */
	return buf[3];
}

static void spiflash_write_enable(void)
{
	uint8_t buf[1];
	w_buf[0] = 0x06;
	transfer_cmd(w_buf, buf, 1);
}

static void page_program(uint32_t addr, uint8_t *data, int len)
{
	w_buf[0] = 0x02;
	w_buf[1] = addr>>16;
	w_buf[2] = addr>>8;
	w_buf[3] = addr>>0;
	memcpy((void *)w_buf+4, (void *)data, len);
	transfer_cmd(w_buf, r_buf, len+4);
}

static void spiflash_sector_erase(uint32_t addr)
{
	w_buf[0] = 0xd8;
	w_buf[1] = addr>>16;
	w_buf[2] = addr>>8;
	w_buf[3] = addr>>0;
	transfer_cmd(w_buf, r_buf, 4);
}

bool spiflash_erase(uint32_t addr)
{
	/* Check Flash is ready */
	while ((spiflash_read_status_register() & 0x01) != 0);
	/* Enable Write operation */
	spiflash_write_enable();
	/* Wait end of operation */
	while ((spiflash_read_status_register() & 0x01) != 0);
	if ((spiflash_read_status_register() & 0x02) != 0x02)
		return false;

	/* Erase sector */
	spiflash_sector_erase(addr);
	/* Wait end of operation */
	while ((spiflash_read_status_register() & 0x01) != 0);

	return true;
}

bool spiflash_page_program(uint32_t addr, uint8_t *data, int len)
{
	/* Check Flash is ready */
	while ((spiflash_read_status_register() & 0x01) != 0);
	/* Enable Write operation */
	spiflash_write_enable();
	/* Wait end of operation */
	while ((spiflash_read_status_register() & 0x01) != 0);
	if ((spiflash_read_status_register() & 0x02) != 0x02)
		return false;

	/* Write data */
	page_program(addr, data, len);
	/* Don't wait for end operation (done at main.c level) */

	return true;
}

/* erase page size in bytes, check flash datasheet */
#define SPI_FLASH_ERASE_SIZE (64*1024)

#define min(x, y) (((x) < (y)) ? (x) : (y))

#endif

#endif
