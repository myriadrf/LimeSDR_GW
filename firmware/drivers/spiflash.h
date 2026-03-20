// This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
// License: BSD

#ifndef __LITESPI_FLASH_H
#define __LITESPI_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_FLASH_BLOCK_SIZE 256
#define CRC32_ERASED_FLASH	 0xFEA8A821

uint32_t spiflash_read_id_register(uint8_t *buf);
uint32_t spiflash_read_status_register(void);

bool spiflash_erase(uint32_t addr);
bool spiflash_page_program(uint32_t addr, uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* __LITESPI_FLASH_H */
