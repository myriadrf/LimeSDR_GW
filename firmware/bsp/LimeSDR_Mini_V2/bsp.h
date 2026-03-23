#ifndef Mini_V2_BSP
#define Mini_V2_BSP

/*-----------------------------------------------------------------------*/
/* Template Check                                                        */
/*-----------------------------------------------------------------------*/
#ifdef Template_BSP
#    error "Header guard 'Template_BSP' must be renamed to a project-specific identifier."
#endif

/*-----------------------------------------------------------------------*/
/* System Includes                                                       */
/*-----------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h> // For debug/logging (optional)

/*-----------------------------------------------------------------------*/
/* Project Includes                                                      */
/*-----------------------------------------------------------------------*/
#include "LMS.h"
#include "LMS64C_protocol.h"
#include "lime_litex_helpers.h"
#include <generated/csr.h>
#include <generated/mem.h>

/*-----------------------------------------------------------------------*/
/* Peripheral Includes                                                   */
/*-----------------------------------------------------------------------*/
#include "DACx311.h"
#include "LM75.h"
#include "litei2c.h"
#include "regremap.h"
#include "spiflash.h"

/*-----------------------------------------------------------------------*/
/* Constants & Macros                                                    */
/*-----------------------------------------------------------------------*/

/* Device Information */
#define BSP_DEV_TYPE   LMS_DEV_MINI_V2
#define BSP_HW_VER     0
#define BSP_EXP_BOARD  EXP_BOARD_UNSUPPORTED
#define BSP_FW_VER     11 // New main.c/bsp structure

/* I2C Addresses */
#define BSP_I2C_EEPROM_ADDR 0x50 // 0xA2

/* SPI & Peripheral Config */
#define BSP_SPI_CS_LMS    (1 << 0)
#define BSP_SPI_CS_DAC    (1 << 1)
#define BSP_DAC_INDEX 0
#define BSP_DAC_DEFAULT_VAL  566 // Default TCXO DAC value loaded when EEPROM is empty

/* LMS Specific IDs */
#define BSP_MAX_ID_LMS7 1

/* MCU SPI register offsets */
#define BSP_MCU_CONTROL_REG 0x02
#define BSP_MCU_STATUS_REG  0x03
#define BSP_MCU_FIFO_WR_REG 0x04

/* Programming modes */
#define BSP_PROG_EEPROM 1
#define BSP_PROG_SRAM   2
#define BSP_BOOT_MCU    3

#define BSP_MAX_MCU_RETRIES 30

/* Flash Memory (W25Q128JV, 128M-bit) */
#define BSP_FLASH_STORAGE_OFFSET 0x00400000 // Start address for user space in FLASH memory
#define BSP_FLASH_PAGE_SIZE         0x100      // 256 bytes, SPI Page size to be used for transfers
#define BSP_FLASH_SECTOR_SIZE       0x1000     // 4KB
#define BSP_FLASH_BLOCK_SIZE        0x10000    // 64KB
#define BSP_CFM0_START_ADDR        0x000000
#define BSP_CFM0_END_ADDR          0x13FFFF

/*-----------------------------------------------------------------------*/
/* Function Prototypes                                                   */
/*-----------------------------------------------------------------------*/

/* Board Initialization & Power */
void bsp_init(void);
void bsp_powerup(void);
void bsp_shutdown(void);

/* Interrupt Management */
void bsp_isr_init(void);
void bsp_process_irqs(void);

/* Board Delays */
void bsp_delay_ms(unsigned int ms);

/* LMS Peripheral Checks & SPI Transfers */
int8_t lms_reset(uint8_t periph_id, uint8_t command);
int8_t lms7002m_periph_id_check(uint8_t periph_id);
int8_t lms8001_periph_id_check(uint8_t periph_id);
void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id);
uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id);
void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id);
uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id);

/* Analog Signal Access */
uint8_t bsp_analog_read(uint8_t channel, uint8_t *unit, uint8_t *value_msb, uint8_t *value_lsb);
uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb);

/* GPIO Control */
uint8_t bsp_gpio_dir_read(uint8_t *data, uint8_t offset);
uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset);
uint8_t bsp_gpio_read(uint8_t *data, uint8_t offset);
uint8_t bsp_gpio_write(uint8_t data, uint8_t offset);
uint8_t bsp_gpio_get_cached(const uint8_t offset);

/* Memory & Programming Commands */
void bsp_vctcxo_permanent_dac_read(uint8_t *data);
void bsp_vctcxo_permanent_dac_write(uint8_t *data);
uint8_t bsp_mem_read(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count);
uint8_t bsp_mem_write(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count);
uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload);
uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload);
uint8_t bsp_program_mode2_check_support(void);
uint8_t bsp_program_mode2_boot_from_flash(void);
uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload);
uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload);
uint8_t bsp_lms_mcu_fw_wr(uint8_t prog_mode, uint8_t current_portion, const uint8_t *data);

/* General SPI Bus Transfers */
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata);

/* Board Serial Number */
uint8_t bsp_serial_read(uint8_t *data_field);
uint8_t bsp_serial_write(const uint8_t *data_field);

/* Peripheral Specific Extensions (Optional) */
uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data);
void spiFlash_read(uint32_t rel_addr, uint32_t length, uint8_t *rdata);

#endif /* Mini_V2_BSP */
