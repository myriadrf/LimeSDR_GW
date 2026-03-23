#ifndef SSDR_BSP
#define SSDR_BSP

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
#include <string.h>

/*-----------------------------------------------------------------------*/
/* Project Includes                                                      */
/*-----------------------------------------------------------------------*/
#include "LMS.h"
#include "LMS64C_protocol.h"
#include "lime_litex_helpers.h"
#include <generated/csr.h>

/*-----------------------------------------------------------------------*/
/* Peripheral Includes                                                   */
/*-----------------------------------------------------------------------*/
#include "LP8758.h"
#include "TMP114.h"
#include "Xil_clk_drp.h"
#include "fpga_flash_qspi.h"
#include "litei2c.h"
#include "regremap.h"

/*-----------------------------------------------------------------------*/
/* Constants & Macros                                                    */
/*-----------------------------------------------------------------------*/

/* Device Information */
#define BSP_DEV_TYPE   LMS_DEV_SSDR
#define BSP_HW_VER     2
#define BSP_EXP_BOARD  EXP_BOARD_UNSUPPORTED
#define BSP_FW_VER     11 // New main.c/bsp structure

/* I2C Addresses */
#define BSP_I2C_DAC_ADDR    0x4C
#define BSP_I2C_TEMP_SENSOR_ADDR  0x4E // TMP114NB
#define BSP_I2C_LP8758_ADDR 0x60
#define BSP_I2C_LM75_ADDR   0x48
#define BSP_I2C_EEPROM_ADDR 0x50
#define BSP_EEPROM_DAC_ADDR 0x0010 // Address in EEPROM memory where TCXO DAC value is stored

/* SPI & Peripheral Config */
#define BSP_DAC_INDEX 0
#define BSP_DAC_DEFAULT_VAL  46870 // Default TCXO DAC value loaded when EEPROM is empty

/* LMS Specific IDs */
#define BSP_MAX_ID_LMS7 1

/* Memory Offsets */
// Since there is no eeprom on the board and the flash is too large for the gw
// we use the top of the flash instead of eeprom, thus the offset to last sector
#define BSP_FLASH_STORAGE_OFFSET 0x01FF0000

/* OTP Keys & Addresses */
#define BSP_OTP_UNLOCK_KEY     0x5A
#define BSP_OTP_SERIAL_ADDR 0x0000010
#define BSP_OTP_SERIAL_LEN  0x10

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
uint8_t bsp_program_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload);

/* General SPI Bus Transfers */
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata);

/* Board Serial Number */
uint8_t bsp_serial_read(uint8_t *data_field);
uint8_t bsp_serial_write(const uint8_t *data_field);

/* Peripheral Specific Extensions (Optional) */
uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data);

#endif /* SSDR_BSP */
