#ifndef HiPer_BSP
#define HiPer_BSP

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
#include "ADF4002.h"
#include "AFE7901.h"
#include "DAC8050.h"
#include "LMK05318B.h"
#include "LP8754.h"
#include "LP8758.h"
#include "TCA6424.h"
#include "TMP114.h"
#include "fpga_flash_qspi.h"
#include "hiper_regremap.h"
#include "litei2c.h"

/*-----------------------------------------------------------------------*/
/* Constants & Macros                                                    */
/*-----------------------------------------------------------------------*/

/* Device Information */
#define BSP_DEV_TYPE   LMS_DEV_HIPERSDR_44xx
#define BSP_HW_VER     1
#define BSP_EXP_BOARD  EXP_BOARD_UNSUPPORTED
#define BSP_FW_VER     11 // New main.c/bsp structure

/* I2C Addresses */
#define BSP_I2C_LMK_ADDR 0x65
#define BSP_I2C_LP8754_ADDR    0x60
#define BSP_I2C_LP8758_ADDR    0x60
#define BSP_I2C_LM75_ADDR      0x48
#define BSP_I2C_EEPROM_ADDR    0x50
#define BSP_EEPROM_DAC_ADDR    0x0010 // Address in EEPROM memory where TCXO DAC value is stored

/* SPI & Peripheral Config */
#define BSP_ADF4002_SPIMASTER 2
#define BSP_ADF4002_CS        0
#define BSP_DAC_INDEX     3
#define BSP_DAC_DEFAULT_VAL      46870 // Default TCXO DAC value loaded when EEPROM is empty

/* LMS Specific IDs */
#define BSP_MAX_ID_LMS7 1
#define BSP_MAX_ID_LMS8 5

/* Power Control Bits (CSR_PWR_CONTROL_REG00) */
#define BSP_PWR_EN_2P05_BIT       (1U << CSR_PWR_CONTROL_REG00_PWR_EN_2P05_OFFSET)
#define BSP_PWR_EN_LMK_BIT        (1U << CSR_PWR_CONTROL_REG00_PWR_EN_LMK_OFFSET)
#define BSP_PAFE_EN_D1P0_BIT      (1U << CSR_PWR_CONTROL_REG00_PAFE_EN_D1P0_OFFSET)
#define BSP_PAFE_EN_A1P2_BIT      (1U << CSR_PWR_CONTROL_REG00_PAFE_EN_A1P2_OFFSET)
#define BSP_PAFE_EN_A1P8_BIT      (1U << CSR_PWR_CONTROL_REG00_PAFE_EN_A1P8_OFFSET)
#define BSP_PAFE_EN_A1P8_1_BIT    (1U << CSR_PWR_CONTROL_REG00_PAFE_EN_A1P8_1_OFFSET)
#define BSP_AFE_DCDC_1P0_NRST_BIT (1U << CSR_PWR_CONTROL_REG00_AFE_DCDC_1P0_NRST_OFFSET)

/* Power Control Bits (CSR_PWR_CONTROL_REG01) */
#define BSP_PG_EN_2P05_BIT      (1U << CSR_PWR_CONTROL_REG01_PG_EN_2P05_OFFSET)
#define BSP_PG_AFE_AVDD_1P2_BIT (1U << CSR_PWR_CONTROL_REG01_PG_AFE_AVDD_1P2_OFFSET)

#define BSP_PWR_LMS8_NRST_OFFSET 7
#define BSP_PWR_LMS8_NRST_POS    0
#define BSP_PWR_LMS8_NRST        (1U << 0)

/* TCA6424 I/O expander control signals (U115) */
#define BSP_ENABLE_6VIN       (1U << 0)
#define BSP_ENABLE_7P5VIN     (1U << 1)
#define BSP_REF_EN_GPS        (1U << 2)
#define BSP_REF_EN_OSC        (1U << 3)
#define BSP_PG_8PO            (1U << 4)
#define BSP_PG_6PO            (1U << 5)
#define BSP_ENABLE_5VIN_EXTLO (1U << 6)
#define BSP_ENABLE_5VIN       (1U << 7)

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
void bsp_lmk0518_pwrup_seq(void);
void bsp_afe7901_pwrup_seq(void);
void bsp_lms8_pwrup(void);

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
void afe7901_init(void);
void afe7901_bringup_pre(void);
void afe7901_bringup_post(void);
uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data);
void bsp_init_adf(void);
void bsp_lmk_check_lock();

#endif /* HiPer_BSP */
