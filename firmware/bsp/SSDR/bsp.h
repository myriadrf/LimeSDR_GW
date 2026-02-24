#ifndef SSDR_BSP
#define SSDR_BSP

#ifdef Template_BSP
    #error "Header guard 'Template_BSP' must be renamed to a project-specific identifier."
#endif

// BSP includes the associated regremap
#include "regremap.h"

// Required includes
#include <stdbool.h>
#include <stdint.h>
#include <generated/csr.h>
#include "litei2c.h"
#include <stdio.h>  // For debug/logging (optional)
#include "LMS.h"
#include "lime_litex_helpers.h"
#include "LMS64C_protocol.h"

#include "LP8758.h"
#include "Xil_clk_drp.h"
#include "TMP114.h"
#include "fpga_flash_qspi.h"
#include <string.h>
/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#define I2C_DAC_ADDR     0x4C
#define I2C_TERMO_ADDR   0x4E // TMP114NB
#define I2C_LP8758_ADDR  0x60
//#define FW_VER 1 // Initial version
//#define FW_VER 2 // Fix for PLL config. hang when changing from low to high frequency.
//#define FW_VER 3 // Added serial number into GET_INFO cmd
#define FW_VER 5 // Firmware for Litex project

#define BSP_DAC_INDEX    0

//I2C devices

#define   LM75_I2C_ADDR		0x48
#define   I2C_ADDR_EEPROM   0x50

//GET INFO
#define DEV_TYPE			LMS_DEV_SSDR
#define HW_VER				2
#define EXP_BOARD			EXP_BOARD_UNSUPPORTED

#define MAX_ID_LMS7		1

#define OTP_UNLOCK_KEY		0x5A
#define OTP_SERIAL_ADDRESS  0x0000010
#define OTP_SERIAL_LENGTH   0x10

#define DAC_DEFF_VAL			46870			// Default TCXO DAC value loaded when EEPROM is empty

// Initialize board-specific hardware
void bsp_init(void);

// Power up board hardware
void bsp_powerup(void);

// Optional: shutdown or reset board hardware
void bsp_shutdown(void);

// ISR functions
static void bsp_isr(void);
void bsp_isr_init(void);
void bsp_process_irqs(void);

// Optional: board-specific delay or timer
void bsp_delay_ms(unsigned int ms);
// LMS specific functions
int8_t lms_reset(uint8_t periph_id, uint8_t command);
int8_t lms7002m_periph_id_check(uint8_t periph_id);
int8_t lms8001_periph_id_check(uint8_t periph_id);
void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id);
uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id);
void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id);
uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id);

// Analog value read/write command handlers
uint8_t bsp_analog_read(uint8_t channel, uint8_t* unit, uint8_t* value_msb, uint8_t* value_lsb);
uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb);
// GPIO read/write commands
uint8_t bsp_gpio_dir_read(uint8_t* data, uint8_t offset);
uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset);
uint8_t bsp_gpio_read(uint8_t* data, uint8_t offset);
uint8_t bsp_gpio_write(uint8_t data, uint8_t offset);
uint8_t bsp_gpio_get_cached(const uint8_t offset);

// BSP Memory wr/rd commands
void bsp_vctcxo_permanent_dac_read(uint8_t *data);
void bsp_vctcxo_permanent_dac_write(uint8_t *data);
uint8_t bsp_mem_read(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count);
uint8_t bsp_mem_write(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count);
uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion,uint8_t data_cnt,const uint8_t *payload);
uint8_t bsp_program_mode1_to_flash(uint32_t current_portion,uint8_t data_cnt,const uint8_t *payload);
uint8_t bsp_program_mode2_check_support(void);
uint8_t bsp_program_mode2_boot_from_flash(void);
uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion,uint8_t data_cnt,const uint8_t *payload);
uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion,uint8_t data_cnt,const uint8_t *payload);
uint8_t bsp_program_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload);

//General SPI bus functions
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata);

//ADF functions
uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data);

//Misc/device specific functions


#endif
