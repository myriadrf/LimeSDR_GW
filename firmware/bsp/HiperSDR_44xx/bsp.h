#ifndef HiPer_BSP
#define HiPer_BSP

// BSP includes the associated regremap
#include "hiper_regremap.h"
// Required includes
#include <generated/csr.h>
#include "litei2c.h"
#include "TMP114.h"
#include "TCA6424.h"
#include "LMK05318B.h"
#include "LP8754.h"
#include "LP8758.h"
#include <stdio.h>  // For debug/logging (optional)
#include "LMS.h"
#include "AFE7901.h"
#include "DAC8050.h"
#include "lime_litex_helpers.h"
#include "ADF4002.h"
#include "LMS64C_protocol.h"
/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#define I2C_LMK05318B_ADDR	0x65
#define I2C_LP8754_ADDR		0x60
#define I2C_LP8758_ADDR		0x60

#define MAX_ID_LMS7 0
#define MAX_ID_LMS8 5


#define PWR_EN_2P05_BIT			(1U << CSR_PWR_CONTROL_REG00_PWR_EN_2P05_OFFSET)
#define PWR_EN_LMK_BIT			(1U << CSR_PWR_CONTROL_REG00_PWR_EN_LMK_OFFSET)
#define PAFE_EN_D1P0_BIT		(1U << CSR_PWR_CONTROL_REG00_PAFE_EN_D1P0_OFFSET)
#define PAFE_EN_A1P2_BIT		(1U << CSR_PWR_CONTROL_REG00_PAFE_EN_A1P2_OFFSET)
#define PAFE_EN_A1P8_BIT		(1U << CSR_PWR_CONTROL_REG00_PAFE_EN_A1P8_OFFSET)
#define PAFE_EN_A1P8_1_BIT		(1U << CSR_PWR_CONTROL_REG00_PAFE_EN_A1P8_1_OFFSET)
#define AFE_DCDC_1P0_NRST_BIT 	(1U << CSR_PWR_CONTROL_REG00_AFE_DCDC_1P0_NRST_OFFSET)

#define PG_EN_2P05_BIT		(1U << CSR_PWR_CONTROL_REG01_PG_EN_2P05_OFFSET)
#define PG_AFE_AVDD_1P2_BIT	(1U << CSR_PWR_CONTROL_REG01_PG_AFE_AVDD_1P2_OFFSET)

// TCA6424 I/O expander control signals
// U115
#define ENABLE_6VIN             (1U<<0)
#define ENABLE_7.5VIN           (1U<<1)
#define REF_EN_GPS              (1U<<2)
#define REF_EN_OSC              (1U<<3)
#define PG_8PO                  (1U<<4)
#define PG_6PO                  (1U<<5)
#define ENABLE_5VIN_EXTLO       (1U<<6)
#define ENABLE_5VIN             (1U<<7)

#define PWR_LMS8_NRST           (1U<<0)

#define ADF4002_SPIMASTER 2
#define ADF4002_CS 0

#define BSP_DAC_INDEX 3
// Since there is no eeprom on the board and the flash is too large for the gw
// we use the top of the flash instead of eeprom, thus the offset to last sector
#define mem_write_offset 0x01FF0000



// Initialize board-specific hardware
void bsp_init(void);

// Power up board hardware
void bsp_powerup(void);

// Power up lmk0518 sequence
void bsp_lmk0518_pwrup_seq(void);

// Power up afe7901 sequence
void bsp_afe7901_pwrup_seq(void);

void bsp_lms8_pwrup(void);

// Optional: shutdown or reset board hardware
void bsp_shutdown(void);

// Optional: board-specific delay or timer
void bsp_delay_ms(unsigned int ms);
// LMS specific functions
int8_t lms7002m_periph_id_check(uint8_t periph_id);
int8_t lms8001_periph_id_check(uint8_t periph_id);
void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id);
uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id);
void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id);
uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id);
void afe7901_init(void);
void afe7901_bringup_pre(void);
void afe7901_bringup_post(void);

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

//General SPI bus functions
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata);

//Misc functions
void bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data);
void bsp_init_adf(void);


#endif /* HiPer_BSP */
