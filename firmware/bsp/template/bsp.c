#include "bsp.h"

void bsp_init(void) {
#error "bsp_init not implemented"
}

void bsp_powerup(void) {
#error "bsp_powerup not implemented"
}

void bsp_shutdown(void) {
#error "bsp_shutdown not implemented"
}

static void bsp_isr(void) {
#error "bsp_isr not implemented"
}

void bsp_isr_init(void) {
#error "bsp_isr_init not implemented"
}

void bsp_process_irqs(void) {
#error "bsp_process_irqs not implemented"
}

void bsp_delay_ms(unsigned int ms) {
#error "bsp_delay_ms not implemented"
}

int8_t lms7002m_periph_id_check(uint8_t periph_id) {
#error "lms7002m_periph_id_check not implemented"
}

int8_t lms8001_periph_id_check(uint8_t periph_id) {
#error "lms8001_periph_id_check not implemented"
}

void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
#error "lms7002m_spi_write not implemented"
}

uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id) {
#error "lms7002m_spi_read not implemented"
}

void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
#error "lms8001_spi_write not implemented"
}

uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id) {
#error "lms8001_spi_read not implemented"
}

uint8_t bsp_analog_read(uint8_t channel, uint8_t *unit, uint8_t *value_msb, uint8_t *value_lsb) {
#error "bsp_analog_read not implemented"
}

uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb) {
#error "bsp_analog_write not implemented"
}

uint8_t bsp_gpio_dir_read(uint8_t *data, uint8_t offset) {
#error "bsp_gpio_dir_read not implemented"
}

uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset) {
#error "bsp_gpio_dir_write not implemented"
}

uint8_t bsp_gpio_read(uint8_t *data, uint8_t offset) {
#error "bsp_gpio_read not implemented"
}

uint8_t bsp_gpio_write(uint8_t data, uint8_t offset) {
#error "bsp_gpio_write not implemented"
}

uint8_t bsp_gpio_get_cached(const uint8_t offset) {
#error "bsp_gpio_get_cached not implemented"
}

void bsp_vctcxo_permanent_dac_read(uint8_t *data) {
#error "bsp_vctcxo_permanent_dac_read not implemented"
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data) {
#error "bsp_vctcxo_permanent_dac_write not implemented"
}

uint8_t bsp_mem_read(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count) {
#error "bsp_mem_read not implemented"
}

uint8_t bsp_mem_write(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count) {
#error "bsp_mem_write not implemented"
}

uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
#error "bsp_program_mode0_fpga_sram not implemented"
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
#error "bsp_program_mode1_to_flash not implemented"
}

uint8_t bsp_program_mode2_check_support(void) {
#error "bsp_program_mode2_check_support not implemented"
}

uint8_t bsp_program_mode2_boot_from_flash(void) {
#error "bsp_program_mode2_boot_from_flash not implemented"
}

uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
#error "bsp_program_mode3_golden_to_flash not implemented"
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
#error "bsp_program_mode4_user_to_flash not implemented"
}

uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len,
    uint8_t *misodata) {
#error "bsp_spi_transfer not implemented"
}

void bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data) {
#error "bsp_control_adf not implemented"
}

void bsp_init_adf(void) {
#error "bsp_init_adf not implemented"
}
