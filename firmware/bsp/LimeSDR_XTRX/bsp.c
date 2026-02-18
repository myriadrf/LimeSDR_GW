#include "bsp.h"

litei2c_regs I2C0_REGS = {
    .master_active_addr   = CSR_I2C0_MASTER_ACTIVE_ADDR,
    .master_addr_addr     = CSR_I2C0_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C0_MASTER_SETTINGS_ADDR,
    .master_status_addr   = CSR_I2C0_MASTER_STATUS_ADDR,
    .master_rxtx_addr     = CSR_I2C0_MASTER_RXTX_ADDR
};

litei2c_regs I2C1_REGS = {
    .master_active_addr   = CSR_I2C1_MASTER_ACTIVE_ADDR,
    .master_addr_addr     = CSR_I2C1_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C1_MASTER_SETTINGS_ADDR,
    .master_status_addr   = CSR_I2C1_MASTER_STATUS_ADDR,
    .master_rxtx_addr     = CSR_I2C1_MASTER_RXTX_ADDR
};

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
    // No LMS8's on XTRX board
    return 2; // No LMS8's
}

void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
#error "lms7002m_spi_write not implemented"
}

uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id) {
#error "lms7002m_spi_read not implemented"
}

void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
    // no LMS8 on XTRX
}

uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id) {
    return -1;
}

uint8_t bsp_analog_read(uint8_t channel, uint8_t *unit, uint8_t *value_msb, uint8_t *value_lsb) {
    if (channel == 0) {
        uint16_t val = 0;
        uint8_t* val_ptr = (uint8_t*)&val;
        litei2c_a8d16_read_register(&I2C0_REGS, I2C_DAC_ADDR, 0x00, &val);
        *value_lsb = val_ptr[1];
        *value_msb = val_ptr[0];
        return STATUS_COMPLETED_CMD;
    }
    if (channel == 1) {
        uint16_t converted_value;
        uint8_t* val_ptr = (uint8_t*)&converted_value;
        uint8_t buf;
        litei2c_a8d16_read_register(&I2C0_REGS, I2C_DAC_ADDR, 0x30, &converted_value);
        //flip bytes
        buf = val_ptr[0];
        val_ptr[0] = val_ptr[1];
        val_ptr[1] = buf;

        converted_value = converted_value >> 4;
        converted_value = converted_value * 10;
        converted_value = converted_value >> 4;

        *value_lsb = val_ptr[1];
        *value_msb = val_ptr[0];
        return STATUS_COMPLETED_CMD;

    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb) {
    if (channel == 0 && unit == 0) { // TCXO DAC, RAW units
        uint16_t val = 0;
        uint8_t* val_ptr = (uint8_t*)&val;
        val_ptr[0] = value_msb;
        val_ptr[1] = value_lsb;
        litei2c_a8d16_write_register(&I2C0_REGS, I2C_DAC_ADDR, 0x30, val);
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_dir_read(uint8_t *data, uint8_t offset) {
// Unsupported on XTRX
    return 1;
}

uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset) {
    // Unsupported on XTRX
    return 1;
}

uint8_t bsp_gpio_read(uint8_t *data, uint8_t offset) {
    // Unsupported on XTRX
    return 1;
}

uint8_t bsp_gpio_write(uint8_t data, uint8_t offset) {
    // Unsupported on XTRX
    return 1;
}

uint8_t bsp_gpio_get_cached(const uint8_t offset) {
    // Unsupported on XTRX
    return 1;
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

uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len,
    uint8_t *misodata) {
#error "bsp_spi_transfer not implemented"
}

void bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data) {
#error "bsp_control_adf not implemented"
}

void bsp_init_adf(void) {
#error "bsp_init_adf not implemented"
uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data) {
    // No ADF on this board
    return 1;
}
