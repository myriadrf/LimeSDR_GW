#include "bsp.h"

void bsp_init(void) {
    // RESET FIFO once on power-up
    ft601_fifo_control_write(1);
    ft601_fifo_control_write(0);
    //Reset LMS7
    limetop_lms7002_top_lms_ctr_gpio_write(0x0);
    limetop_lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
}

void bsp_powerup(void) {
// No implementation
}

void bsp_shutdown(void) {
    // No implementation
}

static void bsp_isr(void) {
    // BSP isr controller not implemented in this board's gw
    //TODO: Update this if it gets implemented
}

void bsp_isr_init(void) {
    // BSP isr controller not implemented in this board's gw
    //TODO: Update this if it gets implemented
}

void bsp_process_irqs(void) {
    // BSP isr controller not implemented in this board's gw
    //TODO: Update this if it gets implemented
}

void bsp_delay_ms(unsigned int ms) {
#error "bsp_delay_ms not implemented"
}

int8_t lms_reset(uint8_t periph_id, uint8_t command) {
#error "lms_reset not implemented"
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

/**
 * @brief Transfers data over SPI using the selected SPI master and chip select.
 *
 * This function handles SPI transactions for LiteX SPIMaster cores.
 *
 * @param master        SPI master index
 * @param cs            Chip select line index (converted to bitmask internally).
 * @param mosidata      Pointer to the transmit buffer (MSB-first).
 * @param transfer_len  Total number of bytes to toggle on the SPI bus (1-4).
 * @param recv_data_len Number of bytes to extract from the received MISO data (0-4),
 *                      counting from the end of the transfer (i.e. if recv_data_len = 1,
 *                      the last byte of the MISO data is extracted).
 * @param misodata      Pointer to receive buffer (can be NULL if response is ignored).
 *                      Buffer must be at least 'data_len' bytes.
 *
 * @return 0 on success, 1 on error (invalid master or length).
 */
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata) {

    uint32_t recv_val = 0;
    uint32_t bits = transfer_len * 8;
    uint32_t cs_mask = 1 << cs;

    if (transfer_len == 0 || transfer_len > 4) return 1;

    // Pack mosidata MSB-first into a 32-bit register
    uint32_t packed_mosi = 0;
    for (uint32_t i = 0; i < transfer_len; i++) {
        packed_mosi = (packed_mosi << 8) | mosidata[i];
    }

    // LiteX SPIMaster in 'raw' mode (default) shifts out from the MSB of its data_width.
    // We must left-align our data to the core's width.
    switch (master) {
        case 0: // 32-bit data_width
            packed_mosi <<= (4 - transfer_len) * 8;
            spimaster_cs_write(cs_mask);
            cdelay(1);
            while ((spimaster_status_read() & 0x1) == 0) {}
            spimaster_mosi_write(packed_mosi);
            spimaster_control_write(bits * SPI_LENGTH | SPI_START);
            while ((spimaster_status_read() & 0x1) == 0) {}
            recv_val = spimaster_miso_read();
            break;

        default:
            return 1;
    }

    // LiteX SPIMaster captures MISO into the LSBs of the register.
    // If we want 'recv_data_len' bytes, they are in recv_val[recv_data_len*8-1:0].
    if (misodata && recv_data_len > 0) {
        for (int i = recv_data_len - 1; i >= 0; i--) {
            misodata[i] = recv_val & 0xFF;
            recv_val >>= 8;
        }
    }

    return 0;
}


uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data) {
    // No ADF on this board
    return 1;
}

// Bit swap in byte. LimeSDR Mini V1 specific function.
uint8_t reverse(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
}

uint8_t bsp_program_mode2_check_support(void) {
}

uint8_t bsp_program_mode2_boot_from_flash(void) {
}

uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    return 1;
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    return 1;
}
