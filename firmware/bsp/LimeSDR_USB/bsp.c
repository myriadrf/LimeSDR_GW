#include "bsp.h"

void bsp_init(void)
{
    limetop_lms7002_top_lms_ctr_gpio_write(0x0);
    limetop_lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
    // TODO: Implement board initialization
}

void bsp_powerup(void)
{
    // TODO: Implement power up sequence
}

void bsp_shutdown(void)
{
    // TODO: Implement shutdown sequence
}

static void bsp_isr(void)
{
    // TODO: Implement interrupt service routine
}

void bsp_isr_init(void)
{
    // TODO: Implement interrupt initialization
}

void bsp_process_irqs(void)
{
    // TODO: Process pending interrupts
}

void bsp_delay_ms(unsigned int ms)
{
    // TODO: Check if delay is reasonably accurate

    // Implement platform-specific delay
    // Example: busy wait or use a hardware timer
    while (ms--) {
        // rough CPU delay loop (not accurate)
        for (volatile int i = 0; i < 1000; i++)
            ;
    }
}

int8_t lms_reset(uint8_t periph_id, uint8_t command)
{
    uint8_t check_val = lms7002m_periph_id_check(periph_id);
    if (check_val == 0)
        return 1;
    uint32_t read_value;
    switch (command) {
    case LMS_RST_DEACTIVATE:
        limetop_lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
        return 0;

    case LMS_RST_ACTIVATE:
        limetop_lms7002_top_lms_ctr_gpio_write(0x0);
        return 0;

    case LMS_RST_PULSE:
        limetop_lms7002_top_lms_ctr_gpio_write(0x0);
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        limetop_lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
        return 0;
    }
}

int8_t lms7002m_periph_id_check(uint8_t periph_id)
{
    if (periph_id > BSP_MAX_ID_LMS7) {
        return 0; // Invalid ID
    }
    return 1; // Valid ID
}

int8_t lms8001_periph_id_check(uint8_t periph_id)
{
    // No LMS8's on XTRX board
    return 2; // No LMS8's
}

void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id)
{
    lms_spi_write(addr, val, periph_id);
}

uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id)
{
    return lms_spi_read(addr, periph_id);
}

void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id)
{
    // no LMS8 on XTRX
}

uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id)
{
    return -1;
}

uint8_t bsp_analog_read(uint8_t channel, uint8_t *unit, uint8_t *value_msb, uint8_t *value_lsb)
{
    // TODO: Implement analog read
    return 0;
}

uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb)
{
    // TODO: Implement analog write
    return 0;
}

uint8_t bsp_gpio_dir_read(uint8_t *data, uint8_t offset)
{
    // TODO: Implement GPIO direction read
    return 0;
}

uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset)
{
    // TODO: Implement GPIO direction write
    return 0;
}

uint8_t bsp_gpio_read(uint8_t *data, uint8_t offset)
{
    // TODO: Implement GPIO read
    return 0;
}

uint8_t bsp_gpio_write(uint8_t data, uint8_t offset)
{
    // TODO: Implement GPIO write
    return 0;
}

uint8_t bsp_gpio_get_cached(const uint8_t offset)
{
    // TODO: Implement GPIO get cached
    return 0;
}

void bsp_vctcxo_permanent_dac_read(uint8_t *data)
{
    // TODO: Implement VCTCXO permanent DAC read
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data)
{
    // TODO: Implement VCTCXO permanent DAC write
}

uint8_t bsp_mem_read(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    // TODO: Implement memory read
    return 0;
}

uint8_t bsp_mem_write(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    // TODO: Implement memory write
    return 0;
}

uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // TODO: Implement FPGA SRAM programming
    return 0;
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // TODO: Implement Flash programming
    return 0;
}

uint8_t bsp_program_mode2_check_support(void)
{
    // TODO: Check if boot from flash is supported
    return 0;
}

uint8_t bsp_program_mode2_boot_from_flash(void)
{
    // TODO: Trigger boot from flash
    return 0;
}

uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // TODO: Implement Golden image Flash programming
    return 0;
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // TODO: Implement User image Flash programming
    return 0;
}

uint8_t bsp_lms_mcu_fw_wr(uint8_t prog_mode, uint8_t current_portion, const uint8_t *data)
{
    // TODO: Implement LMS MCU firmware write
    return 0;
}

uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata)
{
    // TODO: Implement general SPI transfer
    return 0;
}

uint8_t bsp_serial_read(uint8_t *data_field)
{
    // TODO: Implement serial number read
    return 0;
}

uint8_t bsp_serial_write(const uint8_t *data_field)
{
    // TODO: Implement serial number write
    return 0;
}

uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data)
{
    // TODO: Implement ADF control
    return 0;
}

void bsp_init_adf(void)
{
    // TODO: Implement ADF initialization
}
