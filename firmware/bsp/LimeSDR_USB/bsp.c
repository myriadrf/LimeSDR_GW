#include "bsp.h"
#include "spimaster.h"

static litei2c_regs I2C_REGS = {.master_active_addr   = CSR_PSS_I2C_MASTER_ACTIVE_ADDR,
                          .master_addr_addr     = CSR_PSS_I2C_MASTER_ADDR_ADDR,
                          .master_settings_addr = CSR_PSS_I2C_MASTER_SETTINGS_ADDR,
                          .master_status_addr   = CSR_PSS_I2C_MASTER_STATUS_ADDR,
                          .master_rxtx_addr     = CSR_PSS_I2C_MASTER_RXTX_ADDR};

static spimaster_regs SPIMASTER_MAIN = {.control_addr = CSR_SPIMASTER_CONTROL_ADDR,
                                 .status_addr  = CSR_SPIMASTER_STATUS_ADDR,
                                 .mosi_addr    = CSR_SPIMASTER_MOSI_ADDR,
                                 .miso_addr    = CSR_SPIMASTER_MISO_ADDR,
                                 .cs_addr      = CSR_SPIMASTER_CS_ADDR};

static spimaster_regs SPIMASTER_FPGA1 = {.control_addr = CSR_FPGA_SPI1_CONTROL_ADDR,
                                  .status_addr  = CSR_FPGA_SPI1_STATUS_ADDR,
                                  .mosi_addr    = CSR_FPGA_SPI1_MOSI_ADDR,
                                  .miso_addr    = CSR_FPGA_SPI1_MISO_ADDR,
                                  .cs_addr      = CSR_FPGA_SPI1_CS_ADDR};

static i2c_eeprom_t I2C_EEPROM_CFG;
static uint8_t dac_val;

void bsp_init(void)
{
    limetop_lms7002_top_lms_ctr_gpio_write(0x0);
    limetop_lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
    // Init pll control register values
    csr_write_simple(0x0FFF, clk_ctrl_addrs.phcfg_samples);
    csr_write_simple(0x0002, clk_ctrl_addrs.phcfg_step);
    csr_write_simple(0x0001, clk_ctrl_addrs.vco_div_cnt);
    csr_write_simple(0x0001, clk_ctrl_addrs.m_odd_div);
    csr_write_simple(0x0001, clk_ctrl_addrs.n_odd_div);
    csr_write_simple(0x0001, clk_ctrl_addrs.c0_odddiv);
    csr_write_simple(0x0001, clk_ctrl_addrs.c1_odddiv);
    csr_write_simple(0x0001, clk_ctrl_addrs.c2_odddiv);
    csr_write_simple(0x0001, clk_ctrl_addrs.c3_odddiv);
    csr_write_simple(0x0001, clk_ctrl_addrs.c4_odddiv);

    // Init Temperature sensor
    LM75_Init(&I2C_REGS,0);
    // Init EEPROM control struct
    I2C_EEPROM_Init(&I2C_EEPROM_CFG,&I2C_REGS,EEPROM_I2C_ADDR,I2C_EEPROM_ADDR_8BIT,I2C_EEPROM_ADDR_MODE_STANDARD,64,16384);

    // Turn off ADF, turn on DAC
    bsp_control_adf(0,NULL,NULL);

    // Read dac EEPROM value
    bsp_vctcxo_permanent_dac_read(&dac_val);
    // If EEPROM DAC value unwritten, use default
    // Note: This also updates the cached value
    if (dac_val == 0xFF)
        dac_val = BSP_DAC_DEFAULT_VAL;
    // Write new value to DAC
    bsp_analog_write(0,0,0,dac_val);



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
    switch (channel) {
    case 0:
        // No read function, return cached values
        *unit      = 0;
        *value_lsb = dac_val;
        *value_msb = 0;
        return STATUS_COMPLETED_CMD;
    case 1:
        uint16_t temp_val = LM75_Read_Temperature(&I2C_REGS, 0x0);
        uint8_t *temp_ptr = (uint8_t *)&temp_val;
        *value_lsb        = temp_ptr[0];
        *value_msb        = temp_ptr[1];
        *unit             = 0x50;

        return STATUS_COMPLETED_CMD;

    default:
        return STATUS_ERROR_CMD;
    }
}

uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb)
{
    switch (channel) {
        case 0:
            // If setting a new value for DAC - disable ADF, enable DAC
            bsp_control_adf(0,NULL,NULL);
            // Update cached value
            dac_val = value_lsb;
            // Write to DAC
            ad56xx_write(BSP_DAC_SPIMASTER, BSP_DAC_CS, dac_val, AD56XX_MODEL_AD5601, AD56XX_PWR_NORMAL);
            return STATUS_COMPLETED_CMD;
        default:
            return STATUS_ERROR_CMD;
    }
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
    uint8_t dac_val;
    I2C_EEPROM_Read(&I2C_EEPROM_CFG,BSP_EEPROM_DAC_ADDR,&dac_val,1);
    *data = dac_val;
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data)
{
    // TODO: Implement VCTCXO permanent DAC write
}

uint8_t bsp_mem_read(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    // Not implemented for LimeSDR USB
    return 0;
}

uint8_t bsp_mem_write(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    // Not implemented for LimeSDR USB
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

uint8_t bsp_spi_transfer(
    uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata)
{
    spimaster_regs *regs;

    switch (master) {
    case 0: // spimaster
            // LMS
        regs = &SPIMASTER_MAIN;
        break;

    case 1: // fpga_spi1
            // DAC / ADF
        regs = &SPIMASTER_FPGA1;
        break;

    default:
        return 1;
    }

    return spimaster_transfer(regs, cs, mosidata, transfer_len, recv_data_len, misodata);
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
    // If turning on ADF - turn off DAC first
    if (oe == 1) {
        ad56xx_write(BSP_DAC_SPIMASTER, BSP_DAC_CS, 0, AD56XX_MODEL_AD5601, AD56XX_PWR_THREE_STATE);
    }
    const uint8_t spi_master = BSP_ADF4002_SPIMASTER;
    const uint8_t spi_cs     = BSP_ADF4002_CS;
    if (pack_data == false) {
        Control_TCXO_ADF(spi_master, spi_cs, oe, (uint8_t *)data);
    } else {
        Control_TCXO_ADF_packed(spi_master, spi_cs, oe, (uint8_t *)data);
    }
    return 0;
}

void bsp_init_adf(void)
{
    // TODO: Implement ADF initialization
}
