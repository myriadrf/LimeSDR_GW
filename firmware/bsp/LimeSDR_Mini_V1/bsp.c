#include "bsp.h"

// Static, bsp-wide variables (required by some bsp functions)

/* Persistent state across calls */
static uint8_t last_portion_valid = 0;
static uint8_t last_portion       = 0;

litei2c_regs I2C0_REGS = {.master_active_addr   = CSR_I2C0_MASTER_ACTIVE_ADDR,
                          .master_addr_addr     = CSR_I2C0_MASTER_ADDR_ADDR,
                          .master_settings_addr = CSR_I2C0_MASTER_SETTINGS_ADDR,
                          .master_status_addr   = CSR_I2C0_MASTER_STATUS_ADDR,
                          .master_rxtx_addr     = CSR_I2C0_MASTER_RXTX_ADDR};

void bsp_init(void)
{
    // RESET FIFO once on power-up
    ft601_fifo_control_write(1);
    ft601_fifo_control_write(0);
    // Reset LMS7
    limetop_lms7002_top_lms_ctr_gpio_write(0x0);
    limetop_lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
    {
        // Check if there is a value in permanent vctcxo memory
        // If there is, write it to runtime DAC
        // If there isn't write default
        uint16_t perm_dac_val;
        const uint8_t *perm_dac_ptr = (uint8_t *)&perm_dac_val;
        bsp_vctcxo_permanent_dac_read((uint8_t *)&perm_dac_val);
        if (perm_dac_val != 0xFFFF) {
            bsp_analog_write(BSP_DAC_INDEX, 0x00, perm_dac_ptr[1], perm_dac_ptr[0]);
        } else {
            bsp_analog_write(BSP_DAC_INDEX, 0x00, (BSP_DAC_DEFAULT_VAL & 0xff00 >> 8), BSP_DAC_DEFAULT_VAL & 0xff);
        }
    }
}

void bsp_powerup(void)
{
    // No implementation
}

void bsp_shutdown(void)
{
    // No implementation
}

static void bsp_isr(void)
{
    // BSP isr controller not implemented in this board's gw
    // TODO: Update this if it gets implemented
}

void bsp_isr_init(void)
{
    // BSP isr controller not implemented in this board's gw
    // TODO: Update this if it gets implemented
}

void bsp_process_irqs(void)
{
    // BSP isr controller not implemented in this board's gw
    // TODO: Update this if it gets implemented
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
    // No LMS8's on board
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
    // no LMS8
}

uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id)
{
    return -1;
}

static uint16_t dac_val     = 0;
static uint8_t *dac_val_ptr = (uint8_t *)&dac_val;

uint8_t bsp_analog_read(uint8_t channel, uint8_t *unit, uint8_t *value_msb, uint8_t *value_lsb)
{
    switch (channel) {
    case 0:
        // No read function, return cached values
        *unit      = 0;
        *value_lsb = dac_val_ptr[0];
        *value_msb = dac_val_ptr[1];
        return STATUS_COMPLETED_CMD;
    case 1:
        uint16_t temp_val = LM75_Read_Temperature(&I2C0_REGS, 0x0);
        uint8_t *temp_ptr = (uint8_t *)&temp_val;
        *value_lsb        = temp_ptr[0];
        *value_msb        = temp_ptr[1];
        *unit             = 0x50;

        return STATUS_COMPLETED_CMD;

    default:
        return STATUS_ERROR_CMD;
    }
}

uint8_t bsp_analog_write(const uint8_t channel, const uint8_t unit, const uint8_t value_msb, const uint8_t value_lsb)
{
    // Only channel 0 (DAC) and RAW units are supported for write
    if (channel == 0 && unit == 0) {
        dac_val_ptr[0]       = value_lsb;
        dac_val_ptr[1]       = value_msb;
        const uint8_t retval = dacx311_write_value(dac_val, BSP_SPI_CS_DAC, DAC_MODEL_5311);
        if (retval == 0) {
            return STATUS_COMPLETED_CMD;
        }
        return STATUS_ERROR_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_dir_read(uint8_t *data, uint8_t offset)
{
    // LMS64C GPIO control commands seem unused in software
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset)
{
    // LMS64C GPIO control commands seem unused in software
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_read(uint8_t *data, uint8_t offset)
{
    // LMS64C GPIO control commands seem unused in software
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_write(uint8_t data, uint8_t offset)
{
    // LMS64C GPIO control commands seem unused in software
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_get_cached(const uint8_t offset)
{
    // LMS64C GPIO control commands seem unused in software
    return STATUS_ERROR_CMD;
}

void bsp_vctcxo_permanent_dac_read(uint8_t *data)
{
    litei2c_a16d16_read_register(&I2C0_REGS, BSP_I2C_EEPROM_ADDR, BSP_EEPROM_DAC_ADDR, (uint16_t *)data);
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data)
{
    litei2c_a16d16_write_register(&I2C0_REGS, BSP_I2C_EEPROM_ADDR, BSP_EEPROM_DAC_ADDR, (uint16_t *)*data);
}

uint8_t
bsp_mem_read(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    const uint16_t read_addr = (uint16_t)offset & 0xFFFF;
    if (target == 3) // TARGET = EEPROM
    {
        if (progmode == 0) // Read data from EEPROM #1
        {
            uint8_t retval = 0;
            for (uint8_t i = 0; i < data_count; i++) {
                // Read a byte at a time
                retval |= litei2c_a16d8_read_register(&I2C0_REGS, BSP_I2C_EEPROM_ADDR, read_addr + i, &data[i]);
            }
            if (retval == 0)
                return STATUS_COMPLETED_CMD;
        }
    }
    return STATUS_ERROR_CMD;
}

uint8_t
bsp_mem_write(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    const uint16_t write_addr = (uint16_t)offset & 0xFFFF;
    if (target == 3) // TARGET = EEPROM
    {
        if (progmode == 0) // Write data to EEPROM #1
        {
            uint8_t retval = 0;
            for (uint8_t i = 0; i < data_count; i++) {
                // Write a byte at a time
                retval |= litei2c_a16d8_write_register(&I2C0_REGS, BSP_I2C_EEPROM_ADDR, write_addr + i, data[i]);
            }
            // Return Completed if no errors
            if (retval == 0)
                return STATUS_COMPLETED_CMD;
        }
    }
    return STATUS_ERROR_CMD;
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
uint8_t bsp_spi_transfer(
    uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata)
{
    uint32_t recv_val = 0;
    uint32_t bits     = transfer_len * 8;
    uint32_t cs_mask  = 1 << cs;

    if (transfer_len == 0 || transfer_len > 4)
        return 1;

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
        while ((spimaster_status_read() & 0x1) == 0) {
        }
        spimaster_mosi_write(packed_mosi);
        spimaster_control_write(bits * SPI_LENGTH | SPI_START);
        while ((spimaster_status_read() & 0x1) == 0) {
        }
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

uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data)
{
    // No ADF on this board
    return 1;
}

// Bit swap in byte. LimeSDR Mini V1 specific function.
uint8_t reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    return 1;
    // No implementation
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    static uint8_t state, Flash = 0x0;
    static uint32_t word = 0x0;
    static int address;
    static uint32_t byte = 0;
    uint8_t retval       = 0;

    if (current_portion == 0)
        state = 10;
    if (data_cnt == 0) {
        state = 30;
    }
    Flash = 1;

    while (Flash) {
        switch (state) {
        // Init
        case 10:
            // Set Flash memory addresses
            address = BSP_UFM_START_ADDR;
            // Write Control Register of On-Chip Flash IP to un-protect and erase operation
            // wishbone_write32(ONCHIP_FLASH_0_CSR_BASE + (1<<2), 0xf67fffff);
            // wishbone_write32(ONCHIP_FLASH_0_CSR_BASE + (1<<2), 0xf65fffff);
            internal_flash_control_register_write(0xf67fffff);
            internal_flash_control_register_write(0xf65fffff);

            state = 11;
            Flash = 1;

        case 11:
            // Start erase CFM0
            if ((internal_flash_status_register_read() & 0x13) == 0x10) {
                internal_flash_control_register_write(0xf67fffff);
                state = 13;
                Flash = 1;
            }
            if ((internal_flash_status_register_read() & 0x13) == 0x01) {
                state = 11;
                Flash = 1;
            }
            if ((internal_flash_status_register_read() & 0x13) == 0x00) {
                state = 0;
            }

            break;

        // Initiate UFM (ID1) Erase Operation
        case 13:
            // Write Control Register of On-Chip Flash IP to un-protect and erase operation
            internal_flash_control_register_write(0xf67fffff);
            internal_flash_control_register_write(0xf61fffff);

            state = 14;
            Flash = 1;
            break;

        case 14:
            // Start erase UFM ID1
            if ((internal_flash_status_register_read() & 0x13) == 0x10) {
                internal_flash_control_register_write(0xf67fffff);
                state = 16;
                Flash = 1;
            }
            if ((internal_flash_status_register_read() & 0x13) == 0x01) {
                state = 14;
                Flash = 1;
            }
            if ((internal_flash_status_register_read() & 0x13) == 0x00) {
                state = 0;
            }
            break;

        // Initiate UFM (ID2) Erase Operation
        case 16:

            // Write Control Register of On-Chip Flash IP to un-protect and erase operation
            internal_flash_control_register_write(0xf67fffff);
            internal_flash_control_register_write(0xf62fffff);

            state = 17;
            Flash = 1;
            break;

        case 17:
            // Start erase UFM ID2
            if ((internal_flash_status_register_read() & 0x13) == 0x10) {
                internal_flash_control_register_write(0xf67fffff);
                state = 20;
                Flash = 1;
            }
            if ((internal_flash_status_register_read() & 0x13) == 0x01) {
                state = 17;
                Flash = 1;
            }
            if ((internal_flash_status_register_read() & 0x13) == 0x00) {
                state = 0;
            }
            break;

        // Program
        case 20:
            for (byte = 24; byte <= 52; byte += 4) {
                // Take word and swap bits
                word = ((uint32_t)reverse(payload[byte + 0]) << 24) & 0xFF000000;
                word |= ((uint32_t)reverse(payload[byte + 1]) << 16) & 0x00FF0000;
                word |= ((uint32_t)reverse(payload[byte + 2]) << 8) & 0x0000FF00;
                word |= ((uint32_t)reverse(payload[byte + 3]) << 0) & 0x000000FF;

                // Command to write into On-Chip Flash IP
                if (address <= BSP_CFM0_END_ADDR) {
                    *(uint32_t *)(INTERNAL_FLASH_BASE + (address << 2)) = word;
                    // wishbone_write32(ONCHIP_FLASH_0_DATA_BASE + (address<<2), word);

                    while ((internal_flash_status_register_read() & 0x0b) == 0x02) {
                        // printf("Writing CFM0(%d)\n", address);
                    }

                    if ((internal_flash_status_register_read() & 0x0b) == 0x00) {
                        state   = 0;
                        address = 700000;
                    }

                    if ((internal_flash_status_register_read() & 0x0b) == 0x08) {
                    };

                    // Increment address or move to CFM0 sector
                    if (address == BSP_UFM_END_ADDR)
                        address = BSP_CFM0_START_ADDR;
                    else
                        address += 1;
                } else {
                    retval = 1;
                };
            };

            state  = 20;
            Flash  = 0;
            retval = 0;

            break;

        // Finish
        case 30:
            // Re-protect the sector
            // IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xffffffff);
            internal_flash_control_register_write(0xffffffff);

            state  = 0;
            Flash  = 0;
            retval = 0;

            break;

        default:
            retval = 1;
            state  = 0;
            Flash  = 0;
        };
    };
    return retval;
}

uint8_t bsp_program_mode2_check_support(void)
{
    return 0;
}

uint8_t bsp_program_mode2_boot_from_flash(void)
{
    // Copy-pasted old implementation

    uint32_t reg;
    // set CONFIG_SEL overwrite to 1 and CONFIG_SEL to Image 0
    // wishbone_write32(DUAL_BOOT_0_BASE+(1<<2), 0x00000001);
    reg = 0x00000001;

    // set CONFIG_SEL overwrite to 1 and CONFIG_SEL to Image 1
    // IOWR(DUAL_BOOT_0_BASE, 1, 0x00000003);
    reg = 0x00000003;

    *(uint32_t *)(DUAL_CFG_BASE + (1 << 2)) = reg;

    /*wait while core is busy*/
    while (1) {
        reg = *(uint32_t *)(DUAL_CFG_BASE + (3 << 2));
        cdelay(2000);
        if (reg != 0x01)
            break;
    }

    // Trigger reconfiguration to selected Image
    // wishbone_write32(DUAL_BOOT_0_BASE+(0<<2), 0x00000001);
    reg                                     = 0x00000001;
    *(uint32_t *)(DUAL_CFG_BASE + (0 << 2)) = reg;

    return 0;
}

uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    return 1;
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    return 1;
}

uint8_t bsp_lms_mcu_fw_wr(uint8_t prog_mode, uint8_t current_portion, const uint8_t *data)
{
    uint16_t addr;
    uint16_t val;
    uint8_t MCU_retries;
    uint8_t cmd_errors = 0;

    /* Check portion ordering */
    if (current_portion != 0) {
        if (!last_portion_valid || last_portion != (current_portion - 1)) {
            return STATUS_WRONG_ORDER_CMD;
        }
    } else {
        /* First portion resets tracking */
        last_portion_valid = 0;
    }

    if (current_portion == 0) {
        /* Reset MCU */
        addr = (0x80 << 8) | BSP_MCU_CONTROL_REG;
        val  = 0x0000;
        lms_spi_write(addr, val, BSP_SPI_CS_LMS);

        /* Set mode */
        addr = (0x80 << 8) | BSP_MCU_CONTROL_REG;

        switch (prog_mode) {
        case BSP_PROG_EEPROM:
            val = 0x0001;
            lms_spi_write(addr, val, BSP_SPI_CS_LMS);
            break;

        case BSP_PROG_SRAM:
            val = 0x0002;
            lms_spi_write(addr, val, BSP_SPI_CS_LMS);
            break;

        case BSP_BOOT_MCU:
            val = 0x0003;
            lms_spi_write(addr, val, BSP_SPI_CS_LMS);

            /* Read MCU status (boot path) */
            addr = (0x00 << 8) | BSP_MCU_STATUS_REG;
            val  = lms_spi_read(addr, BSP_SPI_CS_LMS);

            /* Save portion and return immediately for boot */
            last_portion       = current_portion;
            last_portion_valid = 1;

            return (cmd_errors) ? STATUS_ERROR_CMD : STATUS_COMPLETED_CMD;

        default:
            return STATUS_ERROR_CMD;
        }
    }

    /* Wait until EMPTY_WRITE_BUFF = 1 */
    MCU_retries = 0;
    while (MCU_retries < BSP_MAX_MCU_RETRIES) {
        addr = (0x00 << 8) | BSP_MCU_STATUS_REG;
        val  = lms_spi_read(addr, BSP_SPI_CS_LMS);
        printf("%08x\n", val);

        if (val & 0x01)
            break;

        MCU_retries++;
        cdelay(3000);
    }

    /* Write 32 bytes to MCU FIFO */
    for (uint8_t block = 0; block < 32; block++) {
        addr = (0x80 << 8) | BSP_MCU_FIFO_WR_REG;
        val  = (0x00 << 8) | data[block];
        lms_spi_write(addr, val, BSP_SPI_CS_LMS);
    }

    /* Wait until EMPTY_WRITE_BUFF = 1 again */
    MCU_retries = 0;
    while (MCU_retries < 500) {
        addr = (0x00 << 8) | BSP_MCU_STATUS_REG;
        val  = lms_spi_read(addr, BSP_SPI_CS_LMS);

        if (val & 0x01)
            break;

        MCU_retries++;
        cdelay(3000);
    }

    /* Last portion: verify programming completed */
    if (current_portion == 255) {
        MCU_retries = 0;
        while (MCU_retries < BSP_MAX_MCU_RETRIES) {
            addr = (0x00 << 8) | BSP_MCU_STATUS_REG;
            val  = lms_spi_read(addr, BSP_SPI_CS_LMS);

            if (val & 0x40)
                break; /* PROGRAMMED = 1 */

            MCU_retries++;
            cdelay(30000);
        }

        if (MCU_retries == BSP_MAX_MCU_RETRIES)
            cmd_errors++;
    }

    /* Save portion tracking */
    last_portion       = current_portion;
    last_portion_valid = 1;

    return (cmd_errors) ? STATUS_ERROR_CMD : STATUS_COMPLETED_CMD;
}

uint8_t bsp_serial_read(uint8_t *data_field)
{
    return STATUS_ERROR_CMD;
}

uint8_t bsp_serial_write(const uint8_t *data_field)
{
    return STATUS_ERROR_CMD;
}