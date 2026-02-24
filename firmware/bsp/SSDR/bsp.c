#include "bsp.h"
#include "bsp/LimeSDR_XTRX/bsp.h"

litei2c_regs I2C0_REGS = {
    .master_active_addr = CSR_I2C0_MASTER_ACTIVE_ADDR,
    .master_addr_addr = CSR_I2C0_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C0_MASTER_SETTINGS_ADDR,
    .master_status_addr = CSR_I2C0_MASTER_STATUS_ADDR,
    .master_rxtx_addr = CSR_I2C0_MASTER_RXTX_ADDR
};

void bsp_init(void) {
    bsp_powerup();
}

void bsp_powerup(void) {
    printf("Initializing DC-DC switching regulators...\n");

    unsigned char adr;
    unsigned char dat;

    printf("PMICs Initialization...\n");
    printf("-----------------------\n");

    printf("FPGA_I2C1 PMIC: Check ID ");
    adr = 0x01;
    LP8758_read_reg(&I2C0_REGS, adr, &dat);
    if (dat != 0xe0) {
        printf("KO, exiting.\n");
    } else {
        printf("OK.\n");

        printf("PMIC: Enable Buck0.\n");
        adr = 0x02;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
        adr = 0x03;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: Enable Buck1.\n");
        adr = 0x04;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
        adr = 0x05;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: Enable Buck2.\n");
        adr = 0x06;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
        adr = 0x07;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: Enable Buck3.\n");
        adr = 0x08;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
        adr = 0x09;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        printf("PMIC: Set Buck1 to 2.0V.\n");
        adr = 0x0C;
        dat = 0xBB;
        //dat = 0xC1;  // 2.12V
        LP8758_write_reg(&I2C0_REGS, adr, dat);

        busy_wait(1);

        cdelay(10);
        pwr_ctrl_ldoen_write(0x1); //Enable LDO
    }
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
    //TODO: Check if delay is reasonably accurate

    // Implement platform-specific delay
    // Example: busy wait or use a hardware timer
    while (ms--) {
        // rough CPU delay loop (not accurate)
        for (volatile int i = 0; i < 1000; i++);
    }
}

int8_t lms_reset(uint8_t periph_id, uint8_t command) {
    uint8_t check_val = lms7002m_periph_id_check(periph_id);
    if (check_val == 0) return 1;
    uint32_t read_value;


    switch (command) {
        case LMS_RST_DEACTIVATE:
            // No implementation
            return 1;
        case LMS_RST_ACTIVATE:
            // No implementation
            return 1;

        case LMS_RST_PULSE:
            read_value = lime_top_lms7002_top_lms1_read() & ~(1 << CSR_LIME_TOP_LMS7002_TOP_LMS1_RESET_OFFSET);
            lime_top_lms7002_top_lms1_write(read_value);
            read_value |= (1 << CSR_LIME_TOP_LMS7002_TOP_LMS1_RESET_OFFSET);
            lime_top_lms7002_top_lms1_write(read_value);
            return 0;
    }
}

int8_t lms7002m_periph_id_check(uint8_t periph_id) {
    if (periph_id > MAX_ID_LMS7) {
        return 0; // Invalid ID
    }
    return 1; // Valid ID
}

int8_t lms8001_periph_id_check(uint8_t periph_id) {
    // NOTE: SSDR has LMS8's but the software does not use LMS8 associated commands
    //       thus, lms8 commands are not implemented
    return 2;
}

void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
    lms_spi_write(addr, val, periph_id);
}

uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id) {
    return lms_spi_read(addr, periph_id);
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
        uint8_t *val_ptr = (uint8_t *) &val;
        litei2c_a8d16_read_register(&I2C0_REGS, I2C_DAC_ADDR, 0x00, &val);
        *value_lsb = val_ptr[1];
        *value_msb = val_ptr[0];
        return STATUS_COMPLETED_CMD;
    }
    if (channel == 1) {
        uint16_t buf;
        uint8_t *buf_point = (uint8_t *) &buf;
        *unit = 0x50; // unit = 0.1C
        buf = TMP114_Read_Temp(&I2C0_REGS, I2C_TERMO_ADDR);
        buf = TMP114_Convert_Temp(buf);
        *value_lsb = buf_point[0];
        *value_msb = buf_point[1];
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb) {
    if (channel == 0 && unit == 0) {
        // TCXO DAC, RAW units
        uint16_t val = 0;
        uint8_t *val_ptr = (uint8_t *) &val;
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
    FlashQspi_CMD_ReadDataByte(mem_write_offset, &data[0]);
    FlashQspi_CMD_ReadDataByte(mem_write_offset + 1, &data[1]);
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data) {
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_SectorErase(mem_write_offset);
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_PageProgramByte(mem_write_offset, &data[0]);
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_PageProgramByte(mem_write_offset + 1, &data[1]);
}

uint8_t bsp_mem_read(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count) {
    // Check if the operation is going to be performed on EEPROM #1 and
    // that it's specifically being used to read VCTCXO DAC value
    // NOTE: condition for IF is copied from previous implementation, might need review
    if (data_count == 2 && target == 3 && progmode == 0 && offset == 16) {
        bsp_vctcxo_permanent_dac_read(data);
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_mem_write(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count) {
    // Check if the operation is going to be performed on EEPROM #1 and
    // that it's specifically being used to store VCTCXO DAC value
    // NOTE: condition for IF is copied from previous implementation, might need review
    if (data_count == 2 && target == 3 && progmode == 0 && offset == 16) {
        bsp_vctcxo_permanent_dac_write(data);
        return STATUS_COMPLETED_CMD;
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
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len,
                         uint8_t *misodata) {
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


uint8_t bsp_control_adf(uint8_t oe, const uint8_t data[3], bool pack_data) {
    // No ADF on this board
    return 1;
}

uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    return 1;
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    return 1;
}

uint8_t bsp_program_mode2_check_support(void) {
    return 1;
}

uint8_t bsp_program_mode2_boot_from_flash(void) {
    return 1;
}


uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    //Both user and gold share the same core implementation
    bsp_program_flash(current_portion, data_cnt, payload);
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    //Both user and gold share the same core implementation
    bsp_program_flash(current_portion, data_cnt, payload);
}

// Same Implementation for both user and gold
uint8_t bsp_program_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    static int address;
    static uint16_t page_buffer_cnt;
    static uint64_t total_data = 0;
    static uint8_t inc_data_count;
    static uint8_t page_buffer[256];
    static int PAGE_SIZE = 256;
    static uint8_t data_to_copy; // how much data to copy to page buffer (incase of overflow)
    static uint8_t data_leftover;

    // write data to Flash from PC
    // Start of programming? reset variables
    if (current_portion == 0) {
        // Gold image must be written at address 0x0
        if (payload[0] == 3) {
            address = 0;
            // printf("DEBUG: Gold Image write to flash\n");
        } else {
            // User image must be written at offset
            address = 0x00310000;
            // printf("DEBUG: User Image write to flash\n");
        }

        page_buffer_cnt = 0;
        total_data = 0;
        // Erase first sector
        FlashQspi_EraseSector(address);
    }

    inc_data_count = payload[5];

    // Check if final packet
    if (inc_data_count == 0) {
        // Flush leftover data, if any
        if (page_buffer_cnt > 0) {
            // Fill unused page data with 1 (no write)
            memset(&page_buffer[page_buffer_cnt], 0xFF, PAGE_SIZE - page_buffer_cnt);
            FlashQspi_ProgramPage(address, page_buffer);
        }
    } else {
        if (PAGE_SIZE < (inc_data_count + page_buffer_cnt)) {
            // Incoming data would overflow the page buffer
            // Calculate ammount of data to copy
            data_to_copy = PAGE_SIZE - page_buffer_cnt;
            data_leftover = page_buffer_cnt - data_to_copy;
            memcpy(&page_buffer[page_buffer_cnt], &payload[24],
                   data_to_copy);
            // We already know the page is full because of overflowing input
            FlashQspi_ProgramPage(address, page_buffer);
            address += 256;
            total_data += 256;
            // Check if new address is bottom of sector, erase if needed
            if ((address & 0xFFF) == 0)
                FlashQspi_EraseSector(address);
            memcpy(&page_buffer[0], &payload[24 + data_to_copy],
                   data_leftover);
            page_buffer_cnt = data_leftover;
        } else {
            // Incoming data would not overflow the page buffer
            memcpy(&page_buffer[page_buffer_cnt], &payload[24],
                   inc_data_count);
            page_buffer_cnt += inc_data_count;
            if (page_buffer_cnt == PAGE_SIZE) {
                FlashQspi_ProgramPage(address, page_buffer);
                page_buffer_cnt = 0;
                address += 256;
                total_data += 256;
                // Check if new address is bottom of sector, erase if needed
                if ((address & 0xFFF) == 0)
                    FlashQspi_EraseSector(address);
            }
        }
    }
}
