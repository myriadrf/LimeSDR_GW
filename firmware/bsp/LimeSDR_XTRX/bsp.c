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
    LP8758_read_reg(&I2C0_REGS,adr,&dat);
    if (dat != 0xe0) {
        printf("KO, exiting.\n");
    } else {
        printf("OK.\n");

        printf("PMIC: Enable Buck0.\n");
        adr = 0x02;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
        adr = 0x03;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: Enable Buck1.\n");
        adr = 0x04;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
        adr = 0x05;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: Enable Buck2.\n");
        adr = 0x06;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
        adr = 0x07;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: Enable Buck3.\n");
        adr = 0x08;
        dat = 0x88;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
        adr = 0x09;
        dat = 0xD2;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        printf("PMIC: Set Buck1 to 3.3V.\n");
        adr = 0x0C;
        dat = 0xFC;
        LP8758_write_reg(&I2C0_REGS,adr,dat);

        busy_wait(1);
    }

    printf("FPGA_I2C2 PMIC: Check ID ");
    adr = 0x01;
    LP8758_read_reg(&I2C1_REGS,adr,&dat);
    if (dat != 0xe0) {
        printf("KO, exiting.\n");
    } else {
        printf("OK.\n");

        printf("PMIC: Enable Buck0.\n");
        adr = 0x02;
        dat = 0x88;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
        adr = 0x03;
        dat = 0xD2;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Enable Buck1.\n");
        adr = 0x04;
        dat = 0x88;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
        adr = 0x05;
        dat = 0xD2;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Enable Buck2.\n");
        adr = 0x06;
        dat = 0x88;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
        adr = 0x07;
        dat = 0xD2;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Enable Buck3.\n");
        adr = 0x08;
        dat = 0x88;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
        adr = 0x09;
        dat = 0xD2;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Set Buck0 to 1.5V.\n");
        adr = 0x0A;
        dat = 0xA2;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Set Buck1 to 3.3V.\n");
        adr = 0x0C;
        dat = 0xFC;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Set Buck2 to 1.75V.\n");
        adr = 0x0E;
        dat = 0xAF;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Set Buck3 to 2.05V.\n");
        adr = 0x10;
        dat = 0xBE;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        printf("PMIC: Clear INT_BUCK_2_3 Status.\n");
        adr = 0x1A;
        dat = 0xFF;
        LP8758_write_reg(&I2C1_REGS,adr,dat);

        busy_wait(1);
    }
}

void bsp_shutdown(void) {
//No implementation intended for this board
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
    //TODO: Check if delay is reasonably accurate

    // Implement platform-specific delay
    // Example: busy wait or use a hardware timer
    while (ms--) {
        // rough CPU delay loop (not accurate)
        for (volatile int i = 0; i < 1000; i++);
    }
}

int8_t lms7002m_periph_id_check(uint8_t periph_id) {
    if (periph_id > MAX_ID_LMS7) {
        return 0; // Invalid ID
    }
    return 1; // Valid ID
}

int8_t lms8001_periph_id_check(uint8_t periph_id) {
    // No LMS8's on XTRX board
    return 2; // No LMS8's
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
