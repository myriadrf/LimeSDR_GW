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
    //TODO: Check if delay is reasonably accurate

    // Implement platform-specific delay
    // Example: busy wait or use a hardware timer
    while (ms--) {
        // rough CPU delay loop (not accurate)
        for (volatile int i = 0; i < 1000; i++);
    }
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
    return 1;
    // No implementation
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    static uint8_t state, Flash = 0x0;
    static uint32_t word = 0x0;
    static int address;
    static uint32_t byte = 0;
    uint8_t retval = 0;

    if (current_portion == 0) state = 10;
    if (data_cnt == 0) {
        state = 30;
    }
    Flash = 1;

    while (Flash) {
        switch (state) {
            //Init
            case 10:
                //Set Flash memory addresses
                address = UFMStartAddress;
                //Write Control Register of On-Chip Flash IP to un-protect and erase operation
                //wishbone_write32(ONCHIP_FLASH_0_CSR_BASE + (1<<2), 0xf67fffff);
                //wishbone_write32(ONCHIP_FLASH_0_CSR_BASE + (1<<2), 0xf65fffff);
                internal_flash_control_register_write(0xf67fffff);
                internal_flash_control_register_write(0xf65fffff);

                state = 11;
                Flash = 1;

            case 11:
                //Start erase CFM0
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

            //Initiate UFM (ID1) Erase Operation
            case 13:
                //Write Control Register of On-Chip Flash IP to un-protect and erase operation
                internal_flash_control_register_write(0xf67fffff);
                internal_flash_control_register_write(0xf61fffff);

                state = 14;
                Flash = 1;
                break;

            case 14:
                //Start erase UFM ID1
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

            //Initiate UFM (ID2) Erase Operation
            case 16:

                //Write Control Register of On-Chip Flash IP to un-protect and erase operation
                internal_flash_control_register_write(0xf67fffff);
                internal_flash_control_register_write(0xf62fffff);

                state = 17;
                Flash = 1;
                break;

            case 17:
                //Start erase UFM ID2
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

            //Program
            case 20:
                for (byte = 24; byte <= 52; byte += 4) {
                    //Take word and swap bits
                    word = ((uint32_t) reverse(payload[byte + 0]) << 24)
                           & 0xFF000000;
                    word |= ((uint32_t) reverse(payload[byte + 1]) << 16)
                            & 0x00FF0000;
                    word |= ((uint32_t) reverse(payload[byte + 2]) << 8)
                            & 0x0000FF00;
                    word |= ((uint32_t) reverse(payload[byte + 3]) << 0)
                            & 0x000000FF;

                    //Command to write into On-Chip Flash IP
                    if (address <= CFM0EndAddress) {
                        *(uint32_t *) (INTERNAL_FLASH_BASE + (address << 2)) = word;
                        //wishbone_write32(ONCHIP_FLASH_0_DATA_BASE + (address<<2), word);

                        while ((internal_flash_status_register_read() & 0x0b) == 0x02) {
                            //printf("Writing CFM0(%d)\n", address);
                        }

                        if ((internal_flash_status_register_read() & 0x0b) == 0x00) {
                            state = 0;
                            address = 700000;
                        }

                        if ((internal_flash_status_register_read() & 0x0b) == 0x08) {
                        };

                        // Increment address or move to CFM0 sector
                        if (address == UFMEndAddress) address = CFM0StartAddress;
                        else address += 1;
                    } else {
                        retval = 1;
                    };
                };

                state = 20;
                Flash = 0;
                retval = 0;

                break;

            //Finish
            case 30:
                //Re-protect the sector
                //IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xffffffff);
                internal_flash_control_register_write(0xffffffff);

                state = 0;
                Flash = 0;
                retval = 0;

                break;

            default:
                retval = 1;
                state = 0;
                Flash = 0;
        };
    };
    return retval;
}

uint8_t bsp_program_mode2_check_support(void) {
    return 0;
}

uint8_t bsp_program_mode2_boot_from_flash(void) {
    // Copy-pasted old implementation

    uint32_t reg;
    //set CONFIG_SEL overwrite to 1 and CONFIG_SEL to Image 0
    //wishbone_write32(DUAL_BOOT_0_BASE+(1<<2), 0x00000001);
    reg = 0x00000001;

    //set CONFIG_SEL overwrite to 1 and CONFIG_SEL to Image 1
    //IOWR(DUAL_BOOT_0_BASE, 1, 0x00000003);
    reg = 0x00000003;

    *(uint32_t *) (DUAL_CFG_BASE + (1 << 2)) = reg;

    /*wait while core is busy*/
    while (1) {
        reg = *(uint32_t *) (DUAL_CFG_BASE + (3 << 2));
        cdelay(2000);
        if (reg != 0x01)
            break;
    }

    //Trigger reconfiguration to selected Image
    //wishbone_write32(DUAL_BOOT_0_BASE+(0<<2), 0x00000001);
    reg = 0x00000001;
    *(uint32_t *) (DUAL_CFG_BASE + (0 << 2)) = reg;

    return 0;
}

uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    return 1;
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload) {
    return 1;
}
