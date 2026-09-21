#include "bsp.h"

static uint8_t serial_otp_unlock_key = 0;

static uint16_t dac_val     = 0;
static uint8_t *dac_val_ptr = (uint8_t *)&dac_val;

uint16_t g_bsp_hw_ver;

void bsp_init(void)
{
    bsp_powerup();
    spimaster1_phase_write(1);
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
            bsp_analog_write(BSP_DAC_INDEX, 0x00, (BSP_DAC_DEFAULT_VAL & 0xff00) >> 8, BSP_DAC_DEFAULT_VAL & 0xff);
        }
    }
    // Read actual hardware version from register and store it in global variable
    g_bsp_hw_ver = limetop_fpgacfg_bom_hw_ver_read();
    g_bsp_hw_ver &= 0x000F;
}

void bsp_powerup(void)
{
    // CA23 power supplies operate autonomously
}

void bsp_shutdown(void)
{
    // No implementation intended for this board
}

void bsp_isr_init(void)
{
    // BSP isr controller not implemented in this board's gw
}

void bsp_process_irqs(void)
{
    // BSP isr controller not implemented in this board's gw
}

void bsp_delay_ms(unsigned int ms)
{
    while (ms--) {
        for (volatile int i = 0; i < 1000; i++)
            ;
    }
}

int8_t lms_reset(uint8_t periph_id, uint8_t command)
{
    uint8_t check_val = lms7002m_periph_id_check(periph_id);
    uint16_t lms1_val;
    if (check_val == 0)
        return 1;
    switch (command) {
        case LMS_RST_DEACTIVATE:
            lms1_val = limetop_lms7002_top_lms1_read();
            // Set bit 1 (reset) to 1
            lms1_val |= 2;
            limetop_lms7002_top_lms1_write(lms1_val);
            return 0;

        case LMS_RST_ACTIVATE:
            lms1_val = limetop_lms7002_top_lms1_read();
            // Set bit 1 (reset) to 0
            lms1_val &= 0xFFFD;
            limetop_lms7002_top_lms1_write(lms1_val);
            return 0;

        case LMS_RST_PULSE:
            lms1_val = limetop_lms7002_top_lms1_read();
            // Set bit 1 (reset) to 0
            lms1_val &= 0xFFFD;
            limetop_lms7002_top_lms1_write(lms1_val);
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
            // Set bit 1 (reset) to 1
            lms1_val |= 2;
            limetop_lms7002_top_lms1_write(lms1_val);
            return 0;

        default:
            return 1;
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
    // No LMS8's on CA23 board
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
    // no LMS8 on CA23
}

uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id)
{
    return -1;
}

uint8_t bsp_analog_read(uint8_t channel, uint8_t *unit, uint8_t *value_msb, uint8_t *value_lsb)
{
    if (channel == BSP_DAC_INDEX) {
        // Channel 0: TCXO DAC value (return cached value)
        *unit      = 0x00;
        *value_lsb = dac_val_ptr[0];
        *value_msb = dac_val_ptr[1];
        return STATUS_COMPLETED_CMD;
    }
    if (channel == 1) {
        // Channel 1: Temperature telemetry - dummy 0xDEADBEEF with unit 0x50
        *value_lsb = 0xEF;
        *value_msb = 0xBE;
        *unit      = 0x50;
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_analog_write(uint8_t channel, uint8_t unit, uint8_t value_msb, uint8_t value_lsb)
{
    if (channel == BSP_DAC_INDEX && unit == 0) {
        // TCXO DAC, RAW units (16-bit AD5662 SPI DAC)
        dac_val_ptr[0] = value_lsb;
        dac_val_ptr[1] = value_msb;
        ad56xx_write(BSP_DAC_SPIMASTER, BSP_DAC_CS, dac_val, AD56XX_MODEL_AD5662, AD56XX_PWR_NORMAL);
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_gpio_dir_read(uint8_t *data, uint8_t offset)
{
    // Unsupported on CA23
    return 1;
}

uint8_t bsp_gpio_dir_write(uint8_t data, uint8_t offset)
{
    // Unsupported on CA23
    return 1;
}

uint8_t bsp_gpio_read(uint8_t *data, uint8_t offset)
{
    // Unsupported on CA23
    return 1;
}

uint8_t bsp_gpio_write(uint8_t data, uint8_t offset)
{
    // Unsupported on CA23
    return 1;
}

uint8_t bsp_gpio_get_cached(const uint8_t offset)
{
    // Unsupported on CA23
    return 1;
}

void bsp_vctcxo_permanent_dac_read(uint8_t *data)
{
    // TODO: FIX this, first FLASH read returns 0xFF. Workaround to read two times...
    FlashQspi_CMD_ReadDataByte(BSP_FLASH_STORAGE_OFFSET, &data[0]);
    FlashQspi_CMD_ReadDataByte(BSP_FLASH_STORAGE_OFFSET, &data[0]);
    FlashQspi_CMD_ReadDataByte(BSP_FLASH_STORAGE_OFFSET + 1, &data[1]);
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data)
{
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_SectorErase(BSP_FLASH_STORAGE_OFFSET);
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_PageProgramByte(BSP_FLASH_STORAGE_OFFSET, &data[0]);
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_PageProgramByte(BSP_FLASH_STORAGE_OFFSET + 1, &data[1]);
}

uint8_t
bsp_mem_read(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    // Check if the operation is going to be performed on target 3 (EEPROM simulation) and
    // that it's specifically being used to read VCTCXO DAC value
    if (data_count == 2 && target == 3 && progmode == 0 && offset == BSP_EEPROM_DAC_ADDR) {
        bsp_vctcxo_permanent_dac_read(data);
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t
bsp_mem_write(uint32_t offset, uint32_t portion, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count)
{
    // Check if the operation is going to be performed on target 3 (EEPROM simulation) and
    // that it's specifically being used to store VCTCXO DAC value
    if (data_count == 2 && target == 3 && progmode == 0 && offset == BSP_EEPROM_DAC_ADDR) {
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
uint8_t bsp_spi_transfer(
    uint8_t master, uint8_t cs, const uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata)
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

    switch (master) {
    case 0: // LMS7002M SPI: 32-bit data_width
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

    case 1: // AD5662 DAC SPI: 24-bit data_width
        spimaster1_phase_write(1);
        packed_mosi <<= (3 - transfer_len) * 8;
        spimaster1_cs_write(cs_mask);
        cdelay(1);
        while ((spimaster1_status_read() & 0x1) == 0) {
        }
        spimaster1_mosi_write(packed_mosi);
        spimaster1_control_write(bits * SPI_LENGTH | SPI_START);
        while ((spimaster1_status_read() & 0x1) == 0) {
        }
        recv_val = spimaster1_miso_read();
        break;

    default:
        return 1;
    }

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

uint8_t bsp_program_mode0_fpga_sram(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    return 1;
}

uint8_t bsp_program_mode1_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // Mode 1 behaves the same as user mode on this board
    return bsp_program_flash(current_portion, data_cnt, payload);
}

uint8_t bsp_program_mode2_check_support(void)
{
    return 1;
}

uint8_t bsp_program_mode2_boot_from_flash(void)
{
    return 1;
}

uint8_t bsp_program_mode3_golden_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // Both user and gold share the same core implementation
    return bsp_program_flash(current_portion, data_cnt, payload);
}

uint8_t bsp_program_mode4_user_to_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
    // Both user and gold share the same core implementation
    return bsp_program_flash(current_portion, data_cnt, payload);
}

uint8_t bsp_lms_mcu_fw_wr(uint8_t prog_mode, uint8_t current_portion, const uint8_t *data)
{
    return STATUS_ERROR_CMD;
}

// Same Implementation for both user and gold
uint8_t bsp_program_flash(uint32_t current_portion, uint8_t data_cnt, const uint8_t *payload)
{
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
            address = 0x220000;
            // printf("DEBUG: User Image write to flash\n");
        }

        page_buffer_cnt = 0;
        total_data      = 0;
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
            data_to_copy  = PAGE_SIZE - page_buffer_cnt;
            data_leftover = page_buffer_cnt - data_to_copy;
            memcpy(&page_buffer[page_buffer_cnt], &payload[24], data_to_copy);
            // We already know the page is full because of overflowing input
            FlashQspi_ProgramPage(address, page_buffer);
            address += 256;
            total_data += 256;
            // Check if new address is bottom of sector, erase if needed
            if ((address & 0xFFF) == 0)
                FlashQspi_EraseSector(address);
            memcpy(&page_buffer[0], &payload[24 + data_to_copy], data_leftover);
            page_buffer_cnt = data_leftover;
        } else {
            // Incoming data would not overflow the page buffer
            memcpy(&page_buffer[page_buffer_cnt], &payload[24], inc_data_count);
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
    // no errors
    return 0;
}

uint8_t bsp_serial_read(uint8_t *data_field)
{
    uint8_t tmprd_serial[32];
    FlashQspi_CMD_ReadOTPData(BSP_OTP_SERIAL_ADDR, 32, tmprd_serial);
    memcpy(data_field + 24, tmprd_serial, 32);
    data_field[1] = 16;
    data_field[2] = serial_otp_unlock_key;
    return STATUS_COMPLETED_CMD;
}

uint8_t bsp_serial_write(const uint8_t *data_field)
{
    uint8_t storage_type = data_field[0];
    uint8_t provided_key = data_field[2];
    uint8_t tmp_serial[32];

    if (storage_type != 3)
        return STATUS_ERROR_CMD;

    if (serial_otp_unlock_key == BSP_OTP_UNLOCK_KEY) {
        memcpy(tmp_serial, data_field + 24, 32);
        FlashQspi_ProgramOTP(BSP_OTP_SERIAL_ADDR, data_field[1], tmp_serial);
        serial_otp_unlock_key = 0;
        return STATUS_COMPLETED_CMD;
    } else if (provided_key == BSP_OTP_UNLOCK_KEY) {
        serial_otp_unlock_key = provided_key;
        return STATUS_COMPLETED_CMD;
    }

    return STATUS_RESOURCE_DENIED_CMD;
}

void gnss_init(void) {
    // TODO: for now this uses PCIE_UART0. This is good for CA23, but might need some
    //       modifications to make it more dynamic in the future
    // Enable ZDA messages
    char pmtk_msg[] = "$PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,1,0*2D\r\n";
    // Enable PPS
    char pmtk_msg2[] = "$PMTK285,4,100*38\r\n";
    for (int i = 0; pmtk_msg2[i] != '\0'; i++) {
        PCIE_UART0_rxtx_write((uint32_t)pmtk_msg2[i]);
    }
    cdelay(0xFFFFF);
    for (int i = 0; pmtk_msg[i] != '\0'; i++) {
        PCIE_UART0_rxtx_write((uint32_t)pmtk_msg[i]);
    }
}
