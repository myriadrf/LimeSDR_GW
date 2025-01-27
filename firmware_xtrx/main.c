/*
-- ----------------------------------------------------------------------------
-- FILE        : main.c
-- DESCRIPTION : LimeSDR XTRX/Mini firmware main.
-- DATE        : 2015-2024
-- AUTHOR(s)   : Lime Microsystems
-- REVISION    : -
-- ----------------------------------------------------------------------------
*/

#include "stdint.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "i2c0.h"
#include "lms7002m.h"
#include "LMS64C_protocol.h"
#ifdef LIMESDR_XTRX
#include <libbase/uart.h>
#include <libbase/console.h>
#include "i2c1.h"
#include "fpga_flash_qspi.h"
#include "LimeSDR_XTRX.h"
#include "regremap.h"
#include "Xil_clk_drp.h"
#else
#include "LimeSDR_MINI_brd_v1r0.h"
#include "csr_access.h"
#include "spiflash.h"
#endif

#define sbi(p, n) ((p) |= (1UL << (n)))
#define cbi(p, n) ((p) &= ~(1 << (n)))

/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#ifdef LIMESDR_XTRX
#define I2C_DAC_ADDR     0x4C
#define I2C_TERMO_ADDR   0x4B
#define LP8758_I2C_ADDR  0x60
//#define FW_VER 1 // Initial version
//#define FW_VER 2 // Fix for PLL config. hang when changing from low to high frequency.
//#define FW_VER 3 // Added serial number into GET_INFO cmd
#define FW_VER 5 // Firmware for Litex project

#else // LIMESDR_MINI_V1 / LIMESDR_MINI_V2
#define SPI_LMS7002_SELECT 0x01
#define SPI_FPGA_SELECT 0x02

//CMD_PROG_MCU
#define PROG_EEPROM 1
#define PROG_SRAM   2
#define BOOT_MCU    3

///CMD_PROG_MCU
#define MCU_CONTROL_REG 0x02
#define MCU_STATUS_REG  0x03
#define MCU_FIFO_WR_REG 0x04

#define MAX_MCU_RETRIES 30

#define DAC_VAL_ADDR  			0x0010		// Address in EEPROM memory where TCXO DAC value is stored
#define DAC_VAL_ADDR_IN_FLASH  	0x00FF0000	// Address in FLASH memory where TCXO DAC value is stored
#define DAC_DEFF_VAL			566			// Default TCXO DAC value loaded when EEPROM is empty

#define FLASH_USRSEC_START_ADDR	0x00400000  // Start address for user space in FLASH memory

#ifdef LIMESDR_MINI_V2
//get info
//#define FW_VER				1 //Initial version
//#define FW_VER				2 //FLASH programming added
//#define FW_VER				3 //Temperature and Si5351C control added
//#define FW_VER				4 //LM75 configured to control fan; I2C speed increased up to 400kHz; ADF/DAC control implementation.
//#define FW_VER				5 //EEPROM and FLASH R/W functionality added
//#define FW_VER				6 // DAC value read from EEPROM memory
//#define FW_VER				7 // DAC value read from FLASH memory
//#define FW_VER				8 // Added FLASH write command protect when write count is 0
//#define FW_VER				9 // Temporary fix for LM75 configuration
#define FW_VER			   10 // Fix for LM75 temperature reading with 0.5 precision
#endif
#endif

/* DEBUG */
//#define DEBUG_FIFO
//#define DEBUG_CSR_ACCESS
//#define DEBUG_LMS_SPI
//#define DEBUG_CMD

/************************** Variable Definitions *****************************/
uint8_t block, cmd_errors;
uint8_t glEp0Buffer_Rx[64], glEp0Buffer_Tx[64];
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet *) glEp0Buffer_Tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet *) glEp0Buffer_Rx;

int boot_img_en = 0;

#ifdef LIMESDR_XTRX
// If an error points here, most likely some of the macros are invalid.
PLL_ADDRS pll1_rx_addrs = GENERATE_MMCM_DRP_ADDRS(CSR_LMS7002_TOP_LMS7002_CLK_PLL1_RX_MMCM);
PLL_ADDRS pll0_tx_addrs = GENERATE_MMCM_DRP_ADDRS(CSR_LMS7002_TOP_LMS7002_CLK_PLL0_TX_MMCM);
SMPL_CMP_ADDRS smpl_cmp_addrs = GENERATE_SMPL_CMP_ADDRS(CSR_LMS7002_TOP);
// clk_ctrl_addrs is declared in regremap.h
CLK_CTRL_ADDRS clk_ctrl_addrs = GENERATE_CLK_CTRL_ADDRS(CSR_LMS7002_TOP_LMS7002_CLK_CLK_CTRL);

//Flash programming variables
volatile uint8_t flash_prog_pending = 0;

uint8_t page_buffer[256]; // page buffer
uint16_t page_buffer_cnt; // how many bytes are present in buffer
uint64_t total_data = 0; // how much data has been transferred in total (debug value)
uint8_t inc_data_count;
int PAGE_SIZE = 256;
uint8_t data_to_copy; // how much data to copy to page buffer (incase of overflow)
uint8_t data_leftover;

//Clock config variables
volatile uint8_t clk_cfg_pending = 0;
volatile uint8_t var_phcfg_start;
volatile uint8_t var_pllcfg_start;
volatile uint8_t var_pllrst_start;


unsigned int irq_mask;

uint16_t dac_val = DAC_DEFF_VAL;

uint8_t serial_otp_unlock_key = 0;
volatile unsigned char serial[32] = {0};
volatile unsigned char tmp_serial[32] = {0};
volatile unsigned char tmprd_serial[32] = {0};

// Since there is no eeprom on the XTRX board and the flash is too large for the gw
// we use the top of the flash instead of eeprom, thus the offset to last sector
#define mem_write_offset 0x01FF0000
#else
unsigned long int last_portion, fpga_data;
int flash_page = 0, flash_page_data_cnt = 0, flash_data_cnt_free = 0, flash_data_counter_to_copy = 0;
unsigned char sc_brdg_data[4];
unsigned char flash_page_data[FLASH_PAGE_SIZE];
tBoard_Config_FPGA *Board_Config_FPGA = (tBoard_Config_FPGA*) flash_page_data;
unsigned long int fpga_byte;

uint32_t byte = 0;
uint32_t byte1;
uint32_t word = 0x0;
uint8_t state, Flash = 0x0;
char spiflash_wdata[4];
uint8_t MCU_retries;

const unsigned int uiBlink = 1;

// Used for MAX10 Flash programming
#ifdef LIMESDR_MINI_V1
uint32_t CFM0StartAddress = 0x012800;
uint32_t CFM0EndAddress   = 0x022FFF;
uint32_t UFMStartAddress = 0x0;
uint32_t UFMEndAddress   = 0x01FFF;
uint32_t word;
#else
uint32_t CFM0StartAddress = 0x000000;
uint32_t CFM0EndAddress   = 0x13FFFF;
#endif
uint16_t dac_val = 720;
unsigned char dac_data[2];

signed short int converted_value = 300;
#endif

volatile uint8_t lms64_packet_pending;

//FPGA conf
int data_cnt = 0;
unsigned long int current_portion;
int address;

#ifdef LIMESDR_XTRX
/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void) {
    char c[2];
    static char s[64];
    static int ptr = 0;

    if (readchar_nonblock()) {
        c[0] = getchar();
        c[1] = 0;
        switch (c[0]) {
            case 0x7f:
            case 0x08:
                if (ptr > 0) {
                    ptr--;
                    fputs("\x08 \x08", stdout);
                }
                break;
            case 0x07:
                break;
            case '\r':
            case '\n':
                s[ptr] = 0x00;
                fputs("\n", stdout);
                ptr = 0;
                return s;
            default:
                if (ptr >= (sizeof(s) - 1))
                    break;
                fputs(c, stdout);
                s[ptr] = c[0];
                ptr++;
                break;
        }
    }

    return NULL;
}

static char *get_token(char **str) {
    char *c, *d;

    c = (char *) strchr(*str, ' ');
    if (c == NULL) {
        d = *str;
        *str = *str + strlen(*str);
        return d;
    }
    *c = 0;
    d = *str;
    *str = c + 1;
    return d;
}

static void prompt(void) {
    printf("\e[92;1mlimesdr-xtrx\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void) {
    puts("\nLimeSDR XTRX minimal demo app built "__DATE__" "__TIME__"\n");
    puts("Available commands:");
    puts("help               - Show this command");
    puts("reboot             - Reboot CPU");
    puts("i2c_test           - Test I2C Buses");
    puts("init_pmic          - Initialize DC-DC switching regulators");
    puts("dump_pmic          - Dump DC-DC switching regulator configuration");
#ifdef CSR_LEDS_BASE
    puts("led                - Led demo");
#endif
#ifdef CSR_BASE
    puts("gpioled            - GPIO override demo");
    puts("mmap               - MMAP demo");
#endif
#ifdef CSR_BASE
    puts("lms                - lms7002_top module status");
#endif
    puts("dac_test           - Test DAC");
    puts("flash_test         - Test FPGA FLASH");
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void) {
    ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE

static void led_cmd(void) {
    int i;
    printf("Led demo...\n");

    printf("Counter mode...\n");
    for (i = 0; i < 32; i++) {
        leds_out_write(i);
        busy_wait(100);
    }

    printf("Shift mode...\n");
    for (i = 0; i < 4; i++) {
        leds_out_write(1 << i);
        busy_wait(200);
    }
    for (i = 0; i < 4; i++) {
        leds_out_write(1 << (3 - i));
        busy_wait(200);
    }

    printf("Dance mode...\n");
    for (i = 0; i < 4; i++) {
        leds_out_write(0x55);
        busy_wait(200);
        leds_out_write(0xaa);
        busy_wait(200);
    }
}

#endif

#ifdef CSR_BASE

static void gpioled_cmd(void) {
    int i, j;
    printf("GPIO Led override demo...\n");
    gpio_gpio_override_write(0x7);
    gpio_gpio_override_dir_write(0x0);
    gpio_gpio_override_val_write(0x0);
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 8; j++) {
            gpio_gpio_override_val_write(0x0);
            busy_wait(100);
            gpio_gpio_override_val_write(1 << i);
            busy_wait(100);
        }
    }

    gpio_gpio_override_val_write(0x0);
    printf("GPIO Led override demo...ended\n");
}

static void mmap_cmd(void) {
    int i;

    volatile uint32_t *mmap_ptr = (volatile uint32_t *) LIME_TOP_MMAP_BASE;
    printf("MMAP demo...\n");

    /* Write counter values to SRAM. */

    printf("Writing values to SRAM at address 0x%08lX...\n", LIME_TOP_MMAP_BASE);
    for (i = 0; i < 8; i++) {
        mmap_ptr[i] = i;
        printf("Wrote %d to address 0x%08X\n", i, (unsigned int) (LIME_TOP_MMAP_BASE + i * sizeof(uint32_t)));
    }

    /* Read back values from SRAM. */
    printf("Reading values from SRAM at address 0x%08lX...\n", LIME_TOP_MMAP_BASE);
    for (i = 0; i < 8; i++) {
        uint32_t value = mmap_ptr[i];
        printf("Read %ld from address 0x%08X\n", value, (unsigned int) (LIME_TOP_MMAP_BASE + i * sizeof(uint32_t)));
    }
    // *mmap_ptr = (volatile uint32_t *)CSR_BASE;
    // printf("Reading values from SRAM at address 0x%08lX...\n", CSR_BASE);
    // for (i = 0; i < 8; i++) {
    // 	uint32_t value = mmap_ptr[i];
    // 	printf("Read 0x%08X from address 0x%08X\n", value, (unsigned int)(CSR_BASE + i * sizeof(uint32_t)));
    // }
    uint32_t dest;
    printf(" CNTRL: ");
    for (int cnt = 0; cnt < 16; cnt++) {
        dest = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
        printf("%lx ", dest);
    }
    printf(" \n");

    printf(" CNTRL wr: \n");
    csr_write_simple(CSR_CNTRL_CNTRL_ADDR, 5);
}

#endif

/*-----------------------------------------------------------------------*/
/* I2C                                                                   */
/*-----------------------------------------------------------------------*/

static void i2c_test(void) {
    printf("I2C0 Scan...\n");
    i2c0_scan();

    printf("\n");

    printf("I2C1 Scan...\n");
    i2c1_scan();
}

static void dac_test(void) {
    unsigned char dat[2];

    unsigned char adr = 0;

    i2c0_read(I2C_DAC_ADDR, adr, dat, 2, true);
    printf("Read DAC val...\n");
    printf("0x%02x: 0x%02x\n", dat[0], dat[1]);
    //i2c0_scan();

    printf("\n");

    printf("Write DAC val...\n");
    adr = 0x30;
    dat[0] = 0xaa;
    dat[1] = 0x55;
    i2c0_write(I2C_DAC_ADDR, adr, dat, 2);
    //i2c1_scan();
    i2c0_read(I2C_DAC_ADDR, adr, dat, 2, true);
    printf("Read DAC val...\n");
    printf("0x%02x: 0x%02x\n", dat[0], dat[1]);
}

static void flash_test(void) {
    uint8_t regvals2[2];
    uint8_t data_bytes[2];
    printf("FPGA FLASH test...\n");
    FlashQspi_CMD_ReadRDSR(&regvals2[0]);
    FlashQspi_CMD_ReadRDCR(&regvals2[1]);
    printf("FPGA FLASH test end...\n");
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_WRDI();
    FlashQspi_CMD_ReadDataByte(0x4fff5, data_bytes);

    // Erase sector
    FlashQspi_CMD_WREN();
    /*retval =*/ FlashQspi_CMD_SectorErase(0x4fff5);

    uint8_t myByte = 0x55;
    uint32_t myAddress = 0x4fff5;
    // Write Byte
    FlashQspi_CMD_WREN();
    /*int result =*/ FlashQspi_CMD_PageProgramByte(myAddress, &myByte);

    FlashQspi_CMD_ReadDataByte(0x4fff5, data_bytes);
}

static void dump_pmic(void) {
    unsigned char adr;
    unsigned char dat;
    printf("FPGA_I2C1 PMIC Dump...\n");
    for (adr = 0; adr < 32; adr++) {
        i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
        printf("0x%02x: 0x%02x\n", adr, dat);
    }
    printf("FPGA_I2C2 PMIC Dump...\n");
    for (adr = 0; adr < 32; adr++) {
        i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
        printf("0x%02x: 0x%02x\n", adr, dat);
    }
}

static void init_pmic(void) {
    printf("Initializing DC-DC switching regulators...\n");

    unsigned char adr;
    unsigned char dat;

    printf("PMICs Initialization...\n");
    printf("-----------------------\n");

    printf("FPGA_I2C1 PMIC: Check ID ");
    adr = 0x01;
    i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
    if (dat != 0xe0) {
        printf("KO, exiting.\n");
    } else {
        printf("OK.\n");

        printf("PMIC: Enable Buck0.\n");
        adr = 0x02;
        dat = 0x88;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
        adr = 0x03;
        dat = 0xD2;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Enable Buck1.\n");
        adr = 0x04;
        dat = 0x88;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
        adr = 0x05;
        dat = 0xD2;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Enable Buck2.\n");
        adr = 0x06;
        dat = 0x88;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
        adr = 0x07;
        dat = 0xD2;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Enable Buck3.\n");
        adr = 0x08;
        dat = 0x88;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
        adr = 0x09;
        dat = 0xD2;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Set Buck1 to 3.3V.\n");
        adr = 0x0C;
        dat = 0xFC;
        i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

        busy_wait(1);
    }


    printf("FPGA_I2C2 PMIC: Check ID ");
    adr = 0x01;
    i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
    if (dat != 0xe0) {
        printf("KO, exiting.\n");
    } else {
        printf("OK.\n");

        printf("PMIC: Enable Buck0.\n");
        adr = 0x02;
        dat = 0x88;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
        adr = 0x03;
        dat = 0xD2;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Enable Buck1.\n");
        adr = 0x04;
        dat = 0x88;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
        adr = 0x05;
        dat = 0xD2;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Enable Buck2.\n");
        adr = 0x06;
        dat = 0x88;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
        adr = 0x07;
        dat = 0xD2;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Enable Buck3.\n");
        adr = 0x08;
        dat = 0x88;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
        adr = 0x09;
        dat = 0xD2;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Set Buck0 to 1.5V.\n");
        adr = 0x0A;
        dat = 0xA2;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Set Buck1 to 3.3V.\n");
        adr = 0x0C;
        dat = 0xFC;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Set Buck2 to 1.75V.\n");
        adr = 0x0E;
        dat = 0xAF;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Set Buck3 to 2.05V.\n");
        adr = 0x10;
        dat = 0xBE;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        printf("PMIC: Clear INT_BUCK_2_3 Status.\n");
        adr = 0x1A;
        dat = 0xFF;
        i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

        busy_wait(1);
    }
}

static void init_vctcxo_dac(void) {
    // Write initial VCTCXO DAC value on firmware boot
    uint8_t i2c_buf[3];
    // uint8_t page_buffer[5] = {0};

    //TODO: FIX this, first FLASH read returns 0xFF. Workaround to read two times...
    FlashQspi_CMD_ReadDataByte(mem_write_offset, &page_buffer[0]);
    FlashQspi_CMD_ReadDataByte(mem_write_offset, &page_buffer[0]);
    FlashQspi_CMD_ReadDataByte(mem_write_offset + 1, &page_buffer[1]);
    printf("FLASH value: 0x%02x: 0x%02x\n", page_buffer[0], page_buffer[1]);

    // Write DAC value stored in flash storage only if it isn't empty (0xFFFF)
    if ((page_buffer[1] == 0xFF) && (page_buffer[0] == 0xFF)) {
        // Write Default value
        dac_val = DAC_DEFF_VAL;
        i2c_buf[0] = 0x30; // cmd
        i2c_buf[1] = (dac_val >> 8) & 0xFF;
        i2c_buf[2] = dac_val & 0xFF;
        printf("Default value\n");
    } else {
        // Write DAC value from FLASH
        dac_val = ((uint16_t) page_buffer[1]) << 8 | ((uint16_t) page_buffer[0]);
        i2c_buf[0] = 0x30; // cmd
        i2c_buf[1] = page_buffer[1];
        i2c_buf[2] = page_buffer[0];
        printf("Value from FLASH\n");
    }

    //TODO: implement value read from non volatile Mem
    //i2c_buf[0] = 0x30; // cmd
    //i2c_buf[1] = (dac_val >> 8) & 0xFF;
    //i2c_buf[2] = dac_val & 0xFF;
    i2c0_write(I2C_DAC_ADDR, i2c_buf[0], &i2c_buf[1], 2);
}


/*-----------------------------------------------------------------------*/
/* LMS7002m status                                                       */
/*-----------------------------------------------------------------------*/

static void lms7002_status(void) {
    printf("TX PLL Lock status = %lx \n", csr_read_simple(pll0_tx_addrs.locked));
    printf("\n");
    printf("RX PLL Lock status = %lx \n", csr_read_simple(pll1_rx_addrs.locked));
}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void) {
    char *str;
    char *token;

    str = readstr();
    if (str == NULL) return;
    token = get_token(&str);
    if (strcmp(token, "help") == 0)
        help();
    else if (strcmp(token, "reboot") == 0)
        reboot_cmd();
    else if (strcmp(token, "i2c_test") == 0)
        i2c_test();
    else if (strcmp(token, "init_pmic") == 0)
        init_pmic();
    else if (strcmp(token, "dump_pmic") == 0)
        dump_pmic();
#ifdef CSR_LEDS_BASE
    else if (strcmp(token, "led") == 0)
        led_cmd();
#endif
#ifdef CSR_BASE
    else if (strcmp(token, "gpioled") == 0)
        gpioled_cmd();
    else if (strcmp(token, "mmap") == 0)
        mmap_cmd();
#endif
#ifdef CSR_BASE
    else if (strcmp(token, "lms") == 0)
        lms7002_status();
#endif
    else if (strcmp(token, "dac_test") == 0)
        dac_test();
    else if (strcmp(token, "flash_test") == 0)
        flash_test();

    prompt();
}

/**
 * Gets 64 bytes packet
 */
/*
void getLMS64Packet(uint8_t *buf, uint8_t k)
{
	uint8_t cnt = 0;
	uint32_t* dest = (uint32_t*)buf;
	for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++)
	{
		busy_wait_us(1);
		dest[cnt] = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
	}

}*/

void getLMS64Packet(uint8_t *buf, uint8_t k);
void getLMS64Packet(uint8_t *buf, uint8_t k) {
    uint8_t cnt = 0;
    uint32_t *dest = (uint32_t *) buf;
    uint32_t temp_buffer[k / sizeof(uint32_t)];
    uint8_t is_stable = 0;

    while (!is_stable) {
        // Read the first buffer into temp_buffer
        for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++) {
            temp_buffer[cnt] = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
        }

        // Read again into dest buffer
        for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++) {
            dest[cnt] = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
        }

        // Compare the two buffers
        is_stable = 1;
        for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++) {
            if (temp_buffer[cnt] != dest[cnt]) {
                is_stable = 0;
                break;
            }
        }
    }
}


static void lms64c_isr(void) {
    lms64_packet_pending = 1;
    CNTRL_ev_pending_write(CNTRL_ev_pending_read()); // Clear interrupt
    CNTRL_ev_enable_write(1 << CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET); // re-enable the event handler
}


static void lms64c_init(void) {
    printf("CNTRL IRQ initialization \n");

    /* Clear all pending interrupts. */
    CNTRL_ev_pending_write(CNTRL_ev_pending_read());

    /* Enable CNTRL irq */
    CNTRL_ev_enable_write(1 << CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET);

    /* Attach isr to interrupt */
    irq_attach(CNTRL_INTERRUPT, lms64c_isr);

    /* Enable interrupt */

    irq_setmask(irq_getmask() | (1 << CNTRL_INTERRUPT));
}

static void clk_ctrl_isr(void) {
    // Reset relevant CSR's
    csr_write_simple(0, clk_ctrl_addrs.pllcfg_done);
    csr_write_simple(0, clk_ctrl_addrs.phcfg_done);
    csr_write_simple(0, clk_ctrl_addrs.pllcfg_error);
    csr_write_simple(0, clk_ctrl_addrs.phcfg_err);

    var_phcfg_start = csr_read_simple(clk_ctrl_addrs.phcfg_start);
    var_pllcfg_start = csr_read_simple(clk_ctrl_addrs.pllcfg_start);
    var_pllrst_start = csr_read_simple(clk_ctrl_addrs.pllrst_start);

    if (var_phcfg_start || var_pllcfg_start || var_pllrst_start)
        clk_cfg_pending = 1;

    lime_top_ev_pending_write(lime_top_ev_pending_read()); //Clear interrupt
    lime_top_ev_enable_write(1 << CSR_LIME_TOP_EV_STATUS_CLK_CTRL_IRQ_OFFSET); // re-enable the event handler
}

static void clk_cfg_irq_init(void) {
    printf("CLK config irq initialization \n");

    /* Clear all pending interrupts. */
    lime_top_ev_pending_write(lime_top_ev_pending_read());

    /* Enable CLK CTRL irq */
    lime_top_ev_enable_write(1 << CSR_LIME_TOP_EV_STATUS_CLK_CTRL_IRQ_OFFSET);

    /* Attach isr to interrupt */
    irq_attach(LIME_TOP_INTERRUPT, clk_ctrl_isr);

    /* Enable interrupt */

    irq_setmask(irq_getmask() | (1 << LIME_TOP_INTERRUPT));
}

void copyArray(unsigned char *source, unsigned char *destination, size_t sourceIndex, size_t destinationIndex,
               size_t count) {
    memcpy(destination + destinationIndex, source + sourceIndex, count);
}

// End of functions specifics to XTRX
#else // LIMESDR_MINI_V1 / LIMESDR_MINI_V2
#ifdef LIMESDR_MINI_V1
/**
 * Bit swap in byte
 */
uint8_t reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}
#endif

static void dac_spi_write(const uint16_t write_data)
{
    // register is 32bits and start sending bit 32
    // we have to shift data to align
    uint32_t cmd = ((uint32_t)write_data) << 16;
    printf("%08lx\n", cmd);

    /* set cs */
    spimaster_cs_write(1 << 1);

    /* Write SPI MOSI Data */
    spimaster_mosi_write(cmd);

    /* Start SPI Xfer */
    spimaster_control_write(
        (1  << CSR_SPIMASTER_CONTROL_START_OFFSET) |
        (16 << CSR_SPIMASTER_CONTROL_LENGTH_OFFSET)
    );

    /* Wait SPI Xfer */
    while ((spimaster_status_read() & (1 << CSR_SPIMASTER_STATUS_DONE_OFFSET)) == 0);
}

/* SPIFlash ------------------------------------------------------------------*/
#if defined(CSR_SPIFLASH_CORE_BASE)
void spiFlash_read(uint32_t rel_addr, uint32_t length, uint8_t *rdata)
{
    void *addr = (void *)SPIFLASH_BASE;
    uint32_t real_len = length;
    int i, ii, data_offset;
    uint32_t rx;
    uint32_t offset = 0;
    // Read access is 32b: must be aligned
    uint32_t base_addr = (rel_addr >> 2) << 2;
    if (base_addr != rel_addr) {
        offset = rel_addr - base_addr;
        real_len++;
    }
    for (i = 0, data_offset = 0; i < real_len; i += 4) {
        rx = *(uint32_t *)(addr + i);
        int max = (data_offset + 4 > length) ? length - data_offset : 4;
        for (ii = offset; ii < max; ii++) {
            rdata[data_offset++] = (rx >> (ii * 8));
        }
        offset = 0;
    }
}
#endif

/* FIFO ----------------------------------------------------------------------*/
int FIFO_loopback_test(int base_addr)
{
    unsigned int fifo_val =0;
    int fifo_wrcnt = 0;
    int fifo_wr_data =0;
    int fifo_rd_cnt =0;
    int fifo_wr_data_array[4];
    int fifo_rd_data_array[4];

    //FIFO testing
    fifo_wrcnt = 0;
    fifo_wr_data = 1;

    //Reset FIFO
    //printf("FIFO reset \n\n");
    ft601_fifo_control_write(1);
    ft601_fifo_control_write(0);
    busy_wait(100);

    while ((ft601_fifo_status_read() & 0x2) != 0x2) {
        ft601_fifo_wdata_write(fifo_wr_data);
        //printf("FIFO Write: %#x \n", fifo_wr_data);
        fifo_wr_data_array[fifo_wrcnt]=fifo_wr_data;
        fifo_wrcnt++;
        fifo_wr_data++;
        //busy_wait(100);

    }
    //printf("FIFO Write counter: %d \n", fifo_wrcnt);

    fifo_rd_cnt=0;
    while (!(ft601_fifo_status_read() & 0x1)) {
        fifo_val = ft601_fifo_rdata_read();
        fifo_rd_data_array[fifo_rd_cnt] = fifo_val;
        //busy_wait(100);
        //printf("FIFO Read: %#x \n", fifo_val);
        fifo_rd_cnt++;
    }
    //printf("FIFO Read counter: %d \n", fifo_rd_cnt);

    int i = 0;
    if (fifo_wrcnt == fifo_rd_cnt) {
        //printf("FIFO WR/RD counters match \n");
        for (i =0; i < fifo_wrcnt; i++){
            if (fifo_wr_data_array[i] != fifo_rd_data_array[i]){
                //printf("FIFO WR/RD arrays does not match \n");
                return(0);
            }
        }
    }
    else {
        //printf("FIFO WR/RD counters does not match \n");
        return(0);
    }

    return(1);
}

/**
 * Gets 64 bytes packet from FIFO.
 */

void getFifoData(uint8_t *buf, uint8_t k);
void getFifoData(uint8_t *buf, uint8_t k)
{
    uint8_t cnt = 0;
    uint32_t* dest = (uint32_t*)buf;
    uint32_t fifo_val = 0;

#ifdef DEBUG_FIFO
    printf("D %d\n", k);
#endif
    for (cnt=0; cnt<k/sizeof(uint32_t); ++cnt)
    {
        fifo_val = ft601_fifo_rdata_read();
#ifdef DEBUG_FIFO
        printf("X%08lx ", fifo_val);
#endif
        dest[cnt] = fifo_val; // Read Data From Fifo
    }
#ifdef DEBUG_FIFO
    printf("\n");
    printf("E\n");
#endif
}

/**
 * Configures LM75
 */
void Configure_LM75(void);
void Configure_LM75(void)
{
    bool spirez;
    unsigned char addr;
    unsigned char wdata[4];
    unsigned char rdata[4];
    (void)spirez;

    // OS polarity configuration
    addr = 0x01; // Pointer = configuration register
    wdata[0] = 0x04; //Configuration value: OS polarity = 1, Comparator/int = 0, Shutdown = 0
    spirez = i2c0_write(LM75_I2C_ADDR, addr, wdata, 1);
    //spirez = i2c0_write(0x60, addr, wdata, 1);
    if (!spirez)
        return;

    // Read  back OS polarity configuration
    addr = 0x01; // Pointer = Configuration register

    spirez = i2c0_read(LM75_I2C_ADDR, addr, rdata, 1, true);
    busy_wait(100);


    // THYST configuration
    addr = 0x02;	// Pointer = THYST register
    wdata[0]=0x2D;	// Set THYST H (45)
    wdata[1]=0;		// Set THYST L
    spirez = i2c0_write(LM75_I2C_ADDR, addr, wdata, 2);


    // Read  back THYST configuration
    addr = 0x02; // Pointer = THYST
    spirez = i2c0_read(LM75_I2C_ADDR, addr, rdata, 2, true);
    busy_wait(100);


    // TOS configuration
    addr = 0x03;	// Pointer = TOS register
    wdata[0]=0x37;	// Set TOS H (55)
    wdata[1]=0;		// Set TOS L
    spirez = i2c0_write(LM75_I2C_ADDR, addr, wdata, 2);


    // Read back TOS configuration
    addr = 0x03; // Pointer = TOS
    spirez = i2c0_read(LM75_I2C_ADDR, addr, rdata, 2, true);
    busy_wait(100);
}

uint16_t rd_dac_val(uint16_t addr)
{
    //uint8_t i2c_error;
    int i2c_error;
    uint8_t addr_lsb = (uint8_t) addr & 0x00FF;
    uint8_t addr_msb = (uint8_t) (addr & 0xFF00) >> 8;
    uint8_t eeprom_rd_val_0;
    uint8_t eeprom_rd_val_1;
    uint16_t rez;
    unsigned char wdata[4];
    unsigned char rdata[4]={0xFF, 0xFF, 0xFF, 0xFF};
    (void) i2c_error;

    /*
    i2c_error = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 0);
    i2c_error = I2C_write(I2C_OPENCORES_0_BASE, addr_msb, 0);
    i2c_error = I2C_write(I2C_OPENCORES_0_BASE, addr_lsb, 0);
    i2c_error = I2C_start(I2C_OPENCORES_0_BASE, EEPROM_I2C_ADDR, 1);
    eeprom_rd_val_0 = I2C_read(I2C_OPENCORES_0_BASE, 0);
    eeprom_rd_val_1 = I2C_read(I2C_OPENCORES_0_BASE, 1);
    */

    wdata[0]=addr_msb;
    wdata[1]=addr_lsb;
    i2c_error = i2c0_write(EEPROM_I2C_ADDR, wdata[0], &wdata[1], 1);
    i2c_error = i2c0_read(EEPROM_I2C_ADDR, wdata[0], rdata, 2, true);
    eeprom_rd_val_0 = rdata[0];
    eeprom_rd_val_1 = rdata[1];

    rez = ((uint16_t)eeprom_rd_val_1 << 8) | eeprom_rd_val_0;
    return rez;
}

#endif // LIMESDR_MINI_V1 / LIMESDR_MINI_V2

/** Checks if peripheral ID is valid.
 Returns 1 if valid, else 0. */
unsigned char Check_Periph_ID(unsigned char max_periph_id, unsigned char Periph_ID);
unsigned char Check_Periph_ID(unsigned char max_periph_id, unsigned char Periph_ID) {
    if (LMS_Ctrl_Packet_Rx->Header.Periph_ID > max_periph_id) {
        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
        return 0;
    } else
        return 1;
}

/**	This function checks if all blocks could fit in data field.
 *	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks(unsigned char block_size);
unsigned char Check_many_blocks(unsigned char block_size) {
    if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field) / block_size)) {
        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
        return 1;
    } else
        return 0;
}

int main(void) {
    int spirez;
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();
    init_pmic();
    printf("CSR_CNTRL_BASE 0x%lx ", CSR_CNTRL_BASE);
    // irq_example_init();
    lms64c_init();
    clk_cfg_irq_init();
    init_pmic();
    init_vctcxo_dac();
    help();
    prompt();

    while (1) {
#ifdef LIMESDR_XTRX
        console_service();
#else
        spirez = ft601_fifo_status_read();	// Read FIFO Status
		lms64_packet_pending = !(spirez & 0x01);
#endif

        // Process received packet
        if (lms64_packet_pending) {
            uint8_t reg_array[4];
            uint16_t addr;
            uint16_t val;
            uint8_t i2c_buf[3];
			uint32_t read_value;

#ifdef LIMESDR_XTRX
            /* Disable CNTRL irq while processing packet */
            CNTRL_ev_enable_write(CNTRL_ev_enable_read() & ~(1 << CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET));
            irq_setmask(irq_getmask() & ~(1 << CNTRL_INTERRUPT));

            lms64_packet_pending = 0;
            // printf("CNTRL PCT GOT!\n");
            uint32_t *dest = (uint32_t *) glEp0Buffer_Tx;

            getLMS64Packet(glEp0Buffer_Rx, 64);
            // printf("RX: ");
            // for (int i = 0; i < 64; i++) {
            //     printf("%02x ", glEp0Buffer_Rx[i]);
            // }
            // printf("\n");
#else
            main_gpo_write(1);

            //Read packet from the FIFO
            getFifoData(glEp0Buffer_Rx, 64);
#endif

            memset(glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); // fill whole tx buffer with zeros
            cmd_errors = 0;

            LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
            LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
            LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

            switch (LMS_Ctrl_Packet_Rx->Header.Command) {
                case CMD_GET_INFO:

#ifdef LIMESDR_XTRX
                    LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
                    LMS_Ctrl_Packet_Tx->Data_field[1] = LMS_DEV_XTRX;
                    LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
#else
#ifdef LIMESDR_MINI_V2
                    LMS_Ctrl_Packet_Tx->Data_field[0] = 10; // FW_VER
                    LMS_Ctrl_Packet_Tx->Data_field[1] = LMS_DEV_MINI_V2; // DEV_TYPE
#else
                    LMS_Ctrl_Packet_Tx->Data_field[0] = 6; // FW_VER
                    LMS_Ctrl_Packet_Tx->Data_field[1] = LMS_DEV_MINI; // DEV_TYPE
#endif
                    LMS_Ctrl_Packet_Tx->Data_field[3] = 0; // HW_VER
#endif
                    LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
                    LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

#ifdef LIMESDR_XTRX
                    // Read Serial number from FLASH OTP region
                    spirez = FlashQspi_CMD_ReadOTPData(OTP_SERIAL_ADDRESS, sizeof(serial), serial);

                    LMS_Ctrl_Packet_Tx->Data_field[10] = serial[7];
                    LMS_Ctrl_Packet_Tx->Data_field[11] = serial[6];
                    LMS_Ctrl_Packet_Tx->Data_field[12] = serial[5];
                    LMS_Ctrl_Packet_Tx->Data_field[13] = serial[4];
                    LMS_Ctrl_Packet_Tx->Data_field[14] = serial[3];
                    LMS_Ctrl_Packet_Tx->Data_field[15] = serial[2];
                    LMS_Ctrl_Packet_Tx->Data_field[16] = serial[1];
                    LMS_Ctrl_Packet_Tx->Data_field[17] = serial[0];
#endif

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

#if defined(LIMESDR_MINI_V1) | defined(LIMESDR_MINI_V2)
                case CMD_GPIO_DIR_WR:
                    printf("CMD_GPIO_DIR_WR\n");
                    //if(Check_many_blocks (2)) break;

                    //write reg addr
                    addr = 0x80;                                          // Write command & BOARD_GPIO_DIR register address MSB
                    addr = (addr << 8) | 0xC4;                            // BOARD_GPIO_DIR register address LSB
                    val = LMS_Ctrl_Packet_Rx->Data_field[0];              // leftmost byte
                    val = (val << 8) | LMS_Ctrl_Packet_Rx->Data_field[1]; // Data fields swapped, while MSB in the data packet is in the
                    //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_FPGA, 4, sc_brdg_data, 0, NULL, 0);
					lms_spi_write(addr, val);

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_GPIO_DIR_RD:
                    printf("CMD_GPIO_DIR_RD\n");
                    //if(Check_many_blocks (2)) break;

                    //write reg addr
                    sc_brdg_data[0] = 0x00;		// Read command & BOARD_GPIO_DIR register address MSB
                    sc_brdg_data[1] = 0xC4;		// BOARD_GPIO_DIR register address LSB
                    //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_FPGA, 2, sc_brdg_data, 2, &sc_brdg_data[2], 0);

                    LMS_Ctrl_Packet_Tx->Data_field[0] = sc_brdg_data[3];	// Data fields swapped, while MSB in the data packet is in the
                    LMS_Ctrl_Packet_Tx->Data_field[1] = sc_brdg_data[2];	// leftmost byte

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;


                case CMD_GPIO_WR:
                    printf("CMD_GPIO_WR\n");
                    //if(Check_many_blocks (2)) break;

                    //write reg addr
                    sc_brdg_data[0] = 0x80;                              // Write command & BOARD_GPIO_VAL register address MSB
                    sc_brdg_data[1] = 0xC6;                              // BOARD_GPIO_VAL register address LSB
                    sc_brdg_data[2] = LMS_Ctrl_Packet_Rx->Data_field[1]; // Data fields swapped, while MSB in the data packet is in the
                    sc_brdg_data[3] = LMS_Ctrl_Packet_Rx->Data_field[0]; // leftmost byte
                    //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_FPGA, 4, sc_brdg_data, 0, NULL, 0);

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_GPIO_RD:
                    printf("CMD_GPIO_RD\n");
                    //if(Check_many_blocks (2)) break;

                    //write reg addr
                    sc_brdg_data[0] = 0x00;		// Read command & BOARD_GPIO_RD register address MSB
                    sc_brdg_data[1] = 0xC2;		// BOARD_GPIO_RD register address LSB
                    //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_NR_FPGA, 2, sc_brdg_data, 2, &sc_brdg_data[2], 0);

                    LMS_Ctrl_Packet_Tx->Data_field[0] = sc_brdg_data[3];	// Data fields swapped, while MSB in the data packet is in the
                    LMS_Ctrl_Packet_Tx->Data_field[1] = sc_brdg_data[2];	// leftmost byte

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;
#endif

#ifdef LIMESDR_XTRX
                case CMD_SERIAL_WR:

                    copyArray(LMS_Ctrl_Packet_Rx->Data_field, tmp_serial, 24, 0, 32);

                    // STORAGE_TYPE
                    switch (LMS_Ctrl_Packet_Rx->Data_field[0]) {
                        case 0: //Default
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            break;
                        case 1: //Volatile memory
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            break;
                        case 2: //Non-Volatile memory
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            break;
                        case 3: //Non-Volatile OTP memory
                            if (serial_otp_unlock_key == OTP_UNLOCK_KEY) {
                                //FlashQspi_EraseSector(&CFG_QSPI, OTP_SERIAL_ADDRESS); //temp for testing
                                spirez = FlashQspi_ProgramOTP(
                                    OTP_SERIAL_ADDRESS, LMS_Ctrl_Packet_Rx->Data_field[1], tmp_serial);
                                serial_otp_unlock_key = 0;
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                            } else if (serial_otp_unlock_key != OTP_UNLOCK_KEY && LMS_Ctrl_Packet_Rx->Data_field[2] ==
                                       OTP_UNLOCK_KEY) {
                                serial_otp_unlock_key = LMS_Ctrl_Packet_Rx->Data_field[2];
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                            } else {
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_RESOURCE_DENIED_CMD;
                            }
                            break;
                        default:
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            break;
                    }

                    break;

                case CMD_SERIAL_RD:
                    spirez = FlashQspi_CMD_ReadOTPData(OTP_SERIAL_ADDRESS, 32, tmprd_serial);
                    copyArray(tmprd_serial, LMS_Ctrl_Packet_Tx->Data_field, 0, 24, 32);
                    LMS_Ctrl_Packet_Tx->Data_field[1] = 16;
                    LMS_Ctrl_Packet_Tx->Data_field[2] = serial_otp_unlock_key;
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;
#endif

                case CMD_LMS_RST:
                    printf("CMD_LMS_RST\n");

                    if (!Check_Periph_ID(MAX_ID_LMS7, LMS_Ctrl_Packet_Rx->Header.Periph_ID))
                        break;

                    switch (LMS_Ctrl_Packet_Rx->Data_field[0]) {
                        case LMS_RST_DEACTIVATE:
#ifdef LIMESDR_XTRX
                            //Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); // high level
                            //printf("LMS RESET deactivate...\n");
#else
                            lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
#endif
                            break;
                        case LMS_RST_ACTIVATE:
#ifdef LIMESDR_XTRX
                            //Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); // low level
                            //printf("LMS RESET activate...\n");
#else
                            lms7002_top_lms_ctr_gpio_write(0x0);
#endif
                            break;

                        case LMS_RST_PULSE:
#ifdef LIMESDR_XTRX
                            //Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); // low level
                            //Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); // high level
                            read_value = lms7002_top_lms1_read() & ~(1 << CSR_LMS7002_TOP_LMS1_RESET_OFFSET);
                            lms7002_top_lms1_write(read_value);
                            read_value |= (1 << CSR_LMS7002_TOP_LMS1_RESET_OFFSET);
                            lms7002_top_lms1_write(read_value);
                        //printf("LMS RST pulse...\n");
#else
                            lms7002_top_lms_ctr_gpio_write(0x0);
                            asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
                            asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
                            lms7002_top_lms_ctr_gpio_write(0xFFFFFFFF);
#endif
                            break;
                        default:
                            cmd_errors++;
                            break;
                    }
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_BRDSPI16_WR:
#ifdef DEBUG_CSR_ACCESS
                    printf("CMD_BRDSPI16_WR\n");
#endif
                    if (Check_many_blocks(4))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        // write reg addr
                        //sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
                        // Clearing write bit in address field because we are not using SPI registers in LiteX implementation
                        cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // clear write bit
#ifdef LIMESDR_XTRX

                        writeCSR(&LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)],
                                 &LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
#else
                        uint16_t addr = (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)] << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)];
#ifdef DEBUG_CSR_ACCESS
                        printf("csr write @ %04d: %02x%02x\n",addr,
                                LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)],
                                LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)]);
#endif
                        if (addr < 32) {
                            fpgacfg_write(addr & 0x1f, &LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
                        } else if (addr < 96) {
                            pllcfg_write(addr & 0x1f, &LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
                        } else if (addr < 192) {
                            tstcfg_write(addr & 0x1f, &LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
                        } else if (addr < 192+32) {
                            periphcfg_write(addr & 0x1f, &LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
                        } else {
                            printf("write error : %04d %04x\n", addr, addr);
                        }
#endif
                    }
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_BRDSPI16_RD:
#ifdef DEBUG_CSR_ACCESS
                    printf("CMD_BRDSPI16_RD\n");
#endif
                    if (Check_many_blocks(4))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        // write reg addr
                        cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit

#ifdef LIMESDR_XTRX
                        readCSR(&LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], reg_array);
#else // LIMESDR_MINI_V1, LIMESDR_MINI_V2
                        uint16_t addr = (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)] << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];
                        uint8_t reg_array[2];
                        if (addr < 32) {
                            fpgacfg_read(addr & 0x1f, reg_array);
                        } else if (addr < 96) {
                            pllcfg_read(addr & 0x1f, reg_array);
                        } else if (addr < 192) {
                            tstcfg_read(addr & 0x1f, reg_array);
                        } else if (addr < 192+32) {
                            periphcfg_read(addr & 0x1f, reg_array);
                        } else {
                            printf("read error\n");
                        }
#endif
                        LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = reg_array[1];
                        LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = reg_array[0];
#ifdef DEBUG_CSR_ACCESS
                        printf("csr read @ %04d: %02x%02x\n",addr, reg_array[1], reg_array[0]);
#endif

                        //			printf("value: 0x%X\n", reg_array[0]);
                        //			printf("value: 0x%X\n", reg_array[1]);
                    }

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                // COMMAND LMS WRITE

                case CMD_LMS7002_WR:
                    if (!Check_Periph_ID(MAX_ID_LMS7, LMS_Ctrl_Packet_Rx->Header.Periph_ID))
                        break;
                    if (Check_many_blocks(4))
                        break;


                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        // write reg addr
                        sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
                        // Parse address
                        addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)];
                        addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)];
                        // Parse value
                        val = LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)];
                        val = (val << 8) | LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
                        // Write
                        lms_spi_write(addr, val);
#ifdef DEBUG_LMS_SPI
                        printf("%04x %04x\n", addr, val);
                        cbi(addr, 15);
                        val = lms_spi_read(addr);
                        printf(" %04x %04x\n", addr, val);
#endif
                    }

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                // COMMAND LMS READ

                case CMD_LMS7002_RD:
                    if (Check_many_blocks(4))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        //write reg addr
                        cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit
                        // Parse address
                        addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)];
                        addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];
                        // Read
                        val = lms_spi_read(addr);
                        // Return value and address
                        LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[
                            0 + (block * 2)];
                        LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[
                            1 + (block * 2)];
                        LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (val >> 8) & 0xFF;
                        LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = val & 0xFF;
#ifdef DEBUG_LMS_SPI
                       printf("%04x %04x\n", addr, val);
#endif
                    }

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

#if defined(CSR_SPIFLASH_CORE_BASE) || defined(INTERNAL_FLASH_BASE)
                case CMD_ALTERA_FPGA_GW_WR: // FPGA active serial

                    current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (
                                          LMS_Ctrl_Packet_Rx->Data_field[2] << 16)
                                      | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
                    data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];
#ifdef LIMESDR_XTRX

                    switch (LMS_Ctrl_Packet_Rx->Data_field[0]) // prog_mode
                    {
                        /*
                        Programming mode:
                        0 - Bitstream to FPGA
                        1 - Bitstream to Flash
                        2 - Bitstream from FLASH
                        3 - Golden image to Flash
                        4 - User image to Flash
                        */
                        case 1:
                        case 3:
                        case 4:
                            // write data to Flash from PC
                            // Start of programming? reset variables
                            if (current_portion == 0) {
                                // Gold image must be written at address 0x0
                                if (LMS_Ctrl_Packet_Rx->Data_field[0] == 3) {
                                    address = 0;
                                    printf("g\n");
                                } else {
                                    // User image must be written at offset
                                    address = 0x220000;
                                    printf("u\n");
                                }

                                page_buffer_cnt = 0;
                                total_data = 0;
                                // Erase first sector
                                FlashQspi_EraseSector(address);
                            }

                            inc_data_count = LMS_Ctrl_Packet_Rx->Data_field[5];

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
                                    memcpy(&page_buffer[page_buffer_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24],
                                           data_to_copy);
                                    // We already know the page is full because of overflowing input
                                    FlashQspi_ProgramPage(address, page_buffer);
                                    address += 256;
                                    total_data += 256;
                                    // Check if new address is bottom of sector, erase if needed
                                    if ((address & 0xFFF) == 0)
                                        FlashQspi_EraseSector(address);
                                    memcpy(&page_buffer[0], &LMS_Ctrl_Packet_Rx->Data_field[24 + data_to_copy],
                                           data_leftover);
                                    page_buffer_cnt = data_leftover;
                                } else {
                                    // Incoming data would not overflow the page buffer
                                    memcpy(&page_buffer[page_buffer_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24],
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
                        // No error conditions present in code
                        // if (spirez == XST_SUCCESS)
                            if (true) {
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                            } else {
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            }
                            break;

                        default:
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            break;
                    }

                    break;
#else

                    switch (LMS_Ctrl_Packet_Rx->Data_field[0]) // prog_mode
                    {
                        /*
                        Programming mode:
                        0 - Bitstream to FPGA
                        1 - Bitstream to Flash
                        2 - Bitstream from FLASH
                        */

                        case 0://Bitstream to FPGA from PC
                            /*
                                if ( Configure_FPGA (&LMS_Ctrl_Packet_Rx->Data_field[24], current_portion, data_cnt) ) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                                else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                             */
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

                            break;

                        case 1: //write data to Flash from PC

                            current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
                            data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

                            if (current_portion == 0) state = 10;
                            if (data_cnt        == 0)
                            {
                                state = 30;
                            }
                            Flash = 1;

                            while(Flash) {
                                switch (state)
                                {
                                //Init
                                case 10:
                                //Set Flash memory addresses
#ifdef LIMESDR_MINI_V1
                                    address = UFMStartAddress;
                                    //Write Control Register of On-Chip Flash IP to un-protect and erase operation
                                    //wishbone_write32(ONCHIP_FLASH_0_CSR_BASE + (1<<2), 0xf67fffff);
                                    //wishbone_write32(ONCHIP_FLASH_0_CSR_BASE + (1<<2), 0xf65fffff);
                                    internal_flash_control_register_write(0xf67fffff);
                                    internal_flash_control_register_write(0xf65fffff);
#else
                                    address = CFM0StartAddress;

                                    //spiflash_erase_primary(spiflash);
                                    // Erase First 64KB block, other blocks are erased later
                                    //flash_op_status = MicoSPIFlash_BlockErase(spiflash, spiflash->memory_base + 0x00000000, 3);
                                    if (spiflash_erase(0x00000000) == false)
                                        printf("spiflash_erase_primary: Error\n");
#endif

                                    state = 11;
                                    Flash = 1;

                                case 11:
                                    //Start erase CFM0
#ifdef LIMESDR_MINI_V1
                                    if((internal_flash_status_register_read() & 0x13) == 0x10)
                                    {
                                        internal_flash_control_register_write(0xf67fffff);
                                        printf("CFM0 Erased\n");
                                        state = 13;
                                        Flash = 1;
                                    }
                                    if((internal_flash_status_register_read() & 0x13) == 0x01)
                                    {
                                        printf("Erasing CFM0\n");
                                        state = 11;
                                        Flash = 1;
                                    }
                                    if((internal_flash_status_register_read() & 0x13) == 0x00)
                                    {
                                        printf("Erase CFM0 Failed\n");
                                        state = 0;
                                    }
#else
                                    //if ((0x03 & MicoSPIFlash_StatusRead (spiflash)) == 0)
                                    if ((0x03 & spiflash_read_status_register()) == 0)
                                    {
                                        //printf("CFM0 Erased\n");
                                        //printf("Enter Programming file.\n");
                                        state = 20;
                                        Flash = 1;
                                    }
                                    if((0x01 & spiflash_read_status_register()) == 0x01)
                                    {
                                        //printf("Erasing CFM0\n");
                                        state = 11;
                                        Flash = 1;
                                    }
                                    if((0x02 & spiflash_read_status_register()) == 0x02)
                                    {
                                        //printf("Erase CFM0 Failed\n");
                                        state = 0;
                                    }
#endif

                                    break;

#ifdef LIMESDR_MINI_V1
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
                                    if((internal_flash_status_register_read() & 0x13) == 0x10)
                                    {
                                        internal_flash_control_register_write(0xf67fffff);
                                        //printf("UFM ID1 Erased\n");
                                        state = 16;
                                        Flash = 1;
                                    }
                                    if((internal_flash_status_register_read() & 0x13) == 0x01)
                                    {
                                        //printf("Erasing UFM ID1\n");
                                        state = 14;
                                        Flash = 1;
                                    }
                                    if((internal_flash_status_register_read() & 0x13) == 0x00)
                                    {
                                        //printf("Erase UFM ID1 Failed\n");
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
                                    if((internal_flash_status_register_read() & 0x13) == 0x10)
                                    {
                                        internal_flash_control_register_write(0xf67fffff);
                                        printf("UFM ID2 Erased\n");
                                        state = 20;
                                        Flash = 1;
                                    }
                                    if((internal_flash_status_register_read() & 0x13) == 0x01)
                                    {
                                        printf("Erasing UFM ID2\n");
                                        state = 17;
                                        Flash = 1;
                                    }
                                    if((internal_flash_status_register_read() & 0x13) == 0x00)
                                    {
                                        printf("Erase UFM ID2 Failed\n");
                                        state = 0;
                                    }
                                    break;
#endif

                                //Program
                                case 20:
                                    for (byte = 24; byte <= 52; byte += 4)
                                    {
#ifdef LIMESDR_MINI_V1
                                //Take word and swap bits
                                word  = ((uint32_t)reverse(LMS_Ctrl_Packet_Rx->Data_field[byte+0]) << 24) & 0xFF000000;
                                word |= ((uint32_t)reverse(LMS_Ctrl_Packet_Rx->Data_field[byte+1]) << 16) & 0x00FF0000;
                                word |= ((uint32_t)reverse(LMS_Ctrl_Packet_Rx->Data_field[byte+2]) <<  8) & 0x0000FF00;
                                word |= ((uint32_t)reverse(LMS_Ctrl_Packet_Rx->Data_field[byte+3]) <<  0) & 0x000000FF;
#else
                                //Take word
                                p_spi_wrdata[0] = LMS_Ctrl_Packet_Rx->Data_field[byte+0];
                                p_spi_wrdata[1] = LMS_Ctrl_Packet_Rx->Data_field[byte+1];
                                p_spi_wrdata[2] = LMS_Ctrl_Packet_Rx->Data_field[byte+2];
                                p_spi_wrdata[3] = LMS_Ctrl_Packet_Rx->Data_field[byte+3];
#endif

                                //Command to write into On-Chip Flash IP
                                if(address <= CFM0EndAddress)
                                {
#ifdef LIMESDR_MINI_V1
                                    *(uint32_t *)(INTERNAL_FLASH_BASE + (address << 2)) = word;
                                    //wishbone_write32(ONCHIP_FLASH_0_DATA_BASE + (address<<2), word);

                                    while((internal_flash_status_register_read() & 0x0b) == 0x02) {
                                        //printf("Writing CFM0(%d)\n", address);
                                    }

                                    if((internal_flash_status_register_read() & 0x0b) == 0x00)
                                    {
                                        printf("Write to addr failed\n");
                                        state = 0;
                                        address = 700000;
                                    }

                                    if((internal_flash_status_register_read() & 0x0b) == 0x08)
                                    {
                                    };

                                    // Increment address or move to CFM0 sector
                                    if (address == UFMEndAddress) address = CFM0StartAddress;
                                    else address += 1;
#else
                                    // Erase Block if we reach starting address of 64KB block
                                    if (address % FLASH_BLOCK_SIZE == 0) {
                                        //flash_op_status = MicoSPIFlash_BlockErase(spiflash, spiflash->memory_base+address, 3);
                                        spiflash_erase(address);
                                    }

                                    //IOWR_32DIRECT(ONCHIP_FLASH_0_DATA_BASE, address, word);
                                    //flash_op_status = MicoSPIFlash_AlignedPageProgram(spiflash, spiflash->memory_base+address, 0x4, wdata);
                                    spiflash_page_program(address, p_spi_wrdata, 0x04);

                                    address += 4;


                                    while((0x01 & spiflash_read_status_register()) == 0x01) {
                                        //printf("Writing CFM0(%d)\n", address);
                                    }
                                    //TODO: Do we need this?
                                    if((0x02 & spiflash_read_status_register()) == 0x02)
                                    {
                                        //printf("Write to %d failed\n", address);
                                        state = 0;
                                        address = 700000;
                                    }
                           /*
                                    if((IORD(ONCHIP_FLASH_0_CSR_BASE, 0) & 0x0b) == 0x08)
                                    {
                                    };
                           */
#endif
                                }
                                else
                                {
                                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                                };
                            };

                            state = 20;
                            Flash = 0;
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

                            break;

                            //Finish
                        case 30:
                            //Re-protect the sector
                            //IOWR(ONCHIP_FLASH_0_CSR_BASE, 1, 0xffffffff);
#ifdef LIMESDR_MINI_V1
                            internal_flash_control_register_write(0xffffffff);
#endif

                            state = 0;
                            Flash = 0;

                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

                            break;

                        default:
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            state = 0;
                            Flash = 0;
                        };
                    };

                    break;

                case 2: //configure FPGA from flash

                    //enable boot to factory image, booting is executed after response to command is sent
                    boot_img_en = 1;

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;



                    break;

                default:
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                }
                break;
#endif
#endif

                // COMMAND ANALOG VALUE READ
                case CMD_ANALOG_VAL_RD:
                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block)]) // ch
                        {
                            case 0: // dac val
#ifdef LIMESDR_XTRX
                                //XIic_Recv(XPAR_I2C_CORES_I2C1_BASEADDR, I2C_DAC_ADDR, i2c_buf, 2, XIIC_STOP);
                                //i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
                                i2c0_read(I2C_DAC_ADDR, 0x0, i2c_buf, 2, true);
#else
                                i2c_buf[0] = (dac_val >> 8) & 0xFF;
                                i2c_buf[1] = dac_val & 0xff;
#endif
                                LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block];
                            // ch
                                LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x00; // RAW //unit, power
                                LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = i2c_buf[0]; // unsigned val, MSB byte
                                LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = i2c_buf[1]; // unsigned val, LSB byte
                            // Storing volatile DAC value
                                dac_val = ((uint16_t) i2c_buf[0]) << 8 | ((uint16_t) i2c_buf[1]);

                                break;

                            case 1: // temperature
#ifdef LIMESDR_XTRX
                                //						i2c_buf[0] = 1;
                                //						i2c_buf[1] = 0x60;
                                //						i2c_buf[2] = 0xA0;
                                // TMP1075 sensor performs periodical temperature readings by default
                                // we only need to read the most recent value
                                i2c_buf[0] = 0;
                            //XIic_Send(XPAR_I2C_CORES_I2C1_BASEADDR,I2C_TERMO_ADDR,i2c_buf,1,XIIC_REPEATED_START);
                            //XIic_Recv(XPAR_I2C_CORES_I2C1_BASEADDR,I2C_TERMO_ADDR,i2c_buf,2,XIIC_STOP);
                                i2c0_read(I2C_TERMO_ADDR, i2c_buf[0], &i2c_buf[0], 2, false);

                                int16_t converted_value = i2c_buf[1] | (i2c_buf[0] << 8);

                                converted_value = converted_value >> 4;
                                converted_value = converted_value * 10;
                                converted_value = converted_value >> 4;
#else
                                i2c_wdata[0]=0x00; // Pointer = temperature register
                                // Read temperature and recalculate
                                i2c0_read(LM75_I2C_ADDR, i2c_wdata[0], i2c_rdata, 2, true);

                                converted_value = (signed short int)i2c_rdata[0];
                                converted_value = converted_value << 8;
                                converted_value = 10 * (converted_value >> 8);
                                spirez = i2c_rdata[1];
                                if(spirez & 0x80) converted_value = converted_value + 5;
#endif


                                LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block];
                            //ch
                                LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x50; //0.1C //unit, power

                                LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (uint8_t) (
                                    (converted_value >> 8) & 0xFF); //signed val, MSB byte
                                LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = (uint8_t) (converted_value & 0xFF);
                            //signed val, LSB byte

                                break;
                            default:
                                cmd_errors++;
                                break;
                        }
                    }

                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                // COMMAND ANALOG VALUE WRITE
                    break;

                case CMD_ANALOG_VAL_WR:
                    if (Check_many_blocks(4))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)]) // do something according to channel
                        {
                            case 0: // TCXO DAC
                                if (LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)] == 0) // RAW units?
                                {
#ifdef LIMESDR_XTRX
                                    i2c_buf[0] = 0x30; // addr
                                    i2c_buf[1] = LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]; // MSB
                                    i2c_buf[2] = LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)]; // LSB
                                    // Storing volatile DAC value
                                    dac_val = ((uint16_t) i2c_buf[1]) << 8 | ((uint16_t) i2c_buf[2]);
                                    // Writing to DAC
                                    //XIic_Send(XPAR_I2C_CORES_I2C1_BASEADDR, I2C_DAC_ADDR, i2c_buf, 3, XIIC_STOP);
                                    i2c0_write(I2C_DAC_ADDR, i2c_buf[0], &i2c_buf[1], 2);
#else // LIMESDR_MINI_V1, LIMESDR_MINI_V2
                                    dac_val = (LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)] << 8 ) + LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
                                    // Write data to the 10bit DAC
                                    dac_data[0] = (unsigned char) ((dac_val & 0x03F0) >> 4); //POWER-DOWN MODE = NORMAL OPERATION (MSB bits =00) + MSB data
                                    dac_data[1] = (unsigned char) ((dac_val & 0x000F) << 4); //LSB data

                                    dac_spi_wrdata = ((unsigned int) dac_data[0]<<8)| ((unsigned int) dac_data[1]) ;
                                    printf("CMD_ANALOG_VAL_WR: %04x\n", dac_spi_wrdata);
                                    dac_spi_write(dac_spi_wrdata);
#endif
                                } else
                                    cmd_errors++;
                                break;
                            default:
                                cmd_errors++;
                                break;
                        }
                    }
                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

#if defined(LIMESDR_MINI_V1) | defined(LIMESDR_MINI_V2)
                case CMD_LMS_MCU_FW_WR:
                    printf("CMD_LMS_MCU_FW_WR\n");
                    current_portion = LMS_Ctrl_Packet_Rx->Data_field[1];

                    //check if portions are send in correct order
                    if(current_portion != 0) { //not first portion?
                        if(last_portion != (current_portion - 1)) { //portion number increments?
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_WRONG_ORDER_CMD;
                            printf("Error 1\n");
                            break;
                        }
                    }

                    //**ZT Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

                    if (current_portion == 0) { //PORTION_NR = first fifo
                        //reset mcu
                        //write reg addr - mSPI_REG2 (Controls MCU input pins)
                        addr = (0x80); //reg addr MSB with write bit
                        addr = (addr << 8) | (MCU_CONTROL_REG); //reg addr LSB

                        val = 0x00; //reg data MSB
                        val = (val << 8) | 0x00; //reg data LSB //8
                        //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
                        //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 4, &sc_brdg_data[0], 0, NULL, 0);
						lms_spi_write(addr, val);

                        //set mode
                        //write reg addr - mSPI_REG2 (Controls MCU input pins)
                        addr = (0x80);                          //reg addr MSB with write bit
                        addr = (addr << 8) | (MCU_CONTROL_REG); //reg addr LSB

                        val = 0x00;            //reg data MSB

                        //reg data LSB
                        switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
                        {
                            case PROG_EEPROM:
                                val = (val << 8) | (0x01); //Programming both EEPROM and SRAM  //8
                                //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
                                //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 4, &sc_brdg_data[0], 0, NULL, 0);
								lms_spi_write(addr, val);
                                break;

                            case PROG_SRAM:
                                val = (val << 8) | (0x02); //Programming only SRAM  //8
                                //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
                                //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 4, &sc_brdg_data[0], 0, NULL, 0);
								lms_spi_write(addr, val);
                                break;


                            case BOOT_MCU:
                                val = (val << 8) | (0x03); //Programming both EEPROM and SRAM  //8
                                //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
                                //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 4, &sc_brdg_data[0], 0, NULL, 0);
								lms_spi_write(addr, val);

                                /*sbi (PORTB, SAEN); //Disable LMS's SPI
                                cbi (PORTB, SAEN); //Enable LMS's SPI*/

                                //spi read
                                //write reg addr
                                addr = (0x00);                         //reg addr MSB
                                addr = (addr << 8) | (MCU_STATUS_REG); //reg addr LSB
                                //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
                                //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 0, NULL, 0);

                                //read reg data
                                //**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
                                //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);
                                val = lms_spi_read(addr);

                                goto BOOTING;

                                break;
                        }
                    }

                    MCU_retries = 0;

                    //wait till EMPTY_WRITE_BUFF = 1
                    while (MCU_retries < MAX_MCU_RETRIES) {
                        //read status reg

                        //spi read
                        //write reg addr
                        addr = (0x00);                         //reg addr MSB
                        addr = (addr << 8) | (MCU_STATUS_REG); //reg addr LSB
                        //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
                        //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 0, NULL, 0);

                        //read reg data
                        //**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
                        //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);
						val = lms_spi_read(addr);
                        printf("%08x\n", val);

                        if (val &0x01) break; //EMPTY_WRITE_BUFF = 1

                        MCU_retries++;
                        //usleep (30);
                        cdelay(3000);
                    }

                    //write 32 bytes to FIFO
                    for (block = 0; block < 32; block++)
                    {
                        /*
                        //wait till EMPTY_WRITE_BUFF = 1
                        while (MCU_retries < MAX_MCU_RETRIES) {
                            //read status reg

                            //spi read
                            //write reg addr
                            SPI_SendByte(0x00); //reg addr MSB
                            SPI_SendByte(MCU_STATUS_REG); //reg addr LSB

                            //read reg data
                            SPI_TransferByte(0x00); //reg data MSB
                            temp_status = SPI_TransferByte(0x00); //reg data LSB

                            if (temp_status &0x01) break;

                            MCU_retries++;
                            Delay_us (30);
                        }*/

                        //write reg addr - mSPI_REG4 (Writes one byte of data to MCU  )
                        addr = (0x80);                          //reg addr MSB with write bit
                        addr = (addr << 8) | (MCU_FIFO_WR_REG); //reg addr LSB

                        val = 0x00;                                                     //reg data MSB
                        val = (val << 8) | (LMS_Ctrl_Packet_Rx->Data_field[2 + block]); //reg data LSB //8

                        //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 4);
                        //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 4, &sc_brdg_data[0], 0, NULL, 0);
						lms_spi_write(addr, val);

                        MCU_retries = 0;
                    }

                    /*sbi (PORTB, SAEN); //Enable LMS's SPI
                    cbi (PORTB, SAEN); //Enable LMS's SPI*/


                    MCU_retries = 0;

                    //wait till EMPTY_WRITE_BUFF = 1
                    while (MCU_retries < 500) {
                        //read status reg

                        //spi read
                        //write reg addr
                        addr = (0x00);                         //reg addr MSB
                        addr = (addr << 8) | (MCU_STATUS_REG); //reg addr LSB
                        //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
                        //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 0, NULL, 0);

                        //read reg data
                        //**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
                        //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);
						val = lms_spi_read(addr);
                        //printf("%08x\n", val);

                        if (val &0x01) break; //EMPTY_WRITE_BUFF = 1

                        MCU_retries++;
                        //usleep (30);
                        cdelay(3000);
                    }


                    if (current_portion  == 255) //PORTION_NR = last fifo
                    {
                        //chek programmed bit

                        MCU_retries = 0;

                        //wait till PROGRAMMED = 1
                        while (MCU_retries < MAX_MCU_RETRIES) {
                            //read status reg

                            //spi read
                            //write reg addr
                            addr = (0x00); //reg addr MSB
                            addr = (addr << 8) | (MCU_STATUS_REG); //reg addr LSB
                            //**ZT CyU3PSpiTransmitWords (&sc_brdg_data[0], 2);
                            //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 0, NULL, 0);

                            //read reg data
                            //**ZT CyU3PSpiReceiveWords (&sc_brdg_data[0], 2); //reg data
                            //spirez = alt_avalon_spi_command(FPGA_SPI_BASE, SPI_LMS7002_SELECT, 2, &sc_brdg_data[0], 2, &sc_brdg_data[0], 0);
							val = lms_spi_read(addr);
                            //printf("%08x\n", val);

                            if (val &0x40) break; //PROGRAMMED = 1

                            MCU_retries++;
                            //usleep (30);
                            cdelay(30000);
                        }

                        if (MCU_retries == MAX_MCU_RETRIES) cmd_errors++;
                    }

                    last_portion = current_portion; //save last portion number

                    BOOTING:

                    if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

                    //**ZT Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

                    break;
#endif

                case CMD_MEMORY_WR:
                    printf("CMD_MEMORY_WR\n");
#ifdef LIMESDR_XTRX
                    // Since the XTRX board does not have an eeprom to store permanent VCTCXO DAC value
                    // a workaround is implemented that uses a sufficiently high address in the configuration flash
                    // to store the DAC value
                    // Since to write data to a flash, a whole sector needs to be erased, additional checks are included
                    // to make sure this function is used ONLY to store VCTCXO DAC value
                    data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

                    if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3))
                    // TARGET = 3 (EEPROM)
                    {
                        // Since the XTRX board does not have an eeprom to store permanent VCTCXO DAC value
                        // a workaround is implemented that uses a sufficiently high address in the configuration flash
                        // to store the DAC value
                        // Since to write data to a flash, a whole sector needs to be erased, additional checks are included
                        // to make sure this function is used ONLY to store VCTCXO DAC value

                        // Check if the user is trying to store VCTCXO DAC value, return error otherwise
                        if (data_cnt == 2 && LMS_Ctrl_Packet_Rx->Data_field[8] == 0 && LMS_Ctrl_Packet_Rx->Data_field[9]
                            == 16) {
                            if (LMS_Ctrl_Packet_Rx->Data_field[0] == 0) // write data to EEPROM #1
                            {
                                LMS_Ctrl_Packet_Rx->Data_field[22] = LMS_Ctrl_Packet_Rx->Data_field[8];
                                LMS_Ctrl_Packet_Rx->Data_field[23] = LMS_Ctrl_Packet_Rx->Data_field[9];
                                FlashQspi_CMD_WREN();
                                FlashQspi_CMD_SectorErase(mem_write_offset);
                                page_buffer[0] = LMS_Ctrl_Packet_Rx->Data_field[24];
                                page_buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[25];
                                //spirez = spirez || FlashQspi_ProgramPage(&CFG_QSPI, mem_write_offset, page_buffer);
                                //cmd_errors = cmd_errors + spirez;
                                FlashQspi_CMD_WREN();
                                FlashQspi_CMD_PageProgramByte(mem_write_offset, &page_buffer[0]);
                                FlashQspi_CMD_WREN();
                                FlashQspi_CMD_PageProgramByte(mem_write_offset + 1, &page_buffer[1]);

                                cmd_errors = 0;
                                if (cmd_errors)
                                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                                else
                                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                            } else
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                        } else
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    } else if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 2))
                    // TARGET = 2 (FPGA FLASH)
                    {
                        // printf("F \n");
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    } else {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                        // printf("N \n");
                    }
#else
#if 0
                    current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
                    data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

                    if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3))
                    // TARGET = 3 (EEPROM)
                    {
                        if(LMS_Ctrl_Packet_Rx->Data_field[0] == 0) //write data to EEPROM #1
                        {
                            i2c_wdata[0]= LMS_Ctrl_Packet_Rx->Data_field[8];
                            i2c_wdata[1]= LMS_Ctrl_Packet_Rx->Data_field[9];

                            for (k=0; k<data_cnt; k++) {
                                i2c_wdata[k+2]= LMS_Ctrl_Packet_Rx->Data_field[24+k];
                            }
                            i2crez = OpenCoresI2CMasterWrite(i2c_master, EEPROM_I2C_ADDR, data_cnt+2, i2c_wdata);
                            OpenCoresI2CMasterStop(i2c_master);
                            MicoSleepMilliSecs(5);

                            if(i2crez) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        }
                        else
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    }
                    else
                    {

                        if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 1)) // TARGET = 1 (FX3)
                        {
                            switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
                            {

                                case 2: //PROG_MODE = 2 (write FW to flash). Note please, that this command is used just to program XO DAC value

                                    flash_page = (LMS_Ctrl_Packet_Rx->Data_field[6] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[7] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[8] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[9]);

                                    if (flash_page >= FLASH_USRSEC_START_ADDR) {
                                        if (flash_page % FLASH_BLOCK_SIZE == 0 && data_cnt > 0) {
                                            flash_op_status = MicoSPIFlash_BlockErase(spiflash, spiflash->memory_base+flash_page, 3);
                                        }

                                        for (k=0; k<data_cnt; k++) {
                                            wdata[k] = LMS_Ctrl_Packet_Rx->Data_field[24+k];
                                        }

                                        if (data_cnt > 0) {
                                            if(MicoSPIFlash_PageProgram(spiflash, spiflash->memory_base+flash_page, (unsigned int)data_cnt, wdata)!= 0) cmd_errors++;
                                        }
                                        if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                                        else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

                                        break;
                                    }
                                    else {
                                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                                        break;
                                    }


                                default:
                                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                                    break;
                            }
                        } else {
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    }
#endif
#endif

                    break;

                case CMD_MEMORY_RD:
                    printf("CMD_MEMORY_RD\n");
#ifdef LIMESDR_XTRX
                    // Since the XTRX board does not have an eeprom to store permanent VCTCXO DAC value
                    // a workaround is implemented that uses a sufficiently high address in the configuration flash
                    // to store the DAC value
                    // Since to write data to a flash, a whole sector needs to be erased, additional checks are included
                    // to make sure this function is used ONLY to store VCTCXO DAC value
                    data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

                    if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3))
                    /// TARGET = 3 (EEPROM)
                    {
                        if (data_cnt == 2 || LMS_Ctrl_Packet_Rx->Data_field[8] == 0 || LMS_Ctrl_Packet_Rx->Data_field[9]
                            == 16) {
                            if (LMS_Ctrl_Packet_Rx->Data_field[0] == 0) // read data from EEPROM #1
                            {
                                //spirez = spirez || FlashQspi_ReadPage(&CFG_QSPI, mem_write_offset, page_buffer);
                                FlashQspi_CMD_ReadDataByte(mem_write_offset, &page_buffer[0]);
                                FlashQspi_CMD_ReadDataByte(mem_write_offset + 1, &page_buffer[1]);
                                glEp0Buffer_Tx[32] = page_buffer[0];
                                glEp0Buffer_Tx[33] = page_buffer[1];

                                // No error conditions in code
                                // if (spirez)
                                // 	LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                                // else
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                            } else
                                LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                        } else
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    } else if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 2))
                    // TARGET = 1 (FPGA FLASH)
                    {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    } else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
#else
#if 0
                    current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
                    data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

                    if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3))
                    /// TARGET = 3 (EEPROM)
                    {
                        if(LMS_Ctrl_Packet_Rx->Data_field[0] == 0) //read data from EEPROM #1
                        {
                            i2c_wdata[0]= LMS_Ctrl_Packet_Rx->Data_field[8];
                            i2c_wdata[1]= LMS_Ctrl_Packet_Rx->Data_field[9];
                            i2crez = OpenCoresI2CMasterWrite(i2c_master, EEPROM_I2C_ADDR, 2, i2c_wdata);

                            i2crez += OpenCoresI2CMasterRead(i2c_master, EEPROM_I2C_ADDR, (unsigned int) data_cnt, i2c_rdata);
                            OpenCoresI2CMasterStop(i2c_master);

                            for (k=0; k<data_cnt; k++)
                            {
                                LMS_Ctrl_Packet_Tx->Data_field[24+k] = i2c_rdata[k];

                            }

                            if(i2crez) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        } else
                            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    } else if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 1))
                    // TARGET = 1 (FX3)
                    {
                            flash_page  = LMS_Ctrl_Packet_Rx->Data_field[6] << 24;
                            flash_page |= LMS_Ctrl_Packet_Rx->Data_field[7] << 16;
                            flash_page |= LMS_Ctrl_Packet_Rx->Data_field[8] << 8;
                            flash_page |= LMS_Ctrl_Packet_Rx->Data_field[9];
                            //flash_page = flash_page / FLASH_PAGE_SIZE;

                            //if( FlashSpiTransfer(FLASH_SPI_BASE, SPI_NR_FLASH, flash_page, FLASH_PAGE_SIZE, flash_page_data, CyTrue) != CY_U3P_SUCCESS)  cmd_errors++;//write to flash
                            //TODO:if( FlashSpiRead(FLASH_SPI_BASE, SPI_NR_FLASH, flash_page, FLASH_PAGE_SIZE, flash_page_data) != CY_U3P_SUCCESS)  cmd_errors++;//write to flash

                            if(MicoSPIFlash_PageRead(spiflash, spiflash->memory_base+flash_page, (unsigned int)data_cnt, rdata)!= 0) cmd_errors++;

                            for (k=0; k<data_cnt; k++)
                            {
                                LMS_Ctrl_Packet_Tx->Data_field[24+k] = rdata[k];
                            }

                            if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                            else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    } else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    }
#endif
#endif

                    break;


                default:
                    /* This is unknown request. */
                    // isHandled = CyFalse;
                    printf("Error: Unknown Command: 0x%02x\n", LMS_Ctrl_Packet_Rx->Header.Command);
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
                    break;
            }

#ifdef DEBUG_CMD
            printf("Command: 0x%02x\n", LMS_Ctrl_Packet_Rx->Header.Command);
#endif

            // Send response to the command
            // for (int i = 0; i < 64 / sizeof(uint32_t); ++i)
#ifdef LIMESDR_XTRX
            for (int i = (64 / sizeof(uint32_t)) - 1; i >= 0; --i) {
                csr_write_simple(dest[i], (CSR_CNTRL_CNTRL_ADDR + i * 4));
            }
#else
            for (int i = 0; i < (64 / sizeof(uint32_t)); ++i) {
                //dest_byte_reordered = ((dest[cnt] & 0x000000FF) <<24) | ((dest[cnt] & 0x0000FF00) <<8) | ((dest[cnt] & 0x00FF0000) >>8) | ((dest[cnt] & 0xFF000000) >>24);
                dest_byte_reordered = dest[i];
                ft601_fifo_wdata_write(dest_byte_reordered);
                //printf("%ld\n", ft601_fifo_status_read());
            }
#endif


#ifdef LIMESDR_XTRX
            // printf("TX: ");
            // for (int i = 0; i < 64; i++) {
            //     printf("%02x ", dest[i]);
            // }
            // printf("\n");

            /* Clear all pending interrupts. */
            CNTRL_ev_pending_write(CNTRL_ev_pending_read());
            /* Reenable CNTRL irq */
            CNTRL_ev_enable_write(1 << CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET);
            irq_setmask(irq_getmask() | (1 << CNTRL_INTERRUPT));
#else
            //gpo_val = 0x0;
            //*gpo_reg = gpo_val;
            main_gpo_write(0);
#endif
        }
#if 0
        unsigned int gpo_reg_04_val = *gpo_reg_04;
        unsigned int gpo_reg_08_val = *gpo_reg_08;
#endif

#ifdef LIMESDR_XTRX
        // Clock config
        if (clk_cfg_pending) {
            irq_mask = irq_getmask(); // save irq mask
            irq_setmask(0); // disable all interrupts until clock cfg is completed

            clk_cfg_pending = 0;
            PLL_ADDRS *pll_addrs_pointer;
            uint8_t rez;

            // PHASE CONFIG
            if (var_phcfg_start > 0) {
                var_phcfg_start = 0;
                uint8_t phcfgmode = csr_read_simple(clk_ctrl_addrs.phcfg_mode);
                uint8_t pll_ind = csr_read_simple(clk_ctrl_addrs.pll_ind);

                // Set pll_addrs pointer according to pll_ind value
                if (pll_ind == 0) {
                    pll_addrs_pointer = &pll0_tx_addrs;
                } else if (pll_ind == 1) {
                    pll_addrs_pointer = &pll1_rx_addrs;
                }
                // CHECK PHASE MODE
                if (phcfgmode == 1) {
                    // Automatic phase search
                    rez = AutoPH_MMCM_CFG(pll_addrs_pointer, &clk_ctrl_addrs, &smpl_cmp_addrs);
                } else {
                    // Manual phase set
                    Update_MMCM_CFG(pll_addrs_pointer, &clk_ctrl_addrs);
                    // There is no fail condition for manual phase yet
                    rez = AUTO_PH_MMCM_CFG_SUCCESS;
                }

                if (rez == AUTO_PH_MMCM_CFG_SUCCESS) {
                    csr_write_simple(1, clk_ctrl_addrs.pllcfg_done);
                    csr_write_simple(1, clk_ctrl_addrs.phcfg_done);
                } else {
                    csr_write_simple(1, clk_ctrl_addrs.pllcfg_error);
                    csr_write_simple(1, clk_ctrl_addrs.phcfg_err);
                }
            }
            // PLL CONFIG
            if (var_pllcfg_start > 0) {
                var_pllcfg_start = 0;
                csr_write_simple(1, clk_ctrl_addrs.pllcfg_done);
            }
            // PLL RESET
            if (var_pllrst_start > 0) {
                var_pllrst_start = 0;
                csr_write_simple(1, clk_ctrl_addrs.pllcfg_done);
            }
            // Reenable all previously enabled interrupts
            irq_setmask(irq_mask);
        }
#else
        if (iShiftLeft == 1){
            if (iValue == 0x8) {
                iShiftLeft = 0;
                iValue = 0x4;
            } else {
                iValue = iValue << 1;
            }
        } else {
            iValue = iValue >> 1;
            if (iValue == 0) {
                iValue = 0x02;
                iShiftLeft = 1;
            }
        }
#endif
    }

    /* all done */
    return(0);
}
