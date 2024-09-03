// This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
// License: BSD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <generated/csr.h>
#include <generated/mem.h>

#include "i2c0.h"
#include "i2c1.h"
#include "lms7002m.h"
#include "fpga_flash_qspi.h"
#include "LMS64C_protocol.h"
#include "LimeSDR_XTRX.h"
#include "regremap.h"
#include "Xil_clk_drp.h"

#define sbi(p, n) ((p) |= (1UL << (n)))
#define cbi(p, n) ((p) &= ~(1 << (n)))

/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#define I2C_DAC_ADDR     0x4C
#define I2C_TERMO_ADDR   0x4B
#define LP8758_I2C_ADDR  0x60

/************************** Variable Definitions *****************************/
uint8_t block, cmd_errors, glEp0Buffer_Rx[64], glEp0Buffer_Tx[64];
tLMS_Ctrl_Packet* LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet*)glEp0Buffer_Tx;
tLMS_Ctrl_Packet* LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet*)glEp0Buffer_Rx;

// If an error points here, most likely some of the macros are invalid.
PLL_ADDRS pll1_rx_addrs = GENERATE_MMCM_DRP_ADDRS(CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM);
PLL_ADDRS pll0_tx_addrs = GENERATE_MMCM_DRP_ADDRS(CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM);
SMPL_CMP_ADDRS smpl_cmp_addrs = GENERATE_SMPL_CMP_ADDRS(CSR_LIME_TOP_LMS7002);
// clk_ctrl_addrs is declared in regremap.h
CLK_CTRL_ADDRS clk_ctrl_addrs = GENERATE_CLK_CTRL_ADDRS(CSR_LIME_TOP_LMS7002_CLK_CTRL);

uint8_t lms64_packet_pending;
volatile uint8_t clk_cfg_pending = 0;

volatile uint8_t var_phcfg_start;
volatile uint8_t var_pllcfg_start;
volatile uint8_t var_pllrst_start;

int data_cnt = 0;

unsigned int irq_mask;

uint16_t dac_val = DAC_DEFF_VAL;

// Since there is no eeprom on the XTRX board and the flash is too large for the gw
// we use the top of the flash instead of eeprom, thus the offset to last sector
#define mem_write_offset 0x01FF0000



//#define FW_VER 1 // Initial version
//#define FW_VER 2 // Fix for PLL config. hang when changing from low to high frequency.
//#define FW_VER 3 // Added serial number into GET_INFO cmd
#define FW_VER 5 // Firmware for Litex project

/*-----------------------------------------------------------------------*/
/* IRQ                                                                   */
/*-----------------------------------------------------------------------*/

///** IRQ example ISR. */
//static void irq_example_isr(void)
//{
//	uint8_t stat;
//
//	/* Read the pending interrupt status. */
//	stat = lime_top_ev_pending_read();
//
//	/* Check if IRQ0 is pending. */
//	if(stat & (1 << CSR_LIME_TOP_EV_STATUS_IRQ0_OFFSET)) {
//		// printf("IRQ0!\n");
//		/* Clear the IRQ0 pending status. */
//		lime_top_ev_pending_write((1 << CSR_LIME_TOP_EV_STATUS_IRQ0_OFFSET));
//	}
//
//	/* Check if IRQ1 is pending. */
//	if(stat & (1 << CSR_LIME_TOP_EV_STATUS_IRQ1_OFFSET)) {
//		// uint32_t dest;
//		// printf(" CNTRL: ");
//		// for (int cnt = 0; cnt < 16 ; cnt++)
//		// {
//		// 	dest = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt*4));
//		// 	printf("%x ", dest);
//		// }
//		// printf(" \n");
//		// busy_wait_us(10000);
//		// printf("IRQ1!\n");
//		/* Clear the IRQ1 pending status. */
//		lime_top_ev_pending_write((1 << CSR_LIME_TOP_EV_STATUS_IRQ1_OFFSET));
//	}
//}
//
///* Initialize the IRQ example. */
//// static void irq_example_init(void)
//// {
//// 	/* Clear all pending interrupts. */
//// 	lime_top_ev_pending_write(lime_top_ev_pending_read());
////
//// 	/* Enable IRQ0 and IRQ1. */
//// 	lime_top_ev_enable_write((1 << CSR_LIME_TOP_EV_STATUS_IRQ0_OFFSET) | (1 << CSR_LIME_TOP_EV_STATUS_IRQ1_OFFSET));
////
//// 	/* Attach the example ISR to the interrupt. */
//// 	irq_attach(LIME_TOP_INTERRUPT, irq_example_isr);
////
//// 	/* Enable the example interrupt. */
//// 	irq_setmask(irq_getmask() | (1 << LIME_TOP_INTERRUPT));
//// }

/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

static char* readstr(void)
{
	char c[2];
	static char s[64];
	static int ptr = 0;

	if (readchar_nonblock())
	{
		c[0] = getchar();
		c[1] = 0;
		switch (c[0])
		{
		case 0x7f:
		case 0x08:
			if (ptr > 0)
			{
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

static char* get_token(char** str)
{
	char* c, * d;

	c = (char*)strchr(*str, ' ');
	if (c == NULL)
	{
		d = *str;
		*str = *str + strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c + 1;
	return d;
}

static void prompt(void)
{
	printf("\e[92;1mlimesdr-xtrx\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
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
#ifdef CSR_LIME_TOP_BASE
	puts("gpioled            - GPIO override demo");
	puts("mmap               - MMAP demo" );
#endif
#ifdef CSR_LIME_TOP_BASE
	puts("lms                - lms7002_top module status");
#endif
	puts("dac_test           - Test DAC");
	puts("flash_test         - Test FPGA FLASH");
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
	ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE

static void led_cmd(void)
{
	int i;
	printf("Led demo...\n");

	printf("Counter mode...\n");
	for (i = 0; i < 32; i++)
	{
		leds_out_write(i);
		busy_wait(100);
	}

	printf("Shift mode...\n");
	for (i = 0; i < 4; i++)
	{
		leds_out_write(1 << i);
		busy_wait(200);
	}
	for (i = 0; i < 4; i++)
	{
		leds_out_write(1 << (3 - i));
		busy_wait(200);
	}

	printf("Dance mode...\n");
	for (i = 0; i < 4; i++)
	{
		leds_out_write(0x55);
		busy_wait(200);
		leds_out_write(0xaa);
		busy_wait(200);
	}
}

#endif

#ifdef CSR_LIME_TOP_BASE

static void gpioled_cmd(void)
{
	int i, j;
	printf("GPIO Led override demo...\n");
	lime_top_gpio_gpio_override_write(0x7);
	lime_top_gpio_gpio_override_dir_write(0x0);
	lime_top_gpio_gpio_override_val_write(0x0);
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 8; j++)
		{
			lime_top_gpio_gpio_override_val_write(0x0);
			busy_wait(100);
			lime_top_gpio_gpio_override_val_write(1 << i);
			busy_wait(100);
		}
	}

	lime_top_gpio_gpio_override_val_write(0x0);
	printf("GPIO Led override demo...ended\n");
}

static void mmap_cmd(void)
{
	int i;

	volatile uint32_t* mmap_ptr = (volatile uint32_t*)LIME_TOP_MMAP_BASE;
	printf("MMAP demo...\n");

	/* Write counter values to SRAM. */

	printf("Writing values to SRAM at address 0x%08lX...\n", LIME_TOP_MMAP_BASE);
	for (i = 0; i < 8; i++)
	{
		mmap_ptr[i] = i;
		printf("Wrote %d to address 0x%08X\n", i, (unsigned int)(LIME_TOP_MMAP_BASE + i * sizeof(uint32_t)));
	}

	/* Read back values from SRAM. */
	printf("Reading values from SRAM at address 0x%08lX...\n", LIME_TOP_MMAP_BASE);
	for (i = 0; i < 8; i++)
	{
		uint32_t value = mmap_ptr[i];
		printf("Read %ld from address 0x%08X\n", value, (unsigned int)(LIME_TOP_MMAP_BASE + i * sizeof(uint32_t)));
	}
	// *mmap_ptr = (volatile uint32_t *)CSR_BASE;
	// printf("Reading values from SRAM at address 0x%08lX...\n", CSR_BASE);
	// for (i = 0; i < 8; i++) {
	// 	uint32_t value = mmap_ptr[i];
	// 	printf("Read 0x%08X from address 0x%08X\n", value, (unsigned int)(CSR_BASE + i * sizeof(uint32_t)));
	// }
	uint32_t dest;
	printf(" CNTRL: ");
	for (int cnt = 0; cnt < 16; cnt++)
	{
		dest = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
		printf("%x ", dest);
	}
	printf(" \n");


	printf(" CNTRL wr: \n");
	csr_write_simple(CSR_CNTRL_CNTRL_ADDR, 5);
	// for (int cnt = 0; cnt < 16 ; cnt++)
	// {
	// dest = csr_read_simple((CSR_CNTRL_KOPUSTAS_ADDR + cnt*4));
	// printf("%x ", dest);
	// }
	// printf(" \n");
}

#endif

/*-----------------------------------------------------------------------*/
/* I2C                                                                   */
/*-----------------------------------------------------------------------*/

static void i2c_test(void)
{
	printf("I2C0 Scan...\n");
	i2c0_scan();

	printf("\n");

	printf("I2C1 Scan...\n");
	i2c1_scan();
}

static void dac_test(void)
{
	unsigned char adr;
	unsigned char dat[2];

	adr=0;

	i2c0_read(I2C_DAC_ADDR, adr, &dat, 2, true);
	printf("Read DAC val...\n");
	printf("0x%02x: 0x%02x\n", dat[0], dat[1]);
	//i2c0_scan();

	printf("\n");

	printf("Write DAC val...\n");
	adr=0x30;
	dat[0] = 0xaa;
	dat[1] = 0x55;
	i2c0_write(I2C_DAC_ADDR, adr, &dat, 2);
	//i2c1_scan();
	i2c0_read(I2C_DAC_ADDR, adr, &dat, 2, true);
	printf("Read DAC val...\n");
	printf("0x%02x: 0x%02x\n", dat[0], dat[1]);
}

static void flash_test(void)
{	uint8_t regvals2[2];
    uint8_t data_bytes[2];
    int retval = 0;
	printf("FPGA FLASH test...\n");
	FlashQspi_CMD_ReadRDSR(&regvals2[0]);
	FlashQspi_CMD_ReadRDCR(&regvals2[1]);
	printf("FPGA FLASH test end...\n");
	FlashQspi_CMD_WREN();
	FlashQspi_CMD_WRDI();
	FlashQspi_CMD_ReadDataByte(0x4fff5, data_bytes);

	// Erase sector
	FlashQspi_CMD_WREN();
	retval = FlashQspi_CMD_SectorErase(0x4fff5);

	uint8_t myByte = 0x55;
	uint32_t myAddress = 0x4fff5;
	// Write Byte
	FlashQspi_CMD_WREN();
	int result = FlashQspi_CMD_PageProgramByte(myAddress, &myByte);

	FlashQspi_CMD_ReadDataByte(0x4fff5, data_bytes);

}

static void dump_pmic(void)
{
	unsigned char adr;
	unsigned char dat;
	printf("FPGA_I2C1 PMIC Dump...\n");
	for (adr = 0; adr < 32; adr++)
	{
		i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
	printf("FPGA_I2C2 PMIC Dump...\n");
	for (adr = 0; adr < 32; adr++)
	{
		i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
}

static void init_pmic(void)
{
	printf("Initializing DC-DC switching regulators...\n");

	unsigned char adr;
	unsigned char dat;

	printf("PMICs Initialization...\n");
	printf("-----------------------\n");

	printf("FPGA_I2C1 PMIC: Check ID ");
	adr = 0x01;
	i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0)
	{
		printf("KO, exiting.\n");
	}
	else
	{
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
	if (dat != 0xe0)
	{
		printf("KO, exiting.\n");
	}
	else
	{
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

static void init_vctcxo_dac(void)
{
	// Write initial VCTCXO DAC value on firmware boot
	uint8_t i2c_buf[3];
	uint8_t page_buffer[5] = {0};

	FlashQspi_CMD_ReadDataByte(mem_write_offset, &page_buffer[0]);
	FlashQspi_CMD_ReadDataByte(mem_write_offset + 1, &page_buffer[1]);
	printf("0x%02x: 0x%02x\n", page_buffer[0], page_buffer[1]);

	// Write DAC value stored in flash storage only if it isn't empty (0xFFFF)
	if ((page_buffer[1] == 0xFF) && (page_buffer[0] == 0xFF)) {
		// Write Default value
		dac_val = DAC_DEFF_VAL;
		i2c_buf[0] = 0x30; // cmd
		i2c_buf[1] = (dac_val >> 8) & 0xFF;
		i2c_buf[2] = dac_val & 0xFF;
	} else {
		// Write DAC value from FLASH
		dac_val = ((uint16_t)page_buffer[1])<<8 | ((uint16_t)page_buffer[0]);
		i2c_buf[0] = 0x30; // cmd
		i2c_buf[1] = page_buffer[1];
		i2c_buf[2] = page_buffer[0];
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

static void lms7002_status(void)
{
	printf("TX PLL Lock status = %x \n", csr_read_simple(pll0_tx_addrs.locked));
	printf("\n");
	printf("RX PLL Lock status = %x \n", csr_read_simple(pll1_rx_addrs.locked));
}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
	char* str;
	char* token;

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
#ifdef CSR_LIME_TOP_BASE
	else if (strcmp(token, "gpioled") == 0)
		gpioled_cmd();
	else if (strcmp(token, "mmap") == 0)
		mmap_cmd();
#endif
#ifdef CSR_LIME_TOP_BASE
	else if (strcmp(token, "lms") == 0)
		lms7002_status();
#endif
	else if (strcmp(token, "dac_test") == 0)
		dac_test();
	else if (strcmp(token, "flash_test") == 0)
		flash_test();

	prompt();
}


/** Checks if peripheral ID is valid.
 Returns 1 if valid, else 0. */
unsigned char Check_Periph_ID(unsigned char max_periph_id, unsigned char Periph_ID)
{
	if (LMS_Ctrl_Packet_Rx->Header.Periph_ID > max_periph_id)
	{
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
		return 0;
	}
	else
		return 1;
}

/**	This function checks if all blocks could fit in data field.
 *	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks(unsigned char block_size)
{
	if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field) / block_size))
	{
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
		return 1;
	}
	else
		return 0;
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

void getLMS64Packet(uint8_t *buf, uint8_t k)
{
    uint8_t cnt = 0;
    uint32_t *dest = (uint32_t *)buf;
    uint32_t temp_buffer[k / sizeof(uint32_t)];
    uint8_t is_stable = 0;

    while (!is_stable)
    {
        // Read the first buffer into temp_buffer
        for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++)
        {
            temp_buffer[cnt] = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
        }

        // Read again into dest buffer
        for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++)
        {
            dest[cnt] = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
        }

        // Compare the two buffers
        is_stable = 1;
        for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++)
        {
            if (temp_buffer[cnt] != dest[cnt])
            {
                is_stable = 0;
                break;
            }
        }
    }
}


static void lms64c_isr(void)
{
	// printf("CNTRL IRQ!\n");
	uint32_t* dest = (uint32_t*)glEp0Buffer_Tx;
	uint32_t read_value;

	uint8_t reg_array[4];
	uint16_t addr;
	uint16_t val;
	uint8_t i2c_buf[3];

	uint8_t page_buffer[5]; // page buffer

	int spirez;

	//printf("ISR: LMS64C Entry\n");

	lms64_packet_pending = 1;
	getLMS64Packet(glEp0Buffer_Rx, 64);

	memset(glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); // fill whole tx buffer with zeros
	cmd_errors = 0;

	LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
	LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
	LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
	LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

	switch (LMS_Ctrl_Packet_Rx->Header.Command)
	{
	case CMD_GET_INFO:

		LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
		LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
		LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
		LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
		LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;

	case CMD_LMS_RST:

		if (!Check_Periph_ID(MAX_ID_LMS7, LMS_Ctrl_Packet_Rx->Header.Periph_ID))
			break;

		switch (LMS_Ctrl_Packet_Rx->Data_field[0])
		{
		case LMS_RST_DEACTIVATE:
			//Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); // high level
			//printf("LMS RESET deactivate...\n");
			break;
		case LMS_RST_ACTIVATE:
			//Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); // low level
			//printf("LMS RESET activate...\n");
			break;

		case LMS_RST_PULSE:
			//Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); // low level
			//Modify_BRDSPI16_Reg_bits(BRD_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); // high level
			lime_top_lms7002_lms1_resetn_write(0x0);
			lime_top_lms7002_lms1_resetn_write(0x1);
			//printf("LMS RST pulse...\n");
			break;
		default:
			cmd_errors++;
			break;
		}

	case CMD_BRDSPI16_WR:
		if (Check_many_blocks(4))
			break;

		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{
			// write reg addr
			//sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
			// Clearing write bit in address field because we are not using SPI registers in LiteX implementation
			cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // clear write bit

			writeCSR(&LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)],
					&LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
		}
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;

	case CMD_BRDSPI16_RD:
		if (Check_many_blocks(4))
			break;

		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{

			// write reg addr
			cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit

			readCSR(&LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], reg_array);
			LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = reg_array[1];
			LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = reg_array[0];

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


		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{
			sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
			// Parse address
			addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)];
			addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)];
			// Parse value
			val = LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)];
			val = (val << 8) | LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
			// Write
			lms_spi_write(addr, val);

		}

		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;

		// COMMAND LMS READ

	case CMD_LMS7002_RD:
		if (Check_many_blocks(4))
			break;

		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{
			cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit
			// Parse address
			addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)];
			addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];
			// Read
			val = lms_spi_read(addr);
			// Return value and address
			LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)];
			LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];
			LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (val >> 8) & 0xFF;
			LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = val & 0xFF;
		}

		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;


	case CMD_ANALOG_VAL_RD:
		// TODO: CMD_ANALOG_VAL_RD
		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{
			switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block)]) // ch
			{
			case 0:				   // dac val
				//XIic_Recv(XPAR_I2C_CORES_I2C1_BASEADDR, I2C_DAC_ADDR, i2c_buf, 2, XIIC_STOP);
				//i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
				i2c0_read(I2C_DAC_ADDR, 0x0, &i2c_buf, 2, true);
				LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; // ch
				LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x00;									 // RAW //unit, power
				LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = i2c_buf[0];							 // unsigned val, MSB byte
				LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = i2c_buf[1];							 // unsigned val, LSB byte
				// Storing volatile DAC value
				dac_val = ((uint16_t)i2c_buf[0])<<8 | ((uint16_t)i2c_buf[1]);

				break;

			case 1: // temperature
//						i2c_buf[0] = 1;
//						i2c_buf[1] = 0x60;
//						i2c_buf[2] = 0xA0;
				// TMP1075 sensor performs periodical temperature readings by default
				// we only need to read the most recent value
				i2c_buf[0]=0;
				//XIic_Send(XPAR_I2C_CORES_I2C1_BASEADDR,I2C_TERMO_ADDR,i2c_buf,1,XIIC_REPEATED_START);
				//XIic_Recv(XPAR_I2C_CORES_I2C1_BASEADDR,I2C_TERMO_ADDR,i2c_buf,2,XIIC_STOP);
				i2c0_read(I2C_TERMO_ADDR, i2c_buf[0], &i2c_buf[0], 2, false);


				LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
				LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x50; //0.1C //unit, power

				int16_t converted_value = i2c_buf[1] | (i2c_buf[0] << 8);

				converted_value = converted_value >> 4;
				converted_value = converted_value * 10;
				converted_value = converted_value >> 4;

				LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (uint8_t)((converted_value >> 8) & 0xFF);//signed val, MSB byte
				LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = (uint8_t)(converted_value & 0xFF);//signed val, LSB byte

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

		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{
			switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)]) // do something according to channel
			{
			case 0:														  // TCXO DAC
				if (LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)] == 0) // RAW units?
				{
					i2c_buf[0] = 0x30;											  // addr
					i2c_buf[1] = LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]; // MSB
					i2c_buf[2] = LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)]; // LSB
					// Storing volatile DAC value
					dac_val = ((uint16_t)i2c_buf[1])<<8 | ((uint16_t)i2c_buf[2]);
					// Writing to DAC
					//XIic_Send(XPAR_I2C_CORES_I2C1_BASEADDR, I2C_DAC_ADDR, i2c_buf, 3, XIIC_STOP);
					i2c0_write(I2C_DAC_ADDR, i2c_buf[0], &i2c_buf[1], 2);
				}
				else
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
		break;

	case CMD_MEMORY_WR:
		// Since the XTRX board does not have an eeprom to store permanent VCTCXO DAC value
		// a workaround is implemented that uses a sufficiently high address in the configuration flash
		// to store the DAC value
		// Since to write data to a flash, a whole sector needs to be erased, additional checks are included
		// to make sure this function is used ONLY to store VCTCXO DAC value
		// Reset spirez
		spirez = 0;
		data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

		if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3)) // TARGET = 3 (EEPROM)
		{
			// Since the XTRX board does not have an eeprom to store permanent VCTCXO DAC value
			// a workaround is implemented that uses a sufficiently high address in the configuration flash
			// to store the DAC value
			// Since to write data to a flash, a whole sector needs to be erased, additional checks are included
			// to make sure this function is used ONLY to store VCTCXO DAC value

			// Check if the user is trying to store VCTCXO DAC value, return error otherwise
			if (data_cnt == 2 && LMS_Ctrl_Packet_Rx->Data_field[8] == 0 && LMS_Ctrl_Packet_Rx->Data_field[9] == 16)
			{
				if (LMS_Ctrl_Packet_Rx->Data_field[0] == 0) // write data to EEPROM #1
				{
					LMS_Ctrl_Packet_Rx->Data_field[22] = LMS_Ctrl_Packet_Rx->Data_field[8];
					LMS_Ctrl_Packet_Rx->Data_field[23] = LMS_Ctrl_Packet_Rx->Data_field[9];
					FlashQspi_CMD_WREN();
					spirez = spirez || FlashQspi_CMD_SectorErase(mem_write_offset);
					page_buffer[0] = LMS_Ctrl_Packet_Rx->Data_field[24];
					page_buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[25];
					//spirez = spirez || FlashQspi_ProgramPage(&CFG_QSPI, mem_write_offset, page_buffer);
					//cmd_errors = cmd_errors + spirez;
					FlashQspi_CMD_WREN();
					spirez = FlashQspi_CMD_PageProgramByte(mem_write_offset, &page_buffer[0]);
					FlashQspi_CMD_WREN();
					spirez = FlashQspi_CMD_PageProgramByte(mem_write_offset+1, &page_buffer[1]);

					cmd_errors = 0;
					if (cmd_errors)
						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else
						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
				}
				else
					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
			}
			else
				LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

		}
		else if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 2)) // TARGET = 2 (FPGA FLASH)
		{
			printf("F \n");
			LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
		}
		else {
			LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
			printf("N \n");
		}

		break;

	case CMD_MEMORY_RD:
		// Since the XTRX board does not have an eeprom to store permanent VCTCXO DAC value
		// a workaround is implemented that uses a sufficiently high address in the configuration flash
		// to store the DAC value
		// Since to write data to a flash, a whole sector needs to be erased, additional checks are included
		// to make sure this function is used ONLY to store VCTCXO DAC value
		// Reset spirez
		spirez = 0;
		data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

		if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3)) /// TARGET = 3 (EEPROM)
		{
			if (data_cnt == 2 || LMS_Ctrl_Packet_Rx->Data_field[8] == 0 || LMS_Ctrl_Packet_Rx->Data_field[9] == 16)
			{
				if (LMS_Ctrl_Packet_Rx->Data_field[0] == 0) // read data from EEPROM #1
				{
					//spirez = spirez || FlashQspi_ReadPage(&CFG_QSPI, mem_write_offset, page_buffer);
					FlashQspi_CMD_ReadDataByte(mem_write_offset, &page_buffer[0]);
					FlashQspi_CMD_ReadDataByte(mem_write_offset + 1, &page_buffer[1]);
					glEp0Buffer_Tx[32] = page_buffer[0];
					glEp0Buffer_Tx[33] = page_buffer[1];

					if (spirez)
						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else
						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
				}
				else
					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
			}
			else
				LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
		}
		else if ((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 2)) // TARGET = 1 (FPGA FLASH)
		{
			LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
		}
		else
			LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

		break;


	default:
		/* This is unknown request. */
		// isHandled = CyFalse;
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
		break;
	}

	// Send response to the command
	//for (int i = 0; i < 64 / sizeof(uint32_t); ++i)
	for (int i = (64 / sizeof(uint32_t)) - 1; i >= 0; --i)
	{
		csr_write_simple(dest[i], (CSR_CNTRL_CNTRL_ADDR + i * 4));
	}

	CNTRL_ev_pending_write(CNTRL_ev_pending_read()); // Clear interrupt
	CNTRL_ev_enable_write(1 << CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET);   // re-enable the event handler
	//printf("ISR: LMS64C exit\n");

}


static void lms64c_init(void)
{
	uint32_t irq_mask;
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

static void clk_ctrl_isr(void)
{
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

static void clk_cfg_irq_init(void)
{
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


int main(void)
{


#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();
	init_pmic();
	printf("CSR_CNTRL_BASE 0x%lx ", CSR_CNTRL_BASE);
	printf("sveiki visi  :) \n");
	// irq_example_init();
	lms64c_init();
	clk_cfg_irq_init();
	init_pmic();
	init_vctcxo_dac();
	help();
	prompt();

	while (1)
	{
		console_service();

		// Clock config
		if (clk_cfg_pending)
		{
			irq_mask = irq_getmask(); // save irq mask
			irq_setmask(0); // disable all interrupts until clock cfg is completed

			clk_cfg_pending = 0;
			PLL_ADDRS* pll_addrs_pointer;
			uint8_t rez;

			// PHASE CONFIG
			if (var_phcfg_start > 0)
			{
				var_phcfg_start = 0;
				uint8_t phcfgmode = csr_read_simple(clk_ctrl_addrs.phcfg_mode);
				uint8_t pll_ind = csr_read_simple(clk_ctrl_addrs.pll_ind);

				// Set pll_addrs pointer according to pll_ind value
				if (pll_ind == 0)
				{
					pll_addrs_pointer = &pll0_tx_addrs;
				}
				else if (pll_ind == 1)
				{
					pll_addrs_pointer = &pll1_rx_addrs;
				}
				// CHECK PHASE MODE
				if (phcfgmode == 1)
				{ // Automatic phase search
					rez = AutoPH_MMCM_CFG(pll_addrs_pointer, &clk_ctrl_addrs, &smpl_cmp_addrs);
				}
				else
				{ // Manual phase set
					Update_MMCM_CFG(pll_addrs_pointer, &clk_ctrl_addrs);
					// There is no fail condition for manual phase yet
					rez = AUTO_PH_MMCM_CFG_SUCCESS;
				}

				if (rez == AUTO_PH_MMCM_CFG_SUCCESS)
				{
					csr_write_simple(1, clk_ctrl_addrs.pllcfg_done);
					csr_write_simple(1, clk_ctrl_addrs.phcfg_done);
				}
				else
				{
					csr_write_simple(1, clk_ctrl_addrs.pllcfg_error);
					csr_write_simple(1, clk_ctrl_addrs.phcfg_err);
				}

			}
			// PLL CONFIG
			if (var_pllcfg_start > 0)
			{
				var_pllcfg_start = 0;
				csr_write_simple(1, clk_ctrl_addrs.pllcfg_done);
			}
			// PLL RESET
			if (var_pllrst_start > 0)
			{
				var_pllrst_start = 0;
				csr_write_simple(1, clk_ctrl_addrs.pllcfg_done);
			}
			// Reenable all previously enabled interrupts
			irq_setmask(irq_mask);
		}
	}
}
