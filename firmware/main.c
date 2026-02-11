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

#include "LMS64C_protocol.h"
#include "HiperSDR_44xx.h"
#include "regremap.h"
#include "Xil_clk_drp.h"
#include "bsp.h"
#include "console_func.h"
#include "fpga_flash_qspi.h"

#define sbi(p, n) ((p) |= (1UL << (n)))
#define cbi(p, n) ((p) &= ~(1 << (n)))

/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#define I2C_DAC_ADDR     0x4C
#define I2C_TERMO_ADDR   0x4B
#define LP8758_I2C_ADDR  0x60

/************************** Variable Definitions *****************************/
uint8_t block, cmd_errors;
uint8_t glEp0Buffer_Rx[64], glEp0Buffer_Tx[64];
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet *) glEp0Buffer_Tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet *) glEp0Buffer_Rx;

#ifdef CSR_LIME_TOP_LMS7002
// If an error points here, most likely some of the macros are invalid.
PLL_ADDRS pll1_rx_addrs = GENERATE_MMCM_DRP_ADDRS(CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM);
PLL_ADDRS pll0_tx_addrs = GENERATE_MMCM_DRP_ADDRS(CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM);
SMPL_CMP_ADDRS smpl_cmp_addrs = GENERATE_SMPL_CMP_ADDRS(CSR_LIME_TOP_LMS7002);
// clk_ctrl_addrs is declared in regremap.h
CLK_CTRL_ADDRS clk_ctrl_addrs = GENERATE_CLK_CTRL_ADDRS(CSR_LIME_TOP_LMS7002_CLK_CTRL);
#endif

volatile uint8_t lms64_packet_pending;

//Flash programming variables
volatile uint8_t flash_prog_pending = 0;

uint8_t page_buffer[256]; // page buffer
int data_cnt = 0;
unsigned long int current_portion;
int address;
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


#define PWR_LMS8_NRST_OFFSET 7
#define PWR_LMS8_NRST_POS    0

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


/**	This function checks if all blocks could fit in data field.
 *	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks(unsigned char block_size) {
    if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field) / block_size)) {
        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
        return 1;
    } else
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
#ifdef CSR_LIME_TOP_LMS7002
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
#endif
}

static void clk_cfg_irq_init(void) {
#ifdef CSR_LIME_TOP_LMS7002
	printf("CLK config irq initialization \n");

    /* Clear all pending interrupts. */
    lime_top_ev_pending_write(lime_top_ev_pending_read());

    /* Enable CLK CTRL irq */
    lime_top_ev_enable_write(1 << CSR_LIME_TOP_EV_STATUS_CLK_CTRL_IRQ_OFFSET);

    /* Attach isr to interrupt */
    irq_attach(LIME_TOP_INTERRUPT, clk_ctrl_isr);

    /* Enable interrupt */

    irq_setmask(irq_getmask() | (1 << LIME_TOP_INTERRUPT));
#endif
}

void copyArray(unsigned char *source, unsigned char *destination, size_t sourceIndex, size_t destinationIndex,
               size_t count) {
    memcpy(destination + destinationIndex, source + sourceIndex, count);
}


int main(void) {
    int spirez;
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();
    //init_pmic();
    printf("CSR_CNTRL_BASE 0x%lx \n", CSR_CNTRL_BASE);
    // irq_example_init();
    lms64c_init();
    bsp_isr_init();
    clk_cfg_irq_init();

    //init_pmic();
    //init_vctcxo_dac();
    bsp_init();

    {

        //Check if there is a value in permanent vctcxo memory
        //If there is, write it to runtime DAC
        //If there isn't write default
        uint16_t perm_dac_val;
        const uint8_t *perm_dac_ptr = (uint8_t *) &perm_dac_val;
        bsp_vctcxo_permanent_dac_read((uint8_t *) &perm_dac_val);
        if (perm_dac_val != 0xFFFF) {
            bsp_analog_write(BSP_DAC_INDEX, 0x00, perm_dac_ptr[1], perm_dac_ptr[0]);
        }
        else {
            bsp_analog_write(BSP_DAC_INDEX, 0x00, (DAC_DEFF_VAL&0xff00 >>8), DAC_DEFF_VAL&0xff);
        }
    }

    help();
    prompt();

    uint8_t prev_pwr_lms8_nrst_state = (bsp_gpio_get_cached(PWR_LMS8_NRST_OFFSET) >> PWR_LMS8_NRST_POS) & 0x01;

    while (1) {
        console_service();

        //TODO: move this to bsp_process_irqs
        // 2. Poll: Extract the current state of that specific bit
		uint8_t current_byte = bsp_gpio_get_cached(PWR_LMS8_NRST_OFFSET);
		uint8_t curr_pwr_lms8_nrst_state = (current_byte >> PWR_LMS8_NRST_POS) & 0x01;

		// 3. Compare: Check if the bit state flipped
		if (curr_pwr_lms8_nrst_state != prev_pwr_lms8_nrst_state) {

			if (curr_pwr_lms8_nrst_state == 1) {
				printf("PWR_LMS8_NRST ON\n");
				bsp_lms8_pwrup();
			} else {
				printf("PWR_LMS8_NRST OFF\n");
			}

			// 4. Update history
			prev_pwr_lms8_nrst_state = curr_pwr_lms8_nrst_state;
		}

	bsp_process_irqs();

        // Process received packet
        if (lms64_packet_pending) {
            /* Disable CNTRL irq while processing packet */
            CNTRL_ev_enable_write(CNTRL_ev_enable_read() & ~(1 << CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET));
            irq_setmask(irq_getmask() & ~(1 << CNTRL_INTERRUPT));

            lms64_packet_pending = 0;
            // printf("CNTRL PCT GOT!\n");
            uint32_t *dest = (uint32_t *) glEp0Buffer_Tx;

            uint8_t reg_array[4];
            uint16_t addr;
            uint16_t val;
            uint8_t i2c_buf[3];

            getLMS64Packet(glEp0Buffer_Rx, 64);
            // printf("RX: ");
            // for (int i = 0; i < 64; i++) {
            //     printf("%02x ", glEp0Buffer_Rx[i]);
            // }
            // printf("\n");

            memset(glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); // fill whole tx buffer with zeros
            cmd_errors = 0;

            LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
            LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
            LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
            LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

            switch (LMS_Ctrl_Packet_Rx->Header.Command) {
                case CMD_GET_INFO:

                    LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
                    LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
                    LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
                    LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
                    LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

                    // Read Serial number from FLASH OTP region
                    //spirez = FlashQspi_CMD_ReadOTPData(OTP_SERIAL_ADDRESS, sizeof(serial), serial);

                    LMS_Ctrl_Packet_Tx->Data_field[10] = serial[7];
                    LMS_Ctrl_Packet_Tx->Data_field[11] = serial[6];
                    LMS_Ctrl_Packet_Tx->Data_field[12] = serial[5];
                    LMS_Ctrl_Packet_Tx->Data_field[13] = serial[4];
                    LMS_Ctrl_Packet_Tx->Data_field[14] = serial[3];
                    LMS_Ctrl_Packet_Tx->Data_field[15] = serial[2];
                    LMS_Ctrl_Packet_Tx->Data_field[16] = serial[1];
                    LMS_Ctrl_Packet_Tx->Data_field[17] = serial[0];

                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

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
                                //spirez = FlashQspi_ProgramOTP(OTP_SERIAL_ADDRESS, LMS_Ctrl_Packet_Rx->Data_field[1], tmp_serial);
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
                    //spirez = FlashQspi_CMD_ReadOTPData(OTP_SERIAL_ADDRESS, 32, tmprd_serial);
                    copyArray(tmprd_serial, LMS_Ctrl_Packet_Tx->Data_field, 0, 24, 32);
                    LMS_Ctrl_Packet_Tx->Data_field[1] = 16;
                    LMS_Ctrl_Packet_Tx->Data_field[2] = serial_otp_unlock_key;
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_LMS_RST:
                    // Reuse val variable to save on memory
                    val = (uint16_t) lms7002m_periph_id_check(LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                    // If periph ID is invalid
                    if (val == 1) {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
                        break;
                    }
                    // If periph ID is valid
                    if (val == 0) {
                        switch (LMS_Ctrl_Packet_Rx->Data_field[0]) {
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
#ifdef CSR_LIME_TOP_LMS7002
                                lime_top_lms7002_lms1_resetn_write(0x0);
                                lime_top_lms7002_lms1_resetn_write(0x1);
                                //printf("LMS RST pulse...\n");
#endif
                                break;
                            default:
                                cmd_errors++;
                                break;
                        }
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        break;
                    }
                // If val is neither 0 nor 1, fallthrough to default to indicate the function is not implemented.
                // Values other than 0 or 1 will be hardcoded, so this whole case should be optimized out by compiler

                case CMD_BRDSPI16_WR:
                    if (Check_many_blocks(4))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
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

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
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

                // COMMAND LMS7 WRITE

                case CMD_LMS7002_WR:
                    // Reuse val variable to save on memory
                    val = (uint16_t) lms7002m_periph_id_check(LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                    // If periph ID is invalid
                    if (val == 0) {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
                        break;
                    }
                    // If periph ID is valid
                    if (val == 1) {
                        if (Check_many_blocks(4))
                            break;


                        for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                            sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
                            // Parse address
                            addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)];
                            addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)];
                            // Parse value
                            val = LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)];
                            val = (val << 8) | LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
                            // Write
                            lms7002m_spi_write(addr, val, LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                        }

                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        break;
                    }
                // If val is neither 0 nor 1, fallthrough to default to indicate the function is not implemented.
                // Values other than 0 or 1 will be hardcoded, so this whole case should be optimized out by compiler

                // COMMAND LMS7 READ


                case CMD_LMS7002_RD:
                    // Reuse val variable to save on memory
                    val = (uint16_t) lms7002m_periph_id_check(LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                    // If periph ID is invalid
                    if (val == 0) {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
                        break;
                    }
                    // If periph ID is valid
                    if (val == 1) {
                        if (Check_many_blocks(4))
                            break;

                        for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                            cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit
                            // Parse address
                            addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)];
                            addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];
                            // Read
                            val = lms7002m_spi_read(addr, (uint32_t) (1 << LMS_Ctrl_Packet_Rx->Header.Periph_ID));
                            // Return value and address
                            LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[
                                0 + (block * 2)];
                            LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[
                                1 + (block * 2)];
                            LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (val >> 8) & 0xFF;
                            LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = val & 0xFF;
                        }

                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        break;
                    }
                // If val is neither 0 nor 1, fallthrough to default to indicate the function is not implemented.
                // Values other than 0 or 1 will be hardcoded, so this whole case should be optimized out by compiler

                // COMMAND LMS7 WRITE

                case CMD_LMS8001_WR:
                    // Reuse val variable to save on memory
                    val = (uint16_t) lms8001_periph_id_check(LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                    // If periph ID is invalid
                    if (val == 0) {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
                        break;
                    }
                    // If periph ID is valid
                    if (val == 1) {
                        if (Check_many_blocks(4))
                            break;


                        for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                            sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
                            // Parse address
                            addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)];
                            addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)];
                            // Parse value
                            val = LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)];
                            val = (val << 8) | LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
                            // Write
                            lms8001_spi_write(addr, val, LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                        }

                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        break;
                    }
                // If val is neither 0 nor 1, fallthrough to default to indicate the function is not implemented.
                // Values other than 0 or 1 will be hardcoded, so this whole case should be optimized out by compiler

                // COMMAND LMS7 READ


                case CMD_LMS8001_RD:
                    // Reuse val variable to save on memory
                    val = (uint16_t) lms8001_periph_id_check(LMS_Ctrl_Packet_Rx->Header.Periph_ID);
                    // If periph ID is invalid
                    if (val == 0) {
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
                        break;
                    }
                    // If periph ID is valid
                    if (val == 1) {
                        if (Check_many_blocks(4))
                            break;

                        for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                            cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit
                            // Parse address
                            addr = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)];
                            addr = (addr << 8) | LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];
                            // Read
                            val = lms8001_spi_read(addr, (uint32_t) (1 << LMS_Ctrl_Packet_Rx->Header.Periph_ID));
                            // Return value and address
                            LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[
                                0 + (block * 2)];
                            LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[
                                1 + (block * 2)];
                            LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (val >> 8) & 0xFF;
                            LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = val & 0xFF;
                        }

                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                        break;
                    }
                // If val is neither 0 nor 1, fallthrough to default to indicate the function is not implemented.
                // Values other than 0 or 1 will be hardcoded, so this whole case should be optimized out by compiler

                case CMD_GPIO_DIR_WR:
                    if (Check_many_blocks(1))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        cmd_errors += bsp_gpio_dir_write(LMS_Ctrl_Packet_Rx->Data_field[block],block);
                    }
                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_GPIO_DIR_RD:
                    if (Check_many_blocks(1))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        cmd_errors += bsp_gpio_dir_read(&LMS_Ctrl_Packet_Tx->Data_field[block],block);
                    }
                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_GPIO_WR:
                	//printf("[CMD]: CMD_GPIO_WR\n");
                    if (Check_many_blocks(1)) {
                    	//printf("[DATA_BLOCK]: %x ", LMS_Ctrl_Packet_Rx->Header.Data_blocks);
                        break;
                    }

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        cmd_errors += bsp_gpio_write(LMS_Ctrl_Packet_Rx->Data_field[block],block);
                        //printf("[%x]: %x\n ", block, LMS_Ctrl_Packet_Rx->Data_field[block]);
                    }
                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                	//printf("[CMD]: %x ", cmd_errors);

                case CMD_GPIO_RD:
                    if (Check_many_blocks(1))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        cmd_errors += bsp_gpio_read(&LMS_Ctrl_Packet_Tx->Data_field[block],block);
                    }
                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                case CMD_ALTERA_FPGA_GW_WR: // FPGA active serial

                    current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (
                                          LMS_Ctrl_Packet_Rx->Data_field[2] << 16)
                                      | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
                    data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

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
                        // cases 0,1,2 not implemented, fall through to default
                        // case 0,
                        // case 1:
                        // case 2:

                        // cases 3,4 share implementation
                        case 3:
                        case 4:
                            // write data to Flash from PC
                            // Start of programming? reset variables
                            if (current_portion == 0) {
                                // Gold image must be written at address 0x0
                                if (LMS_Ctrl_Packet_Rx->Data_field[0] == 3) {
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

                case CMD_PERIPHSPI_TRNSF:

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                    	/*
                        printf("[d] Block: 0x%x\n", block);

                    	printf("[RX]: %x ", LMS_Ctrl_Packet_Rx->Data_field[8 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Rx->Data_field[9 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Rx->Data_field[10 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Rx->Data_field[11 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Rx->Data_field[12 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Rx->Data_field[13 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Rx->Data_field[14 + (block * 8)]);
                    	printf("%x\n", LMS_Ctrl_Packet_Rx->Data_field[15 + (block * 8)]);
                        */
                        cmd_errors += bsp_spi_transfer(
                        		LMS_Ctrl_Packet_Rx->Header.Periph_ID,
                        		LMS_Ctrl_Packet_Rx->Data_field[0],
								&LMS_Ctrl_Packet_Rx->Data_field[8 + (block * 8)],
								LMS_Ctrl_Packet_Rx->Data_field[2],
								2,
								&LMS_Ctrl_Packet_Tx->Data_field[8 + (block * 8)]
                        );
                        /*
                    	printf("[TX]: %x ", LMS_Ctrl_Packet_Tx->Data_field[8 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Tx->Data_field[9 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Tx->Data_field[10 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Tx->Data_field[11 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Tx->Data_field[12 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Tx->Data_field[13 + (block * 8)]);
                    	printf("%x ", LMS_Ctrl_Packet_Tx->Data_field[14 + (block * 8)]);
                    	printf("%x\n", LMS_Ctrl_Packet_Tx->Data_field[15 + (block * 8)]);
                        */
                    }

                    //printf("Err=%x\n", cmd_errors);


                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                	break;

                case CMD_ADF4002_WR:
                    if(Check_many_blocks (3)) break;

                    for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
                    {
                        bsp_control_adf(1, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block*3)], false);//write data to ADF
                    }

                    // No error status possible from bsp_control_adf, so always report completed
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                // COMMAND ANALOG VALUE READ
                case CMD_ANALOG_VAL_RD:
                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        cmd_errors += bsp_analog_read(
                            LMS_Ctrl_Packet_Rx->Data_field[0 + (block)],      // Channel (8bit)
                            &LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)], // Units (8bit)
                            &LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)], // Value MSB (8bit/16bit)
                            &LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)]  // Value LSB (8bit/16bit)
                        );
                    }

                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;

                // COMMAND ANALOG VALUE WRITE
                case CMD_ANALOG_VAL_WR:
                    if (Check_many_blocks(4))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {
                        cmd_errors += bsp_analog_write(
                            LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], // Channel (8bit)
                            LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)], // Units (8bit)
                            LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)], // Value MSB (8bit)
                            LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)]  // Value LSB (8bit)
                        );
                    }
                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
                    break;


                case CMD_MEMORY_WR:

                {
                    uint8_t* rx_ptr    = &LMS_Ctrl_Packet_Rx->Data_field[0];
                    const uint32_t offset    = rx_ptr[9] | (rx_ptr[8] << 8) | (rx_ptr[7] << 16) | (rx_ptr[6] << 24);
                    const uint8_t progmode   = rx_ptr[0];
                    const uint16_t target    = rx_ptr[11] | (rx_ptr[10] << 8);
                    const uint8_t data_count = rx_ptr[5];

                    LMS_Ctrl_Packet_Tx->Header.Status = bsp_mem_write(offset, progmode, target, &rx_ptr[24], data_count);

                    break;
                }

                case CMD_MEMORY_RD:

                {
                    uint8_t* rx_ptr    = &LMS_Ctrl_Packet_Rx->Data_field[0];
                    uint8_t* tx_ptr    = &LMS_Ctrl_Packet_Tx->Data_field[0];
                    const uint32_t offset    = rx_ptr[9] | (rx_ptr[8] << 8) | (rx_ptr[7] << 16) | (rx_ptr[6] << 24);
                    const uint8_t progmode   = rx_ptr[0];
                    const uint16_t target    = rx_ptr[11] | (rx_ptr[10] << 8);
                    const uint8_t data_count = rx_ptr[5];

                    LMS_Ctrl_Packet_Tx->Header.Status = bsp_mem_read(offset, progmode, target, &tx_ptr[24], data_count);

                    break;
                }

                case CMD_BRDCSR_WR:

                    if (Check_many_blocks(16))
                        break;

                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {

                    	// Calculate the starting address of the current 16-byte block
                    	uint8_t *pCurrentBlock = (uint8_t *)&LMS_Ctrl_Packet_Rx->Data_field[block * 16];

                    	// Extract the Offset (take only 4 bytes since CSR is 4bits wide)
                    	uint32_t offset = 	((uint32_t)pCurrentBlock[4] << 24) |
                    						((uint32_t)pCurrentBlock[5] << 16) |
											((uint32_t)pCurrentBlock[6] << 8)  |
											((uint32_t)pCurrentBlock[7] << 0);

                    	//Extract the Value (take only 4 bytes since CSR is 4bits wide)
                    	uint32_t value  = 	((uint32_t)pCurrentBlock[12] << 24) |
                    						((uint32_t)pCurrentBlock[13] << 16) |
											((uint32_t)pCurrentBlock[14] << 8)  |
											((uint32_t)pCurrentBlock[15] << 0);

                    	// Check if address is within range AND aligned to 32 bits (4 bytes)
                    	//TODO: find a way to reuse CSR_SIZE define instead of constant 0x00010000
                    	if (offset >= CSR_BASE && offset < (CSR_BASE + 0x00010000) && (offset & ((CONFIG_CSR_ALIGNMENT / 8) - 1)) == 0) {
                    		// Perform CSR Write
                    		csr_write_simple(value, offset);
                    	}
                    	else {
                    		cmd_errors++;
                    		break;
                    	}


                    }

                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_RESOURCE_DENIED_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

                    break;


                case CMD_BRDCSR_RD:
                    if (Check_many_blocks(16))
                        break;


                    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++) {

                    	// Set Pointers for both RX (Request) and TX (Response)
                    	uint8_t *pCurrentBlockRx = (uint8_t *)&LMS_Ctrl_Packet_Rx->Data_field[block * 16];
                    	uint8_t *pCurrentBlockTx = (uint8_t *)&LMS_Ctrl_Packet_Tx->Data_field[block * 16];

                    	// Extract Offset from RX (Bytes 4-7)
                    	uint32_t offset = 	((uint32_t)pCurrentBlockRx[4] << 24) |
                    						((uint32_t)pCurrentBlockRx[5] << 16) |
											((uint32_t)pCurrentBlockRx[6] << 8)  |
											((uint32_t)pCurrentBlockRx[7] << 0);

                    	uint32_t value =0;
                    	// Check if address is within range AND aligned to 32 bits (4 bytes)
                    	//TODO: find a way to reuse CSR_SIZE define instead of constant 0x00010000
                    	if (offset >= CSR_BASE && offset < (CSR_BASE + 0x00010000) && (offset & ((CONFIG_CSR_ALIGNMENT / 8) - 1)) == 0) {
                    		// Perform CSR Read
                            value = csr_read_simple(offset);
                    	}
                    	else {
                    		cmd_errors++;
                    		break;
                    	}


                        // Response Offset (Echoing the Offset back, and fill MSB with zeros since CSR are 4bytes wide)
                        pCurrentBlockTx[0] = 0;
                        pCurrentBlockTx[1] = 0;
                        pCurrentBlockTx[2] = 0;
                        pCurrentBlockTx[3] = 0;
                        pCurrentBlockTx[4] = (offset >> 24) & 0xff;
                        pCurrentBlockTx[5] = (offset >> 16) & 0xff;
                        pCurrentBlockTx[6] = (offset >> 8)  & 0xff;
                        pCurrentBlockTx[7] = (offset >> 0)  & 0xff;

                        //Response Value, (CSR read value and fill MSB with zeros since CSR are 4bytes wide)
                        pCurrentBlockTx[8] = 0;
                        pCurrentBlockTx[9] = 0;
                        pCurrentBlockTx[10] = 0;
                        pCurrentBlockTx[11] = 0;
                        pCurrentBlockTx[12] = (value >> 24) & 0xff;
                        pCurrentBlockTx[13] = (value >> 16) & 0xff;
                        pCurrentBlockTx[14] = (value >> 8)  & 0xff;
                        pCurrentBlockTx[15] = (value >> 0)  & 0xff;

                    }

                    if (cmd_errors)
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_RESOURCE_DENIED_CMD;
                    else
                        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

                    break;


                default:
                    /* This is unknown request. */
                    // isHandled = CyFalse;
                    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
                    break;
            }

            // Send response to the command
            // for (int i = 0; i < 64 / sizeof(uint32_t); ++i)
            for (int i = (64 / sizeof(uint32_t)) - 1; i >= 0; --i) {
                csr_write_simple(dest[i], (CSR_CNTRL_CNTRL_ADDR + i * 4));
            }


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
        }
#ifdef CSR_LIME_TOP_LMS7002
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
#endif
    }
}
