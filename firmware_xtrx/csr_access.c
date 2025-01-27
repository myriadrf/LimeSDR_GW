/*
-- ----------------------------------------------------------------------------
-- FILE        : csr_access.c
-- DESCRIPTION : CSR Write/Read Accesses Translation.
-- DATE        : 2015-2024
-- AUTHOR(s)   : Lime Microsystems
-- REVISION    : -
-- ----------------------------------------------------------------------------
*/

#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>

#include "csr_access.h"

void fpgacfg_write(uint16_t addr, uint8_t *wdata)
{
	uint32_t value = (0x0000FFFF & (((uint32_t)wdata[0] << 8) | ((uint32_t)wdata[1])));

	switch (addr)
	{
	case 0x4:
		fpgacfg_phase_reg_sel_write(value);
		break;
	case 0x5:
		fpgacfg_drct_clk_en_write(value);
		break;
	case 0x6:
		fpgacfg_load_phase_write(value);
		break;
	case 0x7:
		fpgacfg_ch_en_write(value);
		break;
	case 0x8:
		fpgacfg_reg08_write(value);
		break;
	case 0x9:
		fpgacfg_reg09_write(value);
		break;
	case 0xa:
		fpgacfg_reg10_write(value);
		break;
	case 0xC:
		fpgacfg_wfm_ch_en_write(value);
		break;
	case 0xD:
		fpgacfg_reg13_write(value);
		break;
	case 0xE:
		fpgacfg_wfm_smpl_width_write(value);
		break;
	case 0xF:
		fpgacfg_sync_size_write(value);
		break;
	case 0x10:
		fpgacfg_txant_pre_write(value);
		break;
	case 0x11:
		fpgacfg_txant_post_write(value);
		break;
	case 0x12:
		fpgacfg_spi_ss_write(value);
		break;
	case 0x13:
		lms7002_top_lms1_write(value);
		break;
	case 0x17:
		main_gpio_write(value);
		break;
	case 0x1A:
		general_periph_fpga_led_ctrl_write(value);
		break;
	case 0x1C:
		general_periph_FX3_LED_CTRL_write(value);
		break;
	case 0x1D:
		fpgacfg_clk_ena_write(value);
		break;
	case 0x1E:
		fpgacfg_sync_pulse_period_write(value);
		break;
	default:
		printf("FWE: %04x\n", addr);
	}
}

void fpgacfg_read(uint16_t addr, uint8_t *rdata)
{
	uint32_t value = 0;

	switch (addr)
	{
	case 0x0:
		value = fpgacfg_board_id_read();
		break;
	case 0x1:
		value = fpgacfg_major_rev_read();
		break;
	case 0x2:
		value = fpgacfg_compile_rev_read();
		break;
	case 0x3:
		value = fpgacfg_bom_hw_ver_read();
		break;
	case 0x4:
		value = fpgacfg_phase_reg_sel_read();
		break;
	case 0x5:
		value = fpgacfg_drct_clk_en_read();
		break;
	case 0x6:
		value = fpgacfg_load_phase_read();
		break;
	case 0x7:
		value = fpgacfg_ch_en_read();
		break;
	case 0x8:
		value = fpgacfg_reg08_read();
		break;
	case 0x9:
		value = fpgacfg_reg09_read();
		break;
	case 0xa:
		value = fpgacfg_reg10_read();
		break;
	case 0xC:
		value = fpgacfg_wfm_ch_en_read();
		break;
	case 0xD:
		value = fpgacfg_reg13_read();
		break;
	case 0xE:
		value = fpgacfg_wfm_smpl_width_read();
		break;
	case 0xF:
		value = fpgacfg_sync_size_read();
		break;
	case 0x10:
		value = fpgacfg_txant_pre_read();
		break;
	case 0x11:
		value = fpgacfg_txant_post_read();
		break;
	case 0x12:
		value = fpgacfg_spi_ss_read();
		break;
	case 0x13:
		value = lms7002_top_lms1_read();
		break;
	case 0x17:
		value = main_gpio_read();
		break;
	case 0x1A:
		value = general_periph_fpga_led_ctrl_read();
		break;
	case 0x1C:
		value = general_periph_FX3_LED_CTRL_read();
		break;
	case 0x1D:
		value = fpgacfg_clk_ena_read();
		break;
	case 0x1E:
		value = fpgacfg_sync_pulse_period_read();
		break;
	default:
		printf("FRE: %04x\n", addr);
	}
	rdata[0] = (uint8_t)((value >> 0) & 0xff);
	rdata[1] = (uint8_t)((value >> 8) & 0xff);
}

void pllcfg_write(uint16_t addr, uint8_t *wdata)
{
	uint32_t value = (0x0000FFFF & (((uint32_t)wdata[0] << 8) | ((uint32_t)wdata[1])));

	switch (addr)
	{
	case 0x3:
		lms7002_top_reg03_write(value);
		break;
	/* 0x4-0x12 - No present on XTRX/LimeMSDR-Mini */
	case 0x4:
	case 0x5:
	case 0x6:
	case 0x7:
	case 0x8:
	case 0xa:
	case 0xb:
	case 0xe:
	case 0xf:
	case 0x10:
	case 0x11:
	case 0x12:
		break;
	case 0x1E:
		pllcfg_auto_phcfg_smpls_write(value);
		break;
	case 0x1f:
		pllcfg_auto_phcfg_step_write(value);
		break;
	default:
		printf("Write error :unhandled register %04d\n", addr);
	}
}

void pllcfg_read(uint16_t addr, uint8_t *rdata)
{
	uint32_t value = 0;

	switch (addr)
	{
	case 0x01:
		value = lms7002_top_reg01_read();
		break;
	case 0x02:
		value = pllcfg_pll_lock_read();
		break;
	case 0x05:
		value = 0b110110000;
		break;
	default:
		printf("Read error: unhandled register %d\n", addr);
	}
	rdata[0] = (uint8_t)((value >> 0) & 0xff);
	rdata[1] = (uint8_t)((value >> 8) & 0xff);
}

void tstcfg_write(uint16_t addr, uint8_t *wdata)
{
	uint32_t value = (0x0000FFFF & (((uint32_t)wdata[0] << 8) | ((uint32_t)wdata[1])));

	switch (addr)
	{
	case 0x1:
		tst_top_test_en_write(value);
		break;
	case 0x3:
		tst_top_test_frc_err_write(value);
		break;
	case 0x1D:
		tst_top_tx_tst_i_write(value);
		break;
	case 0x1E:
		tst_top_tx_tst_q_write(value);
		break;
	default:
		printf("TstCfg Write error :unhandled register %d\n", addr);
	}
}

void tstcfg_read(uint16_t addr, uint8_t *rdata)
{
	uint32_t value = 0;

	switch (addr)
	{
	case 0x5:
		value = tst_top_test_cmplt_read();
		break;
	case 0x7:
		value = tst_top_test_rez_read();
		break;
	case 0x9:
		value = tst_top_fx3_clk_cnt_read();
		break;
	/* 0xa-0x11 - No present on XTRX/LimeMSDR-Mini */
	case 0xa:
	case 0xb:
	case 0xc:
	case 0xd:
	case 0xf:
	case 0x10:
	case 0x11:
		value = 0;
		break;
	case 0x12:
		value = tst_top_lmk_clk_cnt0_read();
		break;
	case 0x13:
		value = tst_top_lmk_clk_cnt1_read();
		break;
	case 0x14:
		value = tst_top_adf_cnt_read();
		break;
	default:
		printf("TstCfg Read error: unhandled register %d\n", addr);
	}
	rdata[0] = (uint8_t)((value >> 0) & 0xff);
	rdata[1] = (uint8_t)((value >> 8) & 0xff);
}

void periphcfg_write(uint16_t addr, uint8_t *wdata)
{
	uint32_t value = (0x0000FFFF & (((uint32_t)wdata[0] << 8) | ((uint32_t)wdata[1])));

	switch (addr)
	{
	case 0x0:
		general_periph_board_gpio_OVRD_write(value);
		break;
	case 0x4:
		general_periph_board_gpio_DIR_write(value);
		break;
	case 0x6:
		general_periph_board_gpio_VAL_write(value);
		break;
	case 0x0C:
		general_periph_periph_output_OVRD_0_write(value);
		break;
	case 0x0D:
		general_periph_periph_output_VAL_0_write(value);
		break;
	case 0x0E:
		general_periph_periph_output_OVRD_1_write(value);
		break;
	case 0x0F:
		general_periph_periph_output_VAL_1_write(value);
		break;
	default:
		printf("PeriphCfg Write error :unhandled register %d\n", addr);
	}
}

void periphcfg_read(uint16_t addr, uint8_t *rdata)
{
	uint32_t value = 0;

	switch (addr)
	{
	case 0x2:
		value = general_periph_board_gpio_RD_read();
		break;
	case 0x4:
		value = general_periph_board_gpio_DIR_read();
		break;
	case 0x6:
		value = general_periph_board_gpio_VAL_read();
		break;
	case 0x08:
		general_periph_periph_input_RD_0_read();
		break;
	case 0x09:
		general_periph_periph_input_RD_1_read();
		break;
	default:
		printf("PeriphCfg Read error: unhandled register %d\n", addr);
	}
	rdata[0] = (uint8_t)((value >> 0) & 0xff);
	rdata[1] = (uint8_t)((value >> 8) & 0xff);
}
