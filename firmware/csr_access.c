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
#include <generated/soc.h>

#include "csr_access.h"

void fpgacfg_write(uint16_t addr, uint8_t *wdata)
{
	uint32_t value = (0x0000FFFF & (((uint32_t)wdata[0] << 8) | ((uint32_t)wdata[1])));

	switch (addr)
	{
	case 0x4:
		limetop_fpgacfg_phase_reg_sel_write(value);
		break;
	case 0x5:
		limetop_fpgacfg_drct_clk_en_write(value);
		break;
	case 0x6:
		limetop_fpgacfg_load_phase_write(value);
		break;
	case 0x7:
		limetop_fpgacfg_ch_en_write(value);
		break;
	case 0x8:
		limetop_fpgacfg_reg08_write(value);
		break;
	case 0x9:
		limetop_fpgacfg_reg09_write(value);
		break;
	case 0xa:
		limetop_fpgacfg_reg10_write(value);
		break;
	case 0xC:
		limetop_fpgacfg_wfm_ch_en_write(value);
		break;
	case 0xD:
		limetop_fpgacfg_reg13_write(value);
		break;
	case 0xE:
		limetop_fpgacfg_wfm_smpl_width_write(value);
		break;
	case 0xF:
		limetop_fpgacfg_sync_size_write(value);
		break;
	case 0x10:
		limetop_fpgacfg_txant_pre_write(value);
		break;
	case 0x11:
		limetop_fpgacfg_txant_post_write(value);
		break;
	case 0x12:
		limetop_fpgacfg_spi_ss_write(value);
		break;
#ifdef WITH_LMS7002
	case 0x13:
		limetop_lms7002_top_lms1_write(value);
		break;
#endif
	case 0x17:
		limetop_gpio_write(value);
		break;
	case 0x1A:
		limetop_general_periph_fpga_led_ctrl_write(value);
		break;
	case 0x1C:
		limetop_general_periph_FX3_LED_CTRL_write(value);
		break;
	case 0x1D:
		limetop_fpgacfg_clk_ena_write(value);
		break;
	case 0x1E:
		limetop_fpgacfg_sync_pulse_period_write(value);
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
		value = limetop_fpgacfg_board_id_read();
		break;
	case 0x1:
		value = limetop_fpgacfg_major_rev_read();
		break;
	case 0x2:
		value = limetop_fpgacfg_compile_rev_read();
		break;
	case 0x3:
		value = limetop_fpgacfg_bom_hw_ver_read();
		break;
	case 0x4:
		value = limetop_fpgacfg_phase_reg_sel_read();
		break;
	case 0x5:
		value = limetop_fpgacfg_drct_clk_en_read();
		break;
	case 0x6:
		value = limetop_fpgacfg_load_phase_read();
		break;
	case 0x7:
		value = limetop_fpgacfg_ch_en_read();
		break;
	case 0x8:
		value = limetop_fpgacfg_reg08_read();
		break;
	case 0x9:
		value = limetop_fpgacfg_reg09_read();
		break;
	case 0xa:
		value = limetop_fpgacfg_reg10_read();
		break;
	case 0xC:
		value = limetop_fpgacfg_wfm_ch_en_read();
		break;
	case 0xD:
		value = limetop_fpgacfg_reg13_read();
		break;
	case 0xE:
		value = limetop_fpgacfg_wfm_smpl_width_read();
		break;
	case 0xF:
		value = limetop_fpgacfg_sync_size_read();
		break;
	case 0x10:
		value = limetop_fpgacfg_txant_pre_read();
		break;
	case 0x11:
		value = limetop_fpgacfg_txant_post_read();
		break;
	case 0x12:
		value = limetop_fpgacfg_spi_ss_read();
		break;
#ifdef WITH_LMS7002
	case 0x13:
		value = limetop_lms7002_top_lms1_read();
		break;
#endif
	case 0x17:
		value = limetop_gpio_read();
		break;
	case 0x1A:
		value = limetop_general_periph_fpga_led_ctrl_read();
		break;
	case 0x1C:
		value = limetop_general_periph_FX3_LED_CTRL_read();
		break;
	case 0x1D:
		value = limetop_fpgacfg_clk_ena_read();
		break;
	case 0x1E:
		value = limetop_fpgacfg_sync_pulse_period_read();
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
#ifdef WITH_LMS7002
	case 0x3:
		limetop_lms7002_top_reg03_write(value);
#ifdef LIMESDR_MINI_V1
		limetop_pllcfg_reg03_write(value);
#endif
		break;
#endif
	/* 0x4-0x12 - No present on XTRX/LimeMSDR-Mini */
#ifdef LIMESDR_MINI_V1
	case 0x4:
		limetop_pllcfg_cnt_phase_write(value);
		break;
	case 0x5:
		limetop_pllcfg_reg05_write(value);
		break;
	case 0x6:
		limetop_pllcfg_reg06_write(value);
		break;
	case 0x7:
		limetop_pllcfg_reg07_write(value);
		break;
	case 0x8:
		break;
	case 0xa:
		limetop_pllcfg_n_cnt_write(value);
		break;
	case 0xb:
		limetop_pllcfg_m_cnt_write(value);
		break;
	case 0xe:
		limetop_pllcfg_c0_cnt_write(value);
		break;
	case 0xf:
		limetop_pllcfg_c1_cnt_write(value);
		break;
	case 0x10:
		limetop_pllcfg_c2_cnt_write(value);
		break;
	case 0x11:
		limetop_pllcfg_c3_cnt_write(value);
		break;
	case 0x12:
		limetop_pllcfg_c4_cnt_write(value);
		break;
#else
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
#endif
	case 0x1E:
		limetop_pllcfg_auto_phcfg_smpls_write(value);
		break;
	case 0x1f:
		limetop_pllcfg_auto_phcfg_step_write(value);
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
#ifdef WITH_LMS7002
	case 0x01:
#ifdef LIMESDR_MINI_V1
		value = limetop_pllcfg_reg01_read();
#else
		value = limetop_lms7002_top_reg01_read();
#endif
		break;
#endif
	case 0x02:
#ifdef LIMESDR_MINI_V1
		value = limetop_pllcfg_pll_lock_read();
#else
		value = limetop_pllcfg_pll_lock_read();
#endif
		break;
	case 0x05:
#ifdef LIMESDR_MINI_V1
		value = limetop_pllcfg_reg05_read();
#else
		value = 0b110110000;
#endif
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
		limetop_tst_top_test_en_write(value);
		break;
	case 0x3:
		limetop_tst_top_test_frc_err_write(value);
		break;
#ifdef LIMESDR_MINI_V1
	case 0x4:
		value = limetop_pllcfg_cnt_phase_read();
		break;
	case 0x5:
		value = limetop_pllcfg_reg05_read();
		break;
	case 0x6:
		value = limetop_pllcfg_reg06_read();
		break;
	case 0x7:
		value = limetop_pllcfg_reg07_read();
		break;
	case 0x8:
		break;
	case 0xa:
		value = limetop_pllcfg_n_cnt_read();
		break;
	case 0xb:
		value = limetop_pllcfg_m_cnt_read();
		break;
	case 0xe:
		value = limetop_pllcfg_c0_cnt_read();
		break;
	case 0xf:
		value = limetop_pllcfg_c1_cnt_read();
		break;
	case 0x10:
		value = limetop_pllcfg_c2_cnt_read();
		break;
	case 0x11:
		value = limetop_pllcfg_c3_cnt_read();
		break;
	case 0x12:
		value = limetop_pllcfg_c4_cnt_read();
		break;
#endif
	case 0x1D:
		limetop_tst_top_tx_tst_i_write(value);
		break;
	case 0x1E:
		limetop_tst_top_tx_tst_q_write(value);
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
		value = limetop_tst_top_test_cmplt_read();
		break;
	case 0x7:
		value = limetop_tst_top_test_rez_read();
		break;
	case 0x9:
		value = limetop_tst_top_fx3_clk_cnt_read();
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
		value = limetop_tst_top_lmk_clk_cnt0_read();
		break;
	case 0x13:
		value = limetop_tst_top_lmk_clk_cnt1_read();
		break;
	case 0x14:
		value = limetop_tst_top_adf_cnt_read();
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
		limetop_general_periph_board_gpio_OVRD_write(value);
		break;
	case 0x4:
		limetop_general_periph_board_gpio_DIR_write(value);
		break;
	case 0x6:
		limetop_general_periph_board_gpio_VAL_write(value);
		break;
	case 0x0C:
		limetop_general_periph_periph_output_OVRD_0_write(value);
		break;
	case 0x0D:
		limetop_general_periph_periph_output_VAL_0_write(value);
		break;
	case 0x0E:
		limetop_general_periph_periph_output_OVRD_1_write(value);
		break;
	case 0x0F:
		limetop_general_periph_periph_output_VAL_1_write(value);
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
	case 0x0:
		value = limetop_general_periph_board_gpio_OVRD_read();
		break;
	case 0x2:
		value = limetop_general_periph_board_gpio_RD_read();
		break;
	case 0x4:
		value = limetop_general_periph_board_gpio_DIR_read();
		break;
	case 0x6:
		value = limetop_general_periph_board_gpio_VAL_read();
		break;
	case 0x08:
		value = limetop_general_periph_periph_input_RD_0_read();
		break;
	case 0x09:
		value = limetop_general_periph_periph_input_RD_1_read();
		break;
	default:
		printf("PeriphCfg Read error: unhandled register %d\n", addr);
	}
	rdata[0] = (uint8_t)((value >> 0) & 0xff);
	rdata[1] = (uint8_t)((value >> 8) & 0xff);
}
