#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "regremap.h"

static uint16_t test_val = 0;

// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-USB
void readCSR(uint8_t *address, uint8_t *regdata_array)
{
    uint16_t value = 0;
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];

    switch (addr) {
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
        //TODO: check
        value = 0x2;
        break;
    case 0x5:
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_TXCLK_EN_read() & 0x1;
        value = value | ((limetop_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_RXCLK_EN_read() & 0x1) << 1);
        break;
    case 0x7:
        value = limetop_fpgacfg_ch_en_read();
        break;
    case 0x8:
        value = limetop_fpgacfg_reg08_read() & (0x3 | (1 << 7) | (1 << 8) | (1 << 9));
        break;
    case 0x9:
        value = limetop_fpgacfg_reg09_read();
        break;
    case 0x0a:
        value = limetop_fpgacfg_reg10_read();
        break;
    case 0xF:
        value = limetop_fpgacfg_txant_pre_read();
        break;
    case 0x10:
        value = limetop_fpgacfg_txant_post_read();
        break;
    case 0x18:
        value = limetop_fpgacfg_reg18_read();
        break;
    case 0x19:
        value = limetop_rxtx_top_rx_path_pkt_size_read();
        break;
    case 0x20:
        value = csr_read_simple(clk_ctrl_addrs.c1_phase);
        break;
    case 0x21:
        value = csr_read_simple(clk_ctrl_addrs.pllcfg_done);
        value |= csr_read_simple(clk_ctrl_addrs.pllcfg_busy) << 1;
        value |= csr_read_simple(clk_ctrl_addrs.phcfg_done) << 2;
        value |= csr_read_simple(clk_ctrl_addrs.phcfg_err) << 3;
        value |= csr_read_simple(clk_ctrl_addrs.pllcfg_error) << 7;
        break;
    case 0x22:
        value = csr_read_simple(clk_ctrl_addrs.pll_lock);
        break;
    case 0x23:
        value = csr_read_simple(clk_ctrl_addrs.pllcfg_start);
        value |= csr_read_simple(clk_ctrl_addrs.phcfg_start) << 1;
        value |= csr_read_simple(clk_ctrl_addrs.pllrst_start) << 2;
        value |= csr_read_simple(clk_ctrl_addrs.pll_ind) << 3;
        value |= csr_read_simple(clk_ctrl_addrs.cnt_ind) << 8;
        value |= csr_read_simple(clk_ctrl_addrs.phcfg_updn) << 13;
        value |= csr_read_simple(clk_ctrl_addrs.phcfg_mode) << 14;
        break;
    case 0x24:
        value = csr_read_simple(clk_ctrl_addrs.cnt_phase);
        break;
    case 0x25:
        value = csr_read_simple(clk_ctrl_addrs.pllcfg_vcodiv) << 7;
        break;
    case 0x26:
        value = csr_read_simple(clk_ctrl_addrs.n_div_byp);
        value |= csr_read_simple(clk_ctrl_addrs.n_odd_div) << 1;
        value |= csr_read_simple(clk_ctrl_addrs.m_div_byp) << 2;
        value |= csr_read_simple(clk_ctrl_addrs.m_odd_div) << 3;
        break;
    case 0x27:
        value = csr_read_simple(clk_ctrl_addrs.c0_div_byp);
        value |= csr_read_simple(clk_ctrl_addrs.c0_odddiv) << 1;
        value |= csr_read_simple(clk_ctrl_addrs.c1_div_byp) << 2;
        value |= csr_read_simple(clk_ctrl_addrs.c1_odddiv) << 3;
        value |= csr_read_simple(clk_ctrl_addrs.c2_div_byp) << 4;
        value |= csr_read_simple(clk_ctrl_addrs.c2_odddiv) << 5;
        value |= csr_read_simple(clk_ctrl_addrs.c3_div_byp) << 6;
        value |= csr_read_simple(clk_ctrl_addrs.c3_odddiv) << 7;
        value |= csr_read_simple(clk_ctrl_addrs.c4_div_byp) << 8;
        value |= csr_read_simple(clk_ctrl_addrs.c4_odddiv) << 9;
        break;
    case 0x2A:
        value = csr_read_simple(clk_ctrl_addrs.n_cnt);
        break;
    case 0x2B:
        value = csr_read_simple(clk_ctrl_addrs.m_cnt);
        break;
    case 0x2E:
        value = csr_read_simple(clk_ctrl_addrs.c0_div_cnt);
        break;
    case 0x2F:
        value = csr_read_simple(clk_ctrl_addrs.c1_div_cnt);
        break;
    case 0x30:
        value = csr_read_simple(clk_ctrl_addrs.c2_div_cnt);
        break;
    case 0x31:
        value = csr_read_simple(clk_ctrl_addrs.c3_div_cnt);
        break;
    case 0x32:
        value = csr_read_simple(clk_ctrl_addrs.c4_div_cnt);
        break;
    case 0x3E:
        value = csr_read_simple(clk_ctrl_addrs.phcfg_samples);
        break;
    case 0x3F:
        value = csr_read_simple(clk_ctrl_addrs.phcfg_step);
        break;
    case 0x61:
        value = pss_tst_top_test_en_read();
        break;
    case 0x63:
        value = pss_tst_top_test_frc_err_read();
        break;
    case 0x65:
        value = pss_tst_top_test_cmplt_read();
        break;
    case 0x67:
        value = pss_tst_top_test_rez_read();
        break;
    case 0x69:
        value = pss_tst_top_fx3_clk_cnt_read();
        break;
    case 0x6A:
        value = pss_tst_top_si_clk7_cnt_read();
        break;
    case 0x6B:
        value = pss_tst_top_si_clk6_cnt_read();
        break;
    case 0x6C:
        value = pss_tst_top_si_clk5_cnt_read();
        break;
    case 0x6E:
        value = pss_tst_top_si_clk3_cnt_read();
        break;
    case 0x6F:
        value = pss_tst_top_si_clk2_cnt_read();
        break;
    case 0x70:
        value = pss_tst_top_si_clk1_cnt_read();
        break;
    case 0x71:
        value = pss_tst_top_si_clk0_cnt_read();
        break;
    case 0x72:
        value = pss_tst_top_lmk_clk_cnt_read() & 0xFFFF;
        break;
    case 0x73:
        value = (pss_tst_top_lmk_clk_cnt_read() >> 16) & 0xFFFF;
        break;
    case 0x74:
        value = pss_tst_top_adf_muxout_cnt_read();
        break;
    case 0x7A:
        // Bit0 - Test Complete
        // Bit1 - Test Pass
        // Bit2 - Test Fail
        value = ((pss_tst_top_test_cmplt_read()>>5) & 0x1) |
                ((pss_tst_top_test_rez_read()>>4) & 0x2) |
                    ((pss_tst_top_ddr2_2_tst_fail_read() & 0x1)<<2);
        break;
    case 0x7B:
        value = limetop_rxtx_top_ddr2_1_pnf_per_bit_l_read();
        break;
    case 0x7C:
        value = limetop_rxtx_top_ddr2_1_pnf_per_bit_h_read();
        break;
    case 0xC0:
        // value = periphcfg_BOARD_GPIO_OVRD_read();
        break;
    case 0xC4:
        // value = periphcfg_BOARD_GPIO_DIR_read();
        break;
    case 0xC6:
        // value = periphcfg_BOARD_GPIO_VAL_read();
        break;
    case 0xCA:
        // value = periphcfg_PERIPH_INPUT_SEL_0_read();
        break;
    case 0xD2:
        // value = periphcfg_PERIPH_EN_read();
        break;
    case 0xD3:
        // value = periphcfg_PERIPH_SEL_read();
        break;
    default:
        break;
    }

    regdata_array[0] = (uint8_t)(value & 0xFF);        // Byte 0 (LSB)
    regdata_array[1] = (uint8_t)((value >> 8) & 0xFF); // Byte 1
}

// To write and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-USB
void writeCSR(uint8_t *address, uint8_t *wrdata_array)
{
    uint16_t value = ((uint16_t)wrdata_array[0] << 8) | wrdata_array[1];
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];
    uint32_t reg;

    switch (addr) {
    case 0x05:
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_TXCLK_EN_write((value & 0x1) >> 0);
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_RXCLK_EN_write((value & 0x2) >> 1);
        break;
    case 0x7:
        limetop_fpgacfg_ch_en_write(value);
        break;
    case 0x8:
        reg = limetop_fpgacfg_reg08_read();
        reg &= ~((1 << CSR_LIMETOP_FPGACFG_REG08_SYNCH_DIS_OFFSET) |
                 (1 << CSR_LIMETOP_FPGACFG_REG08_MIMO_INT_EN_OFFSET) |
                 (1 << CSR_LIMETOP_FPGACFG_REG08_TRXIQ_PULSE_OFFSET) | (0x3));
        reg |= (value & 0x03);  // smpl_width
        reg |= (value & 0x80);  // trxiq_pulse
        reg |= (value & 0x100); // mimo_int_en
        reg |= (value & 0x200); // sync_dis
        limetop_fpgacfg_reg08_write(reg);
        break;
    case 0x9:
        limetop_fpgacfg_reg09_write(value);
        break;
    case 0x0a:
        limetop_fpgacfg_reg10_write(value);
        break;
    case 0xF:
        limetop_fpgacfg_txant_pre_write(value);
        break;
    case 0x10:
        limetop_fpgacfg_txant_post_write(value);
        break;
    case 0x13:
        limetop_lms7002_top_lms1_write(value);
        break;
    case 0x18:
        limetop_fpgacfg_reg18_write(value);
        break;
    case 0x19:
        limetop_rxtx_top_rx_path_pkt_size_write(value);
        break;
    case 0x20:
        csr_write_simple(value & 0x1FF, clk_ctrl_addrs.c1_phase);
        break;
    case 0x23:
        csr_write_simple(value & 0x1, clk_ctrl_addrs.pllcfg_start);
        csr_write_simple((value >> 1) & 0x1, clk_ctrl_addrs.phcfg_start);
        csr_write_simple((value >> 2) & 0x1, clk_ctrl_addrs.pllrst_start);
        csr_write_simple((value >> 3) & 0x1F, clk_ctrl_addrs.pll_ind);
        csr_write_simple((value >> 8) & 0x1F, clk_ctrl_addrs.cnt_ind);
        csr_write_simple((value >> 13) & 0x1, clk_ctrl_addrs.phcfg_updn);
        csr_write_simple((value >> 14) & 0x1, clk_ctrl_addrs.phcfg_mode);
        break;
    case 0x24:
        csr_write_simple(value, clk_ctrl_addrs.cnt_phase);
        break;
    case 0x25:
        csr_write_simple((value >> 7) & 0x1, clk_ctrl_addrs.pllcfg_vcodiv);
        break;
    case 0x26:
        csr_write_simple(value & 0x1, clk_ctrl_addrs.n_div_byp);
        csr_write_simple((value >> 1) & 0x1, clk_ctrl_addrs.n_odd_div);
        csr_write_simple((value >> 2) & 0x1, clk_ctrl_addrs.m_div_byp);
        csr_write_simple((value >> 3) & 0x1, clk_ctrl_addrs.m_odd_div);
        break;
    case 0x27:
        csr_write_simple(value & 0x1, clk_ctrl_addrs.c0_div_byp);
        csr_write_simple((value >> 1) & 0x1, clk_ctrl_addrs.c0_odddiv);
        csr_write_simple((value >> 2) & 0x1, clk_ctrl_addrs.c1_div_byp);
        csr_write_simple((value >> 3) & 0x1, clk_ctrl_addrs.c1_odddiv);
        csr_write_simple((value >> 4) & 0x1, clk_ctrl_addrs.c2_div_byp);
        csr_write_simple((value >> 5) & 0x1, clk_ctrl_addrs.c2_odddiv);
        csr_write_simple((value >> 6) & 0x1, clk_ctrl_addrs.c3_div_byp);
        csr_write_simple((value >> 7) & 0x1, clk_ctrl_addrs.c3_odddiv);
        csr_write_simple((value >> 8) & 0x1, clk_ctrl_addrs.c4_div_byp);
        csr_write_simple((value >> 9) & 0x1, clk_ctrl_addrs.c4_odddiv);
        break;
    case 0x2A:
        csr_write_simple(value, clk_ctrl_addrs.n_cnt);
        break;
    case 0x2B:
        csr_write_simple(value, clk_ctrl_addrs.m_cnt);
        break;
    case 0x2E:
        csr_write_simple(value, clk_ctrl_addrs.c0_div_cnt);
        break;
    case 0x2F:
        csr_write_simple(value, clk_ctrl_addrs.c1_div_cnt);
        break;
    case 0x30:
        csr_write_simple(value, clk_ctrl_addrs.c2_div_cnt);
        break;
    case 0x31:
        csr_write_simple(value, clk_ctrl_addrs.c3_div_cnt);
        break;
    case 0x32:
        csr_write_simple(value, clk_ctrl_addrs.c4_div_cnt);
        break;
    case 0x3E:
        csr_write_simple(value, clk_ctrl_addrs.phcfg_samples);
        break;
    case 0x3f:
        csr_write_simple(value, clk_ctrl_addrs.phcfg_step);
        break;
    case 0x61:
        pss_tst_top_test_en_write(value);
        break;
    case 0x63:
        pss_tst_top_test_frc_err_write(value);
        break;
    case 0xC0:
        // periphcfg_BOARD_GPIO_OVRD_write(value);
        break;
    case 0xC4:
        // periphcfg_BOARD_GPIO_DIR_write(value);
        break;
    case 0xC6:
        // periphcfg_BOARD_GPIO_VAL_write(value);
        break;
    case 0xCA:
        // periphcfg_PERIPH_INPUT_SEL_0_write(value);
        break;
    case 0xD2:
        // periphcfg_PERIPH_EN_write(value);
        break;
    case 0xD3:
        // periphcfg_PERIPH_SEL_write(value);
        break;
    // TODO: Implement register remapping
    default:
        break;
    }
}
