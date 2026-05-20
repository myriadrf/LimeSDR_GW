#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "regremap.h"
#include "../LimeSDR_XTRX/regremap.h"

#include "../../../deps/litex/litex/soc/software/include/hw/common.h"

// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-Mini-V1
void readCSR(uint8_t *address, uint8_t *regdata_array)
{
    uint16_t value = 0;
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];

    switch (addr) {
    case 0x00:
        value = limetop_fpgacfg_board_id_read();
        break;
    case 0x01:
        value = limetop_fpgacfg_major_rev_read();
        break;
    case 0x02:
        value = limetop_fpgacfg_compile_rev_read();
        break;
    case 0x03:
        value = limetop_fpgacfg_bom_hw_ver_read();
        break;
    case 0x04:
        value = limetop_fpgacfg_phase_reg_sel_read();
        break;
    case 0x05:
        value = limetop_fpgacfg_drct_clk_en_read();
        break;
    case 0x06:
        value = limetop_fpgacfg_load_phase_read();
        break;
    case 0x07:
        value = limetop_fpgacfg_ch_en_read();
        break;
    case 0x08:
        value = limetop_fpgacfg_reg08_read();
        break;
    case 0x09:
        value = limetop_fpgacfg_reg09_read();
        break;
    case 0x0a:
        value = limetop_fpgacfg_reg10_read();
        break;
    case 0x0c:
        value = limetop_fpgacfg_wfm_ch_en_read();
        break;
    case 0x0d:
        value = limetop_fpgacfg_reg13_read();
        break;
    case 0x0e:
        value = limetop_fpgacfg_wfm_smpl_width_read();
        break;
    case 0x0f:
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
    case 0x17:
        value = main_gpio_read();
        break;
    case 0x1a:
        value = general_periph_fpga_led_ctrl_read();
        break;
    case 0x1c:
        value = general_periph_FX3_LED_CTRL_read();
        break;
    case 0x1d:
        value = limetop_fpgacfg_clk_ena_read();
        break;
    case 0x1e:
        value = limetop_fpgacfg_sync_pulse_period_read();
        break;
#ifdef WITH_LMS7002
        case 0x21:
            value = csr_read_simple(clk_ctrl_addrs.pllcfg_done);
            value |= csr_read_simple(clk_ctrl_addrs.pllcfg_busy) << 1;
            value |= csr_read_simple(clk_ctrl_addrs.phcfg_done) << 2;
            value |= csr_read_simple(clk_ctrl_addrs.phcfg_err) << 3;
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
    #endif
    case 0x65:
        value = tst_top_test_cmplt_read();
        break;
    case 0x67:
        value = tst_top_test_rez_read();
        break;
    case 0x69:
        value = tst_top_fx3_clk_cnt_read();
        break;
    case 0x6a:
    case 0x6b:
    case 0x6c:
    case 0x6d:
    case 0x6f:
    case 0x70:
    case 0x71:
        value = 0;
        break;
    case 0x72:
        value = tst_top_lmk_clk_cnt0_read();
        break;
    case 0x73:
        value = tst_top_lmk_clk_cnt1_read();
        break;
    case 0x74:
        value = tst_top_adf_cnt_read();
        break;

    case 0xc0:
        value = general_periph_board_gpio_OVRD_read();
        break;
    case 0xc2:
        value = general_periph_board_gpio_RD_read();
        break;
    case 0xc4:
        value = general_periph_board_gpio_DIR_read();
        break;
    case 0xc6:
        value = general_periph_board_gpio_VAL_read();
        break;
    case 0xc8:
        value = general_periph_periph_input_RD_0_read();
        break;
    case 0xc9:
        value = general_periph_periph_input_RD_1_read();
        break;
    case 0xcc:
        value = general_periph_periph_output_OVRD_0_read();
        break;
    case 0xcd:
        value = general_periph_periph_output_VAL_0_read();
        break;
    case 0xce:
        value = general_periph_periph_output_OVRD_1_read();
        break;
    case 0xcf:
        value = general_periph_periph_output_VAL_1_read();
        break;

    default:
        printf("FRE: %04x\n", addr);
        break;
    }

    regdata_array[0] = (uint8_t)(value & 0xFF);        // Byte 0 (LSB)
    regdata_array[1] = (uint8_t)((value >> 8) & 0xFF); // Byte 1
}

// To write and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-Mini-V1
void writeCSR(uint8_t *address, uint8_t *wrdata_array)
{
    uint16_t value = (((uint16_t)wrdata_array[0] << 8) | ((uint16_t)wrdata_array[1]));
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];

    switch (addr) {
    case 0x04:
        limetop_fpgacfg_phase_reg_sel_write(value);
        break;
    case 0x05:
        limetop_fpgacfg_drct_clk_en_write(value);
        break;
    case 0x06:
        limetop_fpgacfg_load_phase_write(value);
        break;
    case 0x07:
        limetop_fpgacfg_ch_en_write(value);
        break;
    case 0x08:
        limetop_fpgacfg_reg08_write(value);
        break;
    case 0x09:
        limetop_fpgacfg_reg09_write(value);
        break;
    case 0x0a:
        limetop_fpgacfg_reg10_write(value);
        break;
    case 0x0c:
        limetop_fpgacfg_wfm_ch_en_write(value);
        break;
    case 0x0d:
        limetop_fpgacfg_reg13_write(value);
        break;
    case 0x0e:
        limetop_fpgacfg_wfm_smpl_width_write(value);
        break;
    case 0x0f:
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
    case 0x17:
        main_gpio_write(value);
        break;
    case 0x1a:
        general_periph_fpga_led_ctrl_write(value);
        break;
    case 0x1c:
        general_periph_FX3_LED_CTRL_write(value);
        break;
    case 0x1d:
        limetop_fpgacfg_clk_ena_write(value);
        break;
    case 0x1e:
        limetop_fpgacfg_sync_pulse_period_write(value);
        break;

#ifdef WITH_LMS7002
    case 0x13:
        limetop_lms7002_top_lms1_write(value);
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
#endif
    case 0x61:
        tst_top_test_en_write(value);
        break;
    case 0x63:
        tst_top_test_frc_err_write(value);
        break;
    case 0x7d:
        tst_top_tx_tst_i_write(value);
        break;
    case 0x7e:
        tst_top_tx_tst_q_write(value);
        break;

    case 0xc0:
        general_periph_board_gpio_OVRD_write(value);
        break;
    case 0xc4:
        general_periph_board_gpio_DIR_write(value);
        break;
    case 0xc6:
        general_periph_board_gpio_VAL_write(value);
        break;
    case 0xcc:
        general_periph_periph_output_OVRD_0_write(value);
        break;
    case 0xcd:
        general_periph_periph_output_VAL_0_write(value);
        break;
    case 0xce:
        general_periph_periph_output_OVRD_1_write(value);
        break;
    case 0xcf:
        general_periph_periph_output_VAL_1_write(value);
        break;

    default:
        printf("FWE: %04x\n", addr);
        break;
    }
}