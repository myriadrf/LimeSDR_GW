#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "regremap.h"

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
        value = limetop_gpio_read();
        break;
    case 0x1a:
        value = limetop_general_periph_fpga_led_ctrl_read();
        break;
    case 0x1c:
        value = limetop_general_periph_FX3_LED_CTRL_read();
        break;
    case 0x1d:
        value = limetop_fpgacfg_clk_ena_read();
        break;
    case 0x1e:
        value = limetop_fpgacfg_sync_pulse_period_read();
        break;

#ifdef WITH_LMS7002
    case 0x21:
        value = limetop_pllcfg_reg01_read();
        break;
#endif
    case 0x22:
        value = limetop_pllcfg_pll_lock_read();
        break;
    case 0x25:
        value = limetop_pllcfg_reg05_read();
        break;

    case 0x65:
        value = limetop_tst_top_test_cmplt_read();
        break;
    case 0x67:
        value = limetop_tst_top_test_rez_read();
        break;
    case 0x69:
        value = limetop_tst_top_fx3_clk_cnt_read();
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
        value = limetop_tst_top_lmk_clk_cnt0_read();
        break;
    case 0x73:
        value = limetop_tst_top_lmk_clk_cnt1_read();
        break;
    case 0x74:
        value = limetop_tst_top_adf_cnt_read();
        break;

    case 0xc0:
        value = limetop_general_periph_board_gpio_OVRD_read();
        break;
    case 0xc2:
        value = limetop_general_periph_board_gpio_RD_read();
        break;
    case 0xc4:
        value = limetop_general_periph_board_gpio_DIR_read();
        break;
    case 0xc6:
        value = limetop_general_periph_board_gpio_VAL_read();
        break;
    case 0xc8:
        value = limetop_general_periph_periph_input_RD_0_read();
        break;
    case 0xc9:
        value = limetop_general_periph_periph_input_RD_1_read();
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
#ifdef WITH_LMS7002
    case 0x13:
        limetop_lms7002_top_lms1_write(value);
        break;
#endif
    case 0x17:
        limetop_gpio_write(value);
        break;
    case 0x1a:
        limetop_general_periph_fpga_led_ctrl_write(value);
        break;
    case 0x1c:
        limetop_general_periph_FX3_LED_CTRL_write(value);
        break;
    case 0x1d:
        limetop_fpgacfg_clk_ena_write(value);
        break;
    case 0x1e:
        limetop_fpgacfg_sync_pulse_period_write(value);
        break;

#ifdef WITH_LMS7002
    case 0x23:
        limetop_lms7002_top_reg03_write(value);
        limetop_pllcfg_reg03_write(value);
        break;
#endif
    case 0x24:
        limetop_pllcfg_cnt_phase_write(value);
        break;
    case 0x25:
        limetop_pllcfg_reg05_write(value);
        break;
    case 0x26:
        limetop_pllcfg_reg06_write(value);
        break;
    case 0x27:
        limetop_pllcfg_reg07_write(value);
        break;
    case 0x28:
        break;
    case 0x2a:
        limetop_pllcfg_n_cnt_write(value);
        break;
    case 0x2b:
        limetop_pllcfg_m_cnt_write(value);
        break;
    case 0x2e:
        limetop_pllcfg_c0_cnt_write(value);
        break;
    case 0x2f:
        limetop_pllcfg_c1_cnt_write(value);
        break;
    case 0x30:
        limetop_pllcfg_c2_cnt_write(value);
        break;
    case 0x31:
        limetop_pllcfg_c3_cnt_write(value);
        break;
    case 0x32:
        limetop_pllcfg_c4_cnt_write(value);
        break;
    case 0x3e:
        limetop_pllcfg_auto_phcfg_smpls_write(value);
        break;
    case 0x3f:
        limetop_pllcfg_auto_phcfg_step_write(value);
        break;

    case 0x61:
        limetop_tst_top_test_en_write(value);
        break;
    case 0x63:
        limetop_tst_top_test_frc_err_write(value);
        break;
    case 0x64:
        limetop_pllcfg_cnt_phase_read();
        break; // Original bug preserved
    case 0x65:
        limetop_pllcfg_reg05_read();
        break;
    case 0x66:
        limetop_pllcfg_reg06_read();
        break;
    case 0x67:
        limetop_pllcfg_reg07_read();
        break;
    case 0x68:
        break;
    case 0x6a:
        limetop_pllcfg_n_cnt_read();
        break;
    case 0x6b:
        limetop_pllcfg_m_cnt_read();
        break;
    case 0x6e:
        limetop_pllcfg_c0_cnt_read();
        break;
    case 0x6f:
        limetop_pllcfg_c1_cnt_read();
        break;
    case 0x70:
        limetop_pllcfg_c2_cnt_read();
        break;
    case 0x71:
        limetop_pllcfg_c3_cnt_read();
        break;
    case 0x72:
        limetop_pllcfg_c4_cnt_read();
        break;
    case 0x7d:
        limetop_tst_top_tx_tst_i_write(value);
        break;
    case 0x7e:
        limetop_tst_top_tx_tst_q_write(value);
        break;

    case 0xc0:
        limetop_general_periph_board_gpio_OVRD_write(value);
        break;
    case 0xc4:
        limetop_general_periph_board_gpio_DIR_write(value);
        break;
    case 0xc6:
        limetop_general_periph_board_gpio_VAL_write(value);
        break;
    case 0xcc:
        limetop_general_periph_periph_output_OVRD_0_write(value);
        break;
    case 0xcd:
        limetop_general_periph_periph_output_VAL_0_write(value);
        break;
    case 0xce:
        limetop_general_periph_periph_output_OVRD_1_write(value);
        break;
    case 0xcf:
        limetop_general_periph_periph_output_VAL_1_write(value);
        break;

    default:
        printf("FWE: %04x\n", addr);
        break;
    }
}