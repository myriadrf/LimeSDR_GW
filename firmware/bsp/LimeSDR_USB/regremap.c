#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "regremap.h"

uint16_t transform_fpga_signature(uint16_t write_val)
{
    /* Invert the lower 4 bits and position them at bits [7:4].
     * Bits [15:8] and [3:0] are cleared to zero to ensure exact matching. */
    return (uint16_t)((~write_val & 0x0Fu) << 4);
}

static uint16_t fpga_signature = 0;

// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-USB
bool readCSR(uint8_t *address, uint8_t *regdata_array)
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
    case 0x05:
        /* LMS64C Register 0x0005: Direct Clock Control
         * Bits:
         *   [0] DRCT_TXCLK_EN: TX CLK source (0: PLL, 1: Direct clock)
         *   [1] DRCT_RXCLK_EN: RX CLK source (0: PLL, 1: Direct clock)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_drct_clk_ctrl_read() & 0x3;
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
#ifdef DDR_MODULES_PRESENT
    case 0x0c:
        value = pss_wfm_ch_en_read();
        break;
#endif
    case 0x0d:
#ifdef DDR_MODULES_PRESENT
        value = pss_wfm_smpl_width_read()&0x01;
        value |= (pss_wfm_play_read()&0x01)<<1;
        value |= (pss_wfm_load_read()&0x01)<<2;
#else
        // Reserved when waveform playback is unavailable.
#endif
        break;
    case 0xF:
        value = limetop_fpgacfg_txant_pre_read();
        break;
    case 0x10:
        value = limetop_fpgacfg_txant_post_read();
        break;
    case 0x17:
        value = pss_lb_io_lb_out_override_val_read();
        break;
    case 0x18:
        value = limetop_fpgacfg_reg18_read();
        break;
    case 0x19:
        value = limetop_rxtx_top_rx_path_pkt_size_read();
        break;
    case 0x1A: {
        // LED1_CTRL[2:0]=[OVRD,RED,GREEN] (bits 0:2), LED2_CTRL[6:4]=[OVRD,RED,GREEN] (bits 4:6).
        // The legacy single OVRD bit maps onto two independent (green/red) override-enable bits
        // here, so it is read back as their AND.
        uint8_t ovrd = pss_led_fan_io_led_fan_out_override_read();
        uint8_t val  = pss_led_fan_io_led_fan_out_override_val_read();

        uint8_t led1_ovrd  = (ovrd & 0x1) & ((ovrd >> 1) & 0x1);
        uint8_t led1_green = val & 0x1;
        uint8_t led1_red   = (val >> 1) & 0x1;

        uint8_t led2_ovrd  = ((ovrd >> 2) & 0x1) & ((ovrd >> 3) & 0x1);
        uint8_t led2_green = (val >> 2) & 0x1;
        uint8_t led2_red   = (val >> 3) & 0x1;

        value = (led1_ovrd << 0) | (led1_red << 1) | (led1_green << 2) |
                (led2_ovrd << 4) | (led2_red << 5) | (led2_green << 6);
        break;
    }
    case 0x1C: {
        // FX3_CTRL[2:0]=[OVRD,RED,GREEN].
        uint8_t ovrd = pss_led_fan_io_led_fan_out_override_read();
        uint8_t val  = pss_led_fan_io_led_fan_out_override_val_read();

        uint8_t fx3_ovrd  = ((ovrd >> 4) & 0x1) & ((ovrd >> 5) & 0x1);
        uint8_t fx3_green = (val >> 4) & 0x1;
        uint8_t fx3_red   = (val >> 5) & 0x1;

        value = (fx3_ovrd << 0) | (fx3_red << 1) | (fx3_green << 2);
        break;
    }
    case 0x20:
        /* LMS64C Register 0x0020: C1 Phase Offset
         * Bits [8:0]: Clock output 1 phase offset in degrees (0-360)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c1_phase_read() & 0x1FF;
        break;
    case 0x21:
        /* LMS64C Register 0x0021: PLL & Phase Configuration Status
         * Bits:
         *   [0] PLLCFG_DONE:  PLL configuration done (0: Not done, 1: Done)
         *   [1] PLLCFG_BUSY:  PLL configuration busy (0: Idle, 1: Busy)
         *   [2] PHCFG_DONE:   Phase configuration done (0: Not done, 1: Done)
         *   [3] PHCFG_ERR:    Phase configuration error (0: No error, 1: Error)
         *   [7] PLLCFG_ERROR: PLL configuration error (0: No error, 1: Error)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_pll_status_read();
        break;
    case 0x22:
        /* LMS64C Register 0x0022: PLL Lock Status
         * Bits [15:0]: Array of PLL locked flags (bit 0: TX PLL lock, bit 1: RX PLL lock)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_pll_lock_read();
        break;
    case 0x23:
        /* LMS64C Register 0x0023: PLL & Phase Configuration Control
         * Bits:
         *   [0]    PLLCFG_START: Start PLL config (0 to 1 transition)
         *   [1]    PHCFG_START:  Start phase config (0 to 1 transition)
         *   [2]    PLLRST_START: Start PLL reset (0 to 1 transition)
         *   [7:3]  PLL_IND:      PLL index for reconfiguration
         *   [12:8] CNT_IND:      Counter index for reconfiguration (0: All, 1: M, 2: C0, 3: C1, etc.)
         *   [13]   PHCFG_UPDN:   Phase shift direction (0: Down, 1: Up)
         *   [14]   PHCFG_MODE:   Phase config mode (0: Manual, 1: Auto)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_pll_ctrl_read();
        break;
    case 0x24:
        /* LMS64C Register 0x0024: Counter Phase Value
         * Bits [15:0]: Phase step counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_cnt_phase_read();
        break;
    case 0x25:
        /* LMS64C Register 0x0025: PLL VCO Divider Control
         * Bits:
         *   [7] PLLCFG_VCODIV: PLL VCO divider (0: disabled, 1: enabled)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_pll_vcodiv_read() & (1 << 7);
        break;
    case 0x26:
        /* LMS64C Register 0x0026: M and N Counter Divider Control
         * Bits:
         *   [0] N_DIV_BYP: N counter divider bypass (0: Normal, 1: Bypass)
         *   [1] N_ODD_DIV: N counter odd divider (0: Even, 1: Odd)
         *   [2] M_DIV_BYP: M counter divider bypass (0: Normal, 1: Bypass)
         *   [3] M_ODD_DIV: M counter odd divider (0: Even, 1: Odd)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_mn_div_ctrl_read() & 0xF;
        break;
    case 0x27:
        /* LMS64C Register 0x0027: C0-C4 Output Divider Control
         * Bits:
         *   [0] C0_DIV_BYP: Clock output 0 divider bypass (0: Normal, 1: Bypass)
         *   [1] C0_ODDDIV:  Clock output 0 odd divider (0: Even, 1: Odd)
         *   [2] C1_DIV_BYP: Clock output 1 divider bypass (0: Normal, 1: Bypass)
         *   [3] C1_ODDDIV:  Clock output 1 odd divider (0: Even, 1: Odd)
         *   [4] C2_DIV_BYP: Clock output 2 divider bypass (0: Normal, 1: Bypass)
         *   [5] C2_ODDDIV:  Clock output 2 odd divider (0: Even, 1: Odd)
         *   [6] C3_DIV_BYP: Clock output 3 divider bypass (0: Normal, 1: Bypass)
         *   [7] C3_ODDDIV:  Clock output 3 odd divider (0: Even, 1: Odd)
         *   [8] C4_DIV_BYP: Clock output 4 divider bypass (0: Normal, 1: Bypass)
         *   [9] C4_ODDDIV:  Clock output 4 odd divider (0: Even, 1: Odd)
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c_div_ctrl_read() & 0x3FF;
        break;
    case 0x2A:
        /* LMS64C Register 0x002A: N Counter Value
         * Bits [15:0]: PLL N counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_n_cnt_read();
        break;
    case 0x2B:
        /* LMS64C Register 0x002B: M Counter Value
         * Bits [15:0]: PLL M counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_m_cnt_read();
        break;
    case 0x2E:
        /* LMS64C Register 0x002E: C0 Divider Counter Value
         * Bits [15:0]: Clock output 0 divider counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c0_div_cnt_read();
        break;
    case 0x2F:
        /* LMS64C Register 0x002F: C1 Divider Counter Value
         * Bits [15:0]: Clock output 1 divider counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c1_div_cnt_read();
        break;
    case 0x30:
        /* LMS64C Register 0x0030: C2 Divider Counter Value
         * Bits [15:0]: Clock output 2 divider counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c2_div_cnt_read();
        break;
    case 0x31:
        /* LMS64C Register 0x0031: C3 Divider Counter Value
         * Bits [15:0]: Clock output 3 divider counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c3_div_cnt_read();
        break;
    case 0x32:
        /* LMS64C Register 0x0032: C4 Divider Counter Value
         * Bits [15:0]: Clock output 4 divider counter value
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_c4_div_cnt_read();
        break;
    case 0x3E:
        /* LMS64C Register 0x003E: Auto Phase Configuration Samples
         * Bits [15:0]: Number of samples during auto phase search
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_auto_phcfg_smpls_read();
        break;
    case 0x3F:
        /* LMS64C Register 0x003F: Auto Phase Configuration Step Size
         * Bits [15:0]: Phase step size during auto phase search
         */
        value = limetop_lms7002_top_lms7002_clk_CLK_CTRL_auto_phcfg_step_read();
        break;
    case 0x60:
        value = fpga_signature;
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
    case 0x6D:
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
    case 0x77:
        value = pss_tst_top_ddr2_1_pnf_per_bit_read()&0xFFFF;
        break;
    case 0x78:
        value = (pss_tst_top_ddr2_1_pnf_per_bit_read()>>16)&0xFFFF;
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
        value = pss_tst_top_ddr2_2_pnf_per_bit_read()&0xFFFF;
        break;
    case 0x7C:
        value = (pss_tst_top_ddr2_2_pnf_per_bit_read()>>16)&0xFFFF;
        break;
    case 0xC0:
        // GPIO_OVRD (1 = Enable Manual Control for FPGA_GPIO[i]). Same polarity as gpio_io's
        // override CSR, no translation needed.
        value = pss_gpio_io_gpio_override_read();
        break;
    case 0xC2:
        // GPIO_RD: read current state of FPGA_GPIO[i] pins.
        value = pss_gpio_io_gpio_val_read();
        break;
    case 0xC4:
        // GPIO_DIR 1 = Output
        value = pss_gpio_io_gpio_override_dir_read() & 0xFF;
        break;
    case 0xC6:
        // GPIO_VAL: manual output value for FPGA_GPIO[i].
        value = pss_gpio_io_gpio_override_val_read();
        break;
    case 0xCA:
        // value = periphcfg_PERIPH_INPUT_SEL_0_read();
        break;
    case 0xCC:
        // FAN_OVRD (1 = Enable Manual Fan Control), Fan is bit 6 of led_fan_io's out_pads bus.
        value = (pss_led_fan_io_led_fan_out_override_read() >> 6) & 0x1;
        break;
    case 0xCD:
        // FAN_VAL (Manual Fan Control Value, 1 = ON).
        value = (pss_led_fan_io_led_fan_out_override_val_read() >> 6) & 0x1;
        break;
    case 0xD2:
        // value = periphcfg_PERIPH_EN_read();
        break;
    case 0xD3:
        // value = periphcfg_PERIPH_SEL_read();
        break;
    case 0x0E:
    case 0x28:
    case 0xD1:
    case 0x280:
    case 0x7FE1:
    case 0x7FE2:
    case 0x7FE3:
    case 0x7FE4:
    case 0x7FE5:
        // Reserved: return zero for compatibility with SSDR/XTRX.
        break;

    default:
        return false;
    }

    regdata_array[0] = (uint8_t)(value & 0xFF);        // Byte 0 (LSB)
    regdata_array[1] = (uint8_t)((value >> 8) & 0xFF); // Byte 1
    return true;
}

// To write and re-map old LMS64C protocol style SPI registers to Litex CSRs for LimeSDR-USB
bool writeCSR(uint8_t *address, uint8_t *wrdata_array)
{
    uint16_t value = ((uint16_t)wrdata_array[0] << 8) | wrdata_array[1];
    uint16_t addr  = ((uint16_t)address[0] << 8) | address[1];
    uint32_t reg;

    switch (addr) {
    case 0x05:
        /* LMS64C Register 0x0005: Direct Clock Control
         * Bits:
         *   [0] DRCT_TXCLK_EN: TX CLK source (0: PLL, 1: Direct clock)
         *   [1] DRCT_RXCLK_EN: RX CLK source (0: PLL, 1: Direct clock)
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_drct_clk_ctrl_write(value & 0x3);
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
#ifdef DDR_MODULES_PRESENT
    case 0x0C:
        pss_wfm_ch_en_write(value);
        break;
#endif
    case 0x0D:
#ifdef DDR_MODULES_PRESENT
        pss_wfm_smpl_width_write(value);
        pss_wfm_play_write((value >> 1)&0x1);
        limetop_lms7002_top_txiq_mux_sel_write((value >> 1)&0x1);
        pss_wfm_load_write((value >> 2)&0x1);
#else
        // Reserved when waveform playback is unavailable.
#endif
        break;
    case 0xF:
        limetop_fpgacfg_txant_pre_write(value);
        break;
    case 0x10:
        limetop_fpgacfg_txant_post_write(value);
        break;
    case 0x17:
        pss_lb_io_lb_out_override_val_write(value);
        pss_lb_io_lb_out_override_write(0xFF);
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
    case 0x1A: {
        // LED1_CTRL[2:0]=[OVRD,RED,GREEN] (bits 0:2), LED2_CTRL[6:4]=[OVRD,RED,GREEN] (bits 4:6).
        // The legacy single OVRD bit is fanned out to both the green-pin and red-pin
        // override-enable bits.
        uint8_t ovrd = pss_led_fan_io_led_fan_out_override_read();
        uint8_t val  = pss_led_fan_io_led_fan_out_override_val_read();

        uint8_t led1_ovrd  = (value >> 0) & 0x1;
        uint8_t led1_red   = (value >> 1) & 0x1;
        uint8_t led1_green = (value >> 2) & 0x1;
        uint8_t led2_ovrd  = (value >> 4) & 0x1;
        uint8_t led2_red   = (value >> 5) & 0x1;
        uint8_t led2_green = (value >> 6) & 0x1;

        ovrd &= ~0x0F;
        ovrd |= (led1_ovrd << 0) | (led1_ovrd << 1) | (led2_ovrd << 2) | (led2_ovrd << 3);

        val &= ~0x0F;
        val |= (led1_green << 0) | (led1_red << 1) | (led2_green << 2) | (led2_red << 3);

        pss_led_fan_io_led_fan_out_override_write(ovrd);
        pss_led_fan_io_led_fan_out_override_val_write(val);
        break;
    }
    case 0x1C: {
        // FX3_CTRL[2:0]=[OVRD,RED,GREEN].
        uint8_t ovrd = pss_led_fan_io_led_fan_out_override_read();
        uint8_t val  = pss_led_fan_io_led_fan_out_override_val_read();

        uint8_t fx3_ovrd  = (value >> 0) & 0x1;
        uint8_t fx3_red   = (value >> 1) & 0x1;
        uint8_t fx3_green = (value >> 2) & 0x1;

        ovrd &= ~(0x3 << 4);
        ovrd |= (fx3_ovrd << 4) | (fx3_ovrd << 5);

        val &= ~(0x3 << 4);
        val |= (fx3_green << 4) | (fx3_red << 5);

        pss_led_fan_io_led_fan_out_override_write(ovrd);
        pss_led_fan_io_led_fan_out_override_val_write(val);
        break;
    }
    case 0x20:
        /* LMS64C Register 0x0020: C1 Phase Offset
         * Bits [8:0]: Clock output 1 phase offset in degrees (0-360)
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c1_phase_write(value & 0x1FF);
        break;
    case 0x23:
        /* LMS64C Register 0x0023: PLL & Phase Configuration Control
         * Bits:
         *   [0]    PLLCFG_START: Start PLL config (0 to 1 transition)
         *   [1]    PHCFG_START:  Start phase config (0 to 1 transition)
         *   [2]    PLLRST_START: Start PLL reset (0 to 1 transition)
         *   [7:3]  PLL_IND:      PLL index for reconfiguration
         *   [12:8] CNT_IND:      Counter index for reconfiguration (0: All, 1: M, 2: C0, 3: C1, etc.)
         *   [13]   PHCFG_UPDN:   Phase shift direction (0: Down, 1: Up)
         *   [14]   PHCFG_MODE:   Phase config mode (0: Manual, 1: Auto)
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_pll_ctrl_write(value & 0x7FFF);
        break;
    case 0x24:
        /* LMS64C Register 0x0024: Counter Phase Value
         * Bits [15:0]: Phase step counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_cnt_phase_write(value);
        break;
    case 0x25:
        /* LMS64C Register 0x0025: PLL VCO Divider Control
         * Bits:
         *   [7] PLLCFG_VCODIV: PLL VCO divider (0: disabled, 1: enabled)
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_pll_vcodiv_write(value & (1 << 7));
        break;
    case 0x26:
        /* LMS64C Register 0x0026: M and N Counter Divider Control
         * Bits:
         *   [0] N_DIV_BYP: N counter divider bypass (0: Normal, 1: Bypass)
         *   [1] N_ODD_DIV: N counter odd divider (0: Even, 1: Odd)
         *   [2] M_DIV_BYP: M counter divider bypass (0: Normal, 1: Bypass)
         *   [3] M_ODD_DIV: M counter odd divider (0: Even, 1: Odd)
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_mn_div_ctrl_write(value & 0xF);
        break;
    case 0x27:
        /* LMS64C Register 0x0027: C0-C4 Output Divider Control
         * Bits:
         *   [0] C0_DIV_BYP: Clock output 0 divider bypass (0: Normal, 1: Bypass)
         *   [1] C0_ODDDIV:  Clock output 0 odd divider (0: Even, 1: Odd)
         *   [2] C1_DIV_BYP: Clock output 1 divider bypass (0: Normal, 1: Bypass)
         *   [3] C1_ODDDIV:  Clock output 1 odd divider (0: Even, 1: Odd)
         *   [4] C2_DIV_BYP: Clock output 2 divider bypass (0: Normal, 1: Bypass)
         *   [5] C2_ODDDIV:  Clock output 2 odd divider (0: Even, 1: Odd)
         *   [6] C3_DIV_BYP: Clock output 3 divider bypass (0: Normal, 1: Bypass)
         *   [7] C3_ODDDIV:  Clock output 3 odd divider (0: Even, 1: Odd)
         *   [8] C4_DIV_BYP: Clock output 4 divider bypass (0: Normal, 1: Bypass)
         *   [9] C4_ODDDIV:  Clock output 4 odd divider (0: Even, 1: Odd)
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c_div_ctrl_write(value & 0x3FF);
        break;
    case 0x2A:
        /* LMS64C Register 0x002A: N Counter Value
         * Bits [15:0]: PLL N counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_n_cnt_write(value);
        break;
    case 0x2B:
        /* LMS64C Register 0x002B: M Counter Value
         * Bits [15:0]: PLL M counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_m_cnt_write(value);
        break;
    case 0x2E:
        /* LMS64C Register 0x002E: C0 Divider Counter Value
         * Bits [15:0]: Clock output 0 divider counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c0_div_cnt_write(value);
        break;
    case 0x2F:
        /* LMS64C Register 0x002F: C1 Divider Counter Value
         * Bits [15:0]: Clock output 1 divider counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c1_div_cnt_write(value);
        break;
    case 0x30:
        /* LMS64C Register 0x0030: C2 Divider Counter Value
         * Bits [15:0]: Clock output 2 divider counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c2_div_cnt_write(value);
        break;
    case 0x31:
        /* LMS64C Register 0x0031: C3 Divider Counter Value
         * Bits [15:0]: Clock output 3 divider counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c3_div_cnt_write(value);
        break;
    case 0x32:
        /* LMS64C Register 0x0032: C4 Divider Counter Value
         * Bits [15:0]: Clock output 4 divider counter value
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_c4_div_cnt_write(value);
        break;
    case 0x3E:
        /* LMS64C Register 0x003E: Auto Phase Configuration Samples
         * Bits [15:0]: Number of samples during auto phase search
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_auto_phcfg_smpls_write(value);
        break;
    case 0x3F:
        /* LMS64C Register 0x003F: Auto Phase Configuration Step Size
         * Bits [15:0]: Phase step size during auto phase search
         */
        limetop_lms7002_top_lms7002_clk_CLK_CTRL_auto_phcfg_step_write(value);
        break;
    case 0x60:
        fpga_signature = transform_fpga_signature(value);
        break;
    case 0x61:
        pss_tst_top_test_en_write(value);
        break;
    case 0x63:
        pss_tst_top_test_frc_err_write(value);
        break;
    case 0xC0:
        // GPIO_OVRD (1 = Enable Manual Control for FPGA_GPIO[i]). Same polarity as gpio_io's
        // override CSR, no translation needed.
        pss_gpio_io_gpio_override_write(value & 0xFF);
        break;
    case 0xC4:
        // GPIO_DIR 1 = Output
        pss_gpio_io_gpio_override_dir_write(value & 0xFF);
        break;
    case 0xC6:
        // GPIO_VAL: manual output value for FPGA_GPIO[i].
        pss_gpio_io_gpio_override_val_write(value & 0xFF);
        break;
    case 0xCA:
        // periphcfg_PERIPH_INPUT_SEL_0_write(value);
        break;
    case 0xCC: {
        // FAN_OVRD (1 = Enable Manual Fan Control), Fan is bit 6 of led_fan_io's out_pads bus.
        uint8_t ovrd = pss_led_fan_io_led_fan_out_override_read();
        ovrd &= ~(1 << 6);
        ovrd |= (value & 0x1) << 6;
        pss_led_fan_io_led_fan_out_override_write(ovrd);
        break;
    }
    case 0xCD: {
        // FAN_VAL (Manual Fan Control Value, 1 = ON).
        uint8_t val = pss_led_fan_io_led_fan_out_override_val_read();
        val &= ~(1 << 6);
        val |= (value & 0x1) << 6;
        pss_led_fan_io_led_fan_out_override_val_write(val);
        break;
    }
    case 0xD2:
        // periphcfg_PERIPH_EN_write(value);
        break;
    case 0xD3:
        // periphcfg_PERIPH_SEL_write(value);
        break;
    case 0x0E:
    case 0x28:
    case 0xD1:
    case 0xFF:
    case 0x280:
    case 0x7FE1:
    case 0x7FE2:
    case 0x7FE3:
    case 0x7FE4:
    case 0x7FE5:
    case 0x7FFF:
        // Reserved: accept writes without changing hardware.
        break;

    // TODO: Implement register remapping
    default:
        return false;
    }
    return true;
}
