#include <stdio.h>

#include <generated/csr.h>

#include "regremap.h"

// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs
void readCSR(uint8_t *address, uint8_t *regdata_array) {
    uint16_t value = 0;
    uint32_t tmp;
    uint16_t addr = ((uint16_t) address[0] << 8) | address[1];

    switch (addr) {
        case 0x0:
            value = lime_top_fpgacfg_board_id_read();
            break;
        case 0x1:
            value = lime_top_fpgacfg_major_rev_read();
            break;
        case 0x2:
            value = lime_top_fpgacfg_compile_rev_read();
            break;
        case 0x3:
            value = 0x2;
            break;
        case 0x5:
            value = lime_top_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_TXCLK_EN_read() & 0x1;
            value = value | ((lime_top_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_RXCLK_EN_read() & 0x1) << 1);
            break;
        case 0x7:
            value = lime_top_fpgacfg_ch_en_read();
            break;
        case 0x8:
            value = lime_top_fpgacfg_reg08_read() & (0x3 | (1 << 7) | (1 << 8) | (1 << 9)); 
            break;
        case 0xA:
            tmp = lime_top_fpgacfg_reg10_read();
            value = tmp & 0x03;//(tmp >> 1) & 0x01;
            value |= lime_top_rfsw_control_rfsw_rx_read() << 2;
            value |= lime_top_rfsw_control_rfsw_tx_read() << 4;
            value |= lime_top_rfsw_control_tdd_manual_val_read() << 5;
            value |= lime_top_rfsw_control_tdd_auto_en_read() << 6;
            value |= lime_top_rfsw_control_tdd_invert_read() << 7;
            value |= (tmp & 0x200);
            value |= lime_top_rfsw_control_rfsw_auto_en_read() << 11;
            break;
        case 0x18:
            value = lime_top_fpgacfg_reg18_read();
            break;
        case 0x19:
            value = lime_top_rxtx_top_rx_path_pkt_size_read();
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
        case 0x23:
            value = csr_read_simple(clk_ctrl_addrs.pllcfg_start);
            value |= csr_read_simple(clk_ctrl_addrs.phcfg_start) << 1;
            value |= csr_read_simple(clk_ctrl_addrs.pllrst_start) << 2;
            value |= csr_read_simple(clk_ctrl_addrs.pll_ind) << 3;
            value |= csr_read_simple(clk_ctrl_addrs.phcfg_mode) << 14;
            break;
        case 0x26:
            value = csr_read_simple(clk_ctrl_addrs.vco_div_byp);
            value |= csr_read_simple(clk_ctrl_addrs.vco_mult_byp) << 2;
            break;
        case 0x27:
            value = csr_read_simple(clk_ctrl_addrs.c0_div_byp);
            value |= csr_read_simple(clk_ctrl_addrs.c1_div_byp) << 2;
            break;
        case 0x2A:
            value = csr_read_simple(clk_ctrl_addrs.vco_div_cnt);
            break;
        case 0x2B:
            value = csr_read_simple(clk_ctrl_addrs.vco_mult_cnt);
            break;
        case 0x2E:
            value = csr_read_simple(clk_ctrl_addrs.c0_div_cnt);
            break;
        case 0x2F:
            value = csr_read_simple(clk_ctrl_addrs.c1_div_cnt);
            break;
        case 0x3E:
            value = csr_read_simple(smpl_cmp_addrs.cmp_length);
            break;
        case 0xC0:
            value = periphcfg_BOARD_GPIO_OVRD_read();
            break;
        case 0xC4:
            value = periphcfg_BOARD_GPIO_DIR_read();
            break;
        case 0xC6:
            value = periphcfg_BOARD_GPIO_VAL_read();
            break;
        case 0xCA:
            value = periphcfg_PERIPH_INPUT_SEL_0_read();
            break;
        case 0xD2:
            value = periphcfg_PERIPH_EN_read();
            break;
        case 0xD3:
            value = periphcfg_PERIPH_SEL_read();
            break;
        case 0x61:
            value = sys_clock_test_test_en_read();
            value |= lms_clock_test_test_en_read() << 2;
            break;
        case 0x65:
            value = sys_clock_test_test_complete_read();
            value |= lms_clock_test_test_complete_read() << 2;
            break;
        case 0x69:
            value = sys_clock_test_test_cnt_read();
            break;
        case 0x72:
            value = lms_clock_test_test_cnt_read() & 0xFFFF;
            break;
        case 0x73:
            value = lms_clock_test_test_cnt_read() >> 16;
            break;
#ifdef TIMESOURCE_PRESENT
        // timesource registers
        case 0x280:
            value = lime_top_rxtx_top_rx_path_timestamp_settings_read() & 0xFFFF;
            break;
        case 0x281:
            value = lime_top_rx_delay_mode_read() & 0xFFFF;
            break;
        case 0x282:
            value = lime_top_tx_delay_mode_read() & 0xFFFF;
            break;
        // current time
        case 0x283:
            value = main_time_min_sec_read() & 0xFFFF;
            break;
        case 0x284:
            value = main_time_mon_day_hrs_read() & 0xFFFF;
            break;
        case 0x285:
            value = main_time_yrs_read() & 0xFFFF;
            break;
        // rx start time
        case 0x286:
            value = lime_top_rx_time_min_sec_read() & 0xFFFF;
            break;
        case 0x287:
            value = lime_top_rx_time_mon_day_hrs_read() & 0xFFFF;
            break;
        case 0x288:
            value = lime_top_rx_time_yrs_read() & 0xFFFF;
            break;
        // tx start time
        case 0x289:
            value = lime_top_tx_time_min_sec_read() & 0xFFFF;
            break;
        case 0x28A:
            value = lime_top_tx_time_mon_day_hrs_read() & 0xFFFF;
            break;
        case 0x28B:
            value = lime_top_tx_time_yrs_read() & 0xFFFF;
            break;

#endif

        default:
            break;
    }

    regdata_array[0] = (uint8_t) (value & 0xFF); // Byte 0 (LSB)
    regdata_array[1] = (uint8_t) ((value >> 8) & 0xFF); // Byte 1
    // Litex CSRs are 4byte words, LMS64C spi regs are 2byte - others unused.
    //regdata_array[2] = (uint8_t)((value >> 16) & 0xFF);  // Byte 2
    //regdata_array[3] = (uint8_t)((value >> 24) & 0xFF);  // Byte 3 (MSB)`
}

// To write and re-map old LMS64C protocol style SPI registers to Litex CSRs
void writeCSR(uint8_t *address, uint8_t *wrdata_array) {
    uint16_t value = (0x0000FFFF & (((uint32_t) wrdata_array[0] << 8) | ((uint32_t) wrdata_array[1])));
    uint8_t *value_byte_ptr = (uint8_t *) &value;
    uint16_t addr = ((uint16_t) address[0] << 8) | address[1];
    uint32_t reg;

    switch (addr) {
        case 0x3:
            lime_top_fpgacfg_reserved_03_write(value);
            break;
        case 0x05:
            lime_top_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_TXCLK_EN_write((value & 0x1) >> 0);
            lime_top_lms7002_top_lms7002_clk_CLK_CTRL_DRCT_RXCLK_EN_write((value & 0x2) >> 1);
            break;
        case 0x7:
            lime_top_fpgacfg_ch_en_write(value);
            break;
        case 0x8:
            reg = lime_top_fpgacfg_reg08_read();
            reg &= ~((1 << CSR_LIME_TOP_FPGACFG_REG08_SYNCH_DIS_OFFSET) |
                (1 << CSR_LIME_TOP_FPGACFG_REG08_MIMO_INT_EN_OFFSET) |
                (1 << CSR_LIME_TOP_FPGACFG_REG08_TRXIQ_PULSE_OFFSET) | (0x3));
            reg |= (value & 0x03); // smpl_width
            reg |= (value & 0x80); // trxiq_pulse
            reg |= (value & 0x100); // mimo_int_en
            reg |= (value & 0x200); // sync_dis
            lime_top_fpgacfg_reg08_write(reg);
            break;
        case 0xA:
            reg =  (value & 0x003) << 0; // rx_en + tx_en
            // reg |= (value & 0x002) << 1; // tx_en
            reg |= (value & 0x200) << 0; // test_ptrn_en
            lime_top_fpgacfg_reg10_write(reg);
            //lime_top_lms7002_tx_en_write(value);
            //lime_top_lms7002_rx_en_write(value);
            lime_top_rfsw_control_rfsw_rx_write((value & 0xC) >> 2);
            lime_top_rfsw_control_rfsw_tx_write((value & 0x10) >> 4);
            lime_top_rfsw_control_tdd_manual_val_write((value & 0x20) >> 5);
            lime_top_rfsw_control_tdd_auto_en_write((value & 0x40) >> 6);
            lime_top_rfsw_control_tdd_invert_write((value & 0x80) >> 7);
            //lime_top_lms7002_test_ptrn_en_write((value & 0x200) >> 9);
            lime_top_rfsw_control_rfsw_auto_en_write((value & 0x800) >> 11);
            break;
        case 0x13:
            printf("13\n");
            break;
        case 0x18:
            lime_top_fpgacfg_reg18_write(value);
            break;
        case 0x19:
            lime_top_rxtx_top_rx_path_pkt_size_write(value);
            break;
        case 0x20:
            csr_write_simple(value & 0x1FF, clk_ctrl_addrs.c1_phase);
            break;
        case 0x23:
            csr_write_simple((value & 1), clk_ctrl_addrs.pllcfg_start);
            csr_write_simple((value & 2) >> 1, clk_ctrl_addrs.phcfg_start);
            csr_write_simple((value & 4) >> 2, clk_ctrl_addrs.pllrst_start);
            csr_write_simple((value & 8) >> 3, clk_ctrl_addrs.pll_ind);
            csr_write_simple((value & 0x4000) >> 14, clk_ctrl_addrs.phcfg_mode);
            break;
        case 0x26:
            csr_write_simple((value & 1), clk_ctrl_addrs.vco_div_byp);
            csr_write_simple((value & 4) >> 2, clk_ctrl_addrs.vco_mult_byp);
            break;
        case 0x27:
            csr_write_simple((value & 1), clk_ctrl_addrs.c0_div_byp);
            csr_write_simple((value & 4) >> 2, clk_ctrl_addrs.c1_div_byp);
            break;
        case 0x2A:
            // Check if either of the bytes are over 32. This is to prevent divider values larger than 64
            value_byte_ptr[0] = (value_byte_ptr[0] > 32) ? 32 : value_byte_ptr[0];
            value_byte_ptr[1] = (value_byte_ptr[1] > 32) ? 32 : value_byte_ptr[1];
            csr_write_simple(value, clk_ctrl_addrs.vco_div_cnt);
            break;
        case 0x2B:
            // Check if either of the bytes are over 32. This is to prevent divider values larger than 64
            value_byte_ptr[0] = (value_byte_ptr[0] > 32) ? 32 : value_byte_ptr[0];
            value_byte_ptr[1] = (value_byte_ptr[1] > 32) ? 32 : value_byte_ptr[1];
            csr_write_simple(value, clk_ctrl_addrs.vco_mult_cnt);
            break;
        case 0x2E:
            // Check if either of the bytes are over 32. This is to prevent divider values larger than 64
            value_byte_ptr[0] = (value_byte_ptr[0] > 32) ? 32 : value_byte_ptr[0];
            value_byte_ptr[1] = (value_byte_ptr[1] > 32) ? 32 : value_byte_ptr[1];
            csr_write_simple(value, clk_ctrl_addrs.c0_div_cnt);
            break;
        case 0x2F:
            // Check if either of the bytes are over 32. This is to prevent divider values larger than 64
            value_byte_ptr[0] = (value_byte_ptr[0] > 32) ? 32 : value_byte_ptr[0];
            value_byte_ptr[1] = (value_byte_ptr[1] > 32) ? 32 : value_byte_ptr[1];
            csr_write_simple(value, clk_ctrl_addrs.c1_div_cnt);
            break;
        case 0x3E:
            csr_write_simple(value, smpl_cmp_addrs.cmp_length);
            break;
        case 0xC0:
        	periphcfg_BOARD_GPIO_OVRD_write(value);
            break;
        case 0xC4:
        	periphcfg_BOARD_GPIO_DIR_write(value);
            break;
        case 0xC6:
        	periphcfg_BOARD_GPIO_VAL_write(value);
            break;
        case 0xCA:
            periphcfg_PERIPH_INPUT_SEL_0_write(value);
            break;
        case 0xD2:
            periphcfg_PERIPH_EN_write(value);
            break;
        case 0xD3:
            periphcfg_PERIPH_SEL_write(value);
            break;
        case 0x61:
            sys_clock_test_test_en_write(value & 0x1);
            lms_clock_test_test_en_write((value & 0x4) >> 2);
            break;
#ifdef TIMESOURCE_PRESENT
        // timesource registers
        case 0x280:
            lime_top_rxtx_top_rx_path_timestamp_settings_write(value);
            break;
        case 0x281:
            lime_top_rx_delay_mode_write(value);
            break;
        case 0x282:
            lime_top_tx_delay_mode_write(value);
            break;

#endif
        default:
            break;
    }
}
