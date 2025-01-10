#include <stdio.h>

#include <generated/csr.h>

#include "regremap.h"

// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs
void readCSR(uint8_t *address, uint8_t *regdata_array) {
    uint16_t value = 0;
    uint16_t addr = ((uint16_t) address[0] << 8) | address[1];

    switch (addr) {
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
            value = 0x2;
            break;
        case 0x5:
            value = lime_top_lms7002_CLK_CTRL_DRCT_TXCLK_EN_read() & 0x1;
            value = value | ((lime_top_lms7002_CLK_CTRL_DRCT_RXCLK_EN_read() & 0x1) << 1);
            break;
        case 0x7:
            value = lime_top_rx_path_ch_en_read();
            break;
        case 0x8:
            value = lime_top_rx_path_smpl_width_read() & 0x3;
        //TODO: check ddr_en usage
        //value = value | ((lime_top_lms7002_ddr_en_read() & 0x1) << 6);
            value = value | ((lime_top_lms7002_trxiq_pulse_read() & 0x1) << 7);
            value = value | ((lime_top_lms7002_mimo_int_en_read() & 0x1) << 8);
            value = value | ((lime_top_tx_path_sync_dis_read() & 0x1) << 9);
            break;
        case 0xA:
            value = lime_top_lms7002_tx_en_read();
            value |= rfsw_control_rfsw_rx_read() << 2;
            value |= rfsw_control_rfsw_tx_read() << 4;
            value |= rfsw_control_tdd_manual_val_read() << 5;
            value |= rfsw_control_tdd_auto_en_read() << 6;
            value |= rfsw_control_tdd_invert_read() << 7;
            value |= lime_top_lms7002_test_ptrn_en_read() << 9;
            value |= rfsw_control_rfsw_auto_en_read() << 11;
            break;
        case 0x18:
            value = 0;
            value |= value | ((fpgacfg_TCXO_EN_read() & 0x1) << 1);
            value |= value | ((fpgacfg_EXT_CLK_read() & 0x1) << 2);
            break;
        case 0x19:
            value = lime_top_rx_path_pkt_size_read();
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

    switch (addr) {
        case 0x3:
            fpgacfg_reserved_03_write(value);
            break;
        case 0x05:
            lime_top_lms7002_CLK_CTRL_DRCT_TXCLK_EN_write((value & 0x1) >> 0);
            lime_top_lms7002_CLK_CTRL_DRCT_RXCLK_EN_write((value & 0x2) >> 1);
            break;
        case 0x7:
            lime_top_tx_path_ch_en_write(value);
            lime_top_rx_path_ch_en_write(value);
            lime_top_lms7002_ch_en_write(value);
            break;
        case 0x8:
            lime_top_rx_path_smpl_width_write(value & 0x3);
            lime_top_tx_path_smpl_width_write(value & 0x3);
        //TODO: check ddr_en usage
        //lime_top_lms7002_ddr_en_write((value & 0x40) >> 6);
            lime_top_lms7002_trxiq_pulse_write((value & 0x80) >> 7);
            lime_top_lms7002_mimo_int_en_write((value & 0x100) >> 8);
            lime_top_tx_path_sync_dis_write((value & 0x200) >> 9);
            break;
        case 0xA:
            lime_top_lms7002_tx_en_write(value);
            lime_top_lms7002_rx_en_write(value);
            rfsw_control_rfsw_rx_write((value & 0xC) >> 2);
            rfsw_control_rfsw_tx_write((value & 0x10) >> 4);
            rfsw_control_tdd_manual_val_write((value & 0x20) >> 5);
            rfsw_control_tdd_auto_en_write((value & 0x40) >> 6);
            rfsw_control_tdd_invert_write((value & 0x80) >> 7);
            lime_top_lms7002_test_ptrn_en_write((value & 0x200) >> 9);
            rfsw_control_rfsw_auto_en_write((value & 0x800) >> 11);
            break;
        case 0x13:
            printf("13\n");
            break;
        case 0x18:
            fpgacfg_TCXO_EN_write((value & 0x2) >> 1);
            fpgacfg_EXT_CLK_write((value & 0x4) >> 2);
            break;
        case 0x19:
            lime_top_rx_path_pkt_size_write(value);
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
        default:
            break;
    }
}
