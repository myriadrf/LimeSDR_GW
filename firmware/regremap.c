#include <stdio.h>

#include <generated/csr.h>

#include "regremap.h"


// To read and re-map old LMS64C protocol style SPI registers to Litex CSRs
void readCSR(uint8_t *address, uint8_t *regdata_array) {
	uint32_t value=0;
	uint16_t addr = ((uint16_t)address[0] << 8) | address[1];

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
		break;
	case 0x19:
		value = lime_top_rx_path_pkt_size_read();
		break;
	case 0x21:
		value = 0x5;
		break;
	default:
		break;
	}

	regdata_array[0] = (uint8_t)(value & 0xFF);          // Byte 0 (LSB)
	regdata_array[1] = (uint8_t)((value >> 8) & 0xFF);   // Byte 1
	// Litex CSRs are 4byte words, LMS64C spi regs are 2byte - others unused.
	//regdata_array[2] = (uint8_t)((value >> 16) & 0xFF);  // Byte 2
	//regdata_array[3] = (uint8_t)((value >> 24) & 0xFF);  // Byte 3 (MSB)
}

// To write and re-map old LMS64C protocol style SPI registers to Litex CSRs
void writeCSR(uint8_t *address, uint8_t *wrdata_array) {
	uint32_t value = (0x0000FFFF &  ( ((uint32_t)wrdata_array[0] << 8) | ((uint32_t)wrdata_array[1]) ) );
	uint16_t addr = ((uint16_t)address[0] << 8) | address[1];

	switch (addr) {
	case 0x3:
		fpgacfg_reserved_03_write(value);
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
		break;
	case 0x13:
		printf("13\n");
		break;
	case 0x18:
		printf("18\n");
		break;
	case 0x19:
		lime_top_rx_path_pkt_size_write(value);
		break;
	default:
		break;
	}
}



