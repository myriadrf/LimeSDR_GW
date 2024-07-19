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
		value = fpgacfg_reserved_03_read();
		break;
	case 0x7:
		value = fpgacfg_channel_cntrl_read();
		break;
	case 0xA:
		value = lime_top_lms7002_tx_en_read();
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
		fpgacfg_channel_cntrl_write(value);
		break;
	case 0xA:
		lime_top_lms7002_tx_en_write(value);
		lime_top_lms7002_rx_en_write(value);
		break;
	default:
		break;
	}
}



