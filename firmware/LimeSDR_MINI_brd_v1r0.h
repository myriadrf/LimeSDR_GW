/*
-- ----------------------------------------------------------------------------	
-- FILE        : LIMESDR_MINI_brd_v1r0.h
-- DESCRIPTION : LimeSDR Mini Board Header.
-- DATE        : 2015-2024
-- AUTHOR(s)   : Lime Microsystems
-- REVISION    : -
-- ----------------------------------------------------------------------------	
*/

#ifndef _LIMESDR_MINI_BRD_V1R0_H_
#define _LIMESDR_MINI_BRD_V1R0_H_

#include "LMS64C_protocol.h"

#pragma message ("**** limesdr_mini_brd_v1r0 ****")

#define MAX_ID_LMS7                1

//I2C devices
#define   LM75_I2C_ADDR		0x48 //0x90
#define EEPROM_I2C_ADDR		0x50 //0xA2

//get info
#ifdef LIMESDR_MINI_V2
#define DEV_TYPE			LMS_DEV_MINI_V2
#else
#define DEV_TYPE			LMS_DEV_MINI
#endif
#define HW_VER				0
#define EXP_BOARD			EXP_BOARD_UNSUPPORTED

//FPGA Cyclone IV GX (EP4GX30F23C7N) bitstream (RBF) size in bytes
#define FPGA_SIZE 			2751361 //1191788

//FLash memory (W25Q128JV , 128M-bit))
#define FLASH_PAGE_SIZE 	0x100 	//256 bytes, SPI Page size to be used for transfers
#define FLASH_SECTOR_SIZE 	0x1000 	//4KB
#define FLASH_BLOCK_SIZE	0x10000 //64KB

//FLASH memory layout
#define FLASH_LAYOUT_FPGA_METADATA	64//FPGA autoload metadata (start sector)
#define FLASH_LAYOUT_FPGA_BITSTREAM	0//FPGA bitstream (start sector) till end

#define FLASH_CMD_SECTOR_ERASE 0x20 //0x1B //Depends on flash, reversed: 0xD8 or 0x20
#define FLASH_CMD_PAGE_WRITE   0x02 //0x40 //Reversed 0x02
#define FLASH_CMD_PAGE_WRITEEN 0x06 //0x60 //Reversed 0x06
#define FLASH_CMD_PAGE_READ    0x03 //0xC0 //Reversed 0x03
#define FLASH_CMD_PAGE_READST  0x05 //0xA0 //Reversed 0x05
#define FLASH_CMD_READJEDECID  0x9F //0xF9 //Reversed 0x9F

typedef struct{
	uint32_t Bitream_size;
	uint8_t Autoload;
}tBoard_Config_FPGA; //tBoard_Config_FPGA or tBoard_cfg_FPGA

#endif
