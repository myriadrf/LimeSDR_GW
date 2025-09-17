/**
-- ----------------------------------------------------------------------------
-- FILE:	SSDR.h
-- DESCRIPTION:	SSDR v1.0
-- DATE:	2025.09.17
-- AUTHOR(s):	Lime Microsystems
-- REVISION: v0r0
-- ----------------------------------------------------------------------------

*/
#ifndef SRC_SSDR_H_
#define SRC_SSDR_H_

#include "LMS64C_protocol.h"

//I2C devices

#define   LM75_I2C_ADDR		0x48
#define   I2C_ADDR_EEPROM   0x50

//GET INFO
#define DEV_TYPE			LMS_DEV_XTRX
#define HW_VER				2
#define EXP_BOARD			EXP_BOARD_UNSUPPORTED

#define MAX_ID_LMS7		1

#define OTP_UNLOCK_KEY		0x5A
#define OTP_SERIAL_ADDRESS  0x0000010
#define OTP_SERIAL_LENGTH   0x10

#define DAC_DEFF_VAL			46870			// Default TCXO DAC value loaded when EEPROM is empty

#endif /* SRC_SSDR_H_ */
