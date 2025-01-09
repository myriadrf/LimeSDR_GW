/*
 * fpga_flash_qspi.h
 *
 *  Created on: Sep 7, 2022
 *      Author: Lime Microsystems
 */

#ifndef SRC_FPGA_FLASH_QSPI_H_
#define SRC_FPGA_FLASH_QSPI_H_

//#include "xspi.h"		/* SPI device driver */
#include <generated/csr.h>
#include <stdint.h>

#define SPI_CS_HIGH (0 << 0)
#define SPI_CS_LOW  (1 << 0)
#define SPI_START   (1 << 0)
#define SPI_DONE    (1 << 0)
#define SPI_LENGTH  (1 << 8)
#define SPI_MAX_TRANSFER_SIZE 5


// Command set for MX25L25645G FLASH memory
#define ENSO 0xB1	// Enter secured OTP
#define EXSO 0xC1   // Exit secured OTP
#define PP   0x02   // Page Program (PP)
#define WRDI 0x04   // Write Disable (WRDI)
#define RDSR 0x05   // Read Status Register (RDSR)
#define WREN 0x06   // Write Enable (WREN)
#define RDCR 0x15   // Read Configuration Register (RDCR)


void Spi_Transfer(uint8_t * sendbuff, uint8_t * recvbuff, unsigned int ByteCount);
//void Init_flash_qspi(u16 DeviceId, XSpi *InstancePtr, u32 Options);
//int FlashQspi_CMD_DisQPI(XSpi *InstancePtr);
int FlashQspi_CMD(uint8_t cmd);
void FlashQspi_CMD_ReadRDSR(uint8_t* Data);
void FlashQspi_CMD_ReadRDCR(uint8_t* Data);
void FlashQspi_CMD_WREN(void);
void FlashQspi_CMD_WRDI(void);
//int FlashQspi_CMD_WRSR(XSpi *InstancePtr, u8 StatusReg, u8 ConfigReg);
//int FlashQspi_CMD_ReadRDCR(XSpi *InstancePtr, u8* Data);
//int FlashQspi_CMD_ReadDataPage(XSpi *InstancePtr, u32 address, u8* buffer);
void FlashQspi_CMD_ReadDataByte(uint32_t address, uint8_t* buffer);
int FlashQspi_CMD_ReadOTPData(uint32_t address, uint8_t bytes, uint8_t* buffer);
void FlashQspi_CMD_WriteDataPage(uint32_t address, uint8_t* buffer);
//int FlashQspi_CMD_PageProgram(XSpi *InstancePtr, u32 address, u8 bytes, u8* buffer);
void FlashQspi_CMD_SectorErase(uint32_t address);

void FlashQspi_ProgramPage(uint32_t address, uint8_t* data);
void FlashQspi_CMD_PageProgramByte(uint32_t address, uint8_t* byte);
int FlashQspi_ProgramOTP(uint32_t address, uint8_t bytes, uint8_t* data);
void FlashQspi_EraseSector(uint32_t address);
//int FlashQspi_ReadPage(XSpi *InstancePtr, u32 address, u8* data);

#endif /* SRC_FPGA_FLASH_QSPI_H_ */
