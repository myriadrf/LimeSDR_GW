/*
 * fpga_flash_qspi.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Lime Microsystems
 */
#include <stdio.h>
#include "fpga_flash_qspi.h"

#define DEBUG
/*
void Init_flash_qspi(u16 DeviceId, XSpi *InstancePtr, u32 Options)
{
	int spi_status;
	XSpi_Config *ConfigPtr;

    //Get default config
	ConfigPtr = XSpi_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		//return XST_DEVICE_NOT_FOUND;
	}

    //Temporary mode override for initialization, saving original value
//    spi_mode = ConfigPtr->SpiMode;
//    ConfigPtr->SpiMode = 0;

    //Perform IP core config
    spi_status = XSpi_CfgInitialize(InstancePtr, ConfigPtr,
				  ConfigPtr->BaseAddress);

    //Set Options
    spi_status = XSpi_SetOptions(InstancePtr, Options);
	if(spi_status != XST_SUCCESS) {
		//return XST_FAILURE;
	}

    // Start the SPI driver so that interrupts and the device are enabled
	spi_status = XSpi_Start(InstancePtr);

    //disable global interrupts since we will use a polled approach
	XSpi_IntrGlobalDisable(InstancePtr);
    XSpi_SetSlaveSelect(InstancePtr,1);
    FlashQspi_CMD_DisQPI(InstancePtr);
//	 retval = FlashQspi_CMD_WREN(InstancePtr);

//     retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);
//     retval = FlashQspi_CMD_ReadRDCR(InstancePtr,&config_reg);

    // retval = FlashQspi_WRSR(InstancePtr,status_reg,config_reg);

    // retval = FlashQspi_ReadRDSR(InstancePtr,&status_reg);
    // retval = FlashQspi_ReadRDCR(InstancePtr,&config_reg);

//	 retval = FlashQspi_CMD_WRDI(InstancePtr);
    // retval = FlashQspi_ReadRDSR(InstancePtr,&status_reg);
    // retval = FlashQspi_ReadRDCR(InstancePtr,&config_reg);
//////////////
    // retval = FlashQspi_CMD_WREN(InstancePtr);
    // retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);

    // retval = FlashQspi_CMD_ReadDataPage(InstancePtr, 0x8000,bufferRD);
    // retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);

    // retval = FlashQspi_CMD_SectorErase(InstancePtr, 0x8000);

    // retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);
    // retval = FlashQspi_CMD_ReadDataPage(InstancePtr, 0x8000,bufferRD);
    // retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);

    // retval = FlashQspi_CMD_WREN(InstancePtr);

    // retval = FlashQspi_CMD_WriteDataPage(InstancePtr, 0x8000,bufferWR);

    // retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);
    // while(status_reg&1 > 0)
    // {
    //     retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);
    // }
    // retval = FlashQspi_CMD_ReadDataPage(InstancePtr, 0x8000,bufferRD);
    // retval = FlashQspi_CMD_WRDI(InstancePtr);

//    FlashQspi_ReadPage(InstancePtr,0x8000,bufferRD);
//    FlashQspi_EraseSector(InstancePtr,0x8000);
//    FlashQspi_ReadPage(InstancePtr,0x8000,bufferRD);
//    FlashQspi_ProgramPage(InstancePtr,0x8000,bufferWR);
//    FlashQspi_ReadPage(InstancePtr,0x8000,bufferRD);



}
*/

/*
int FlashQspi_CMD(XSpi *InstancePtr, u8 cmd)
{
	u8 sendbuf[1]={0};
	sendbuf[0] = cmd;

	return XSpi_Transfer(InstancePtr, sendbuf, NULL, 1);
}
*/


int FlashQspi_CMD_ReadRDSR(uint8_t* Data)
{
	uint8_t sendbuf[5]={0};
	uint8_t recvbuf[5]={0};
    int retval = 0;


    sendbuf[0] = RDSR;

    while ((flash_spi_status_read() & 0x1) == 0);
    //retval = XSpi_Transfer(InstancePtr, sendbuf, recvbuf, 3);
    Spi_Transfer(sendbuf, recvbuf, 3);
    //Flash memory repeats status register two times
    *Data = recvbuf[1];

    return retval;
}

 int FlashQspi_CMD_ReadRDCR(uint8_t* Data)
 {
 	uint8_t sendbuf[5]={0};
 	uint8_t recvbuf[5]={0};
     int retval;


     sendbuf[0] = RDCR;

     while ((spimaster_status_read() & 0x1) == 0);
     //retval = XSpi_Transfer(InstancePtr, sendbuf, recvbuf, 3);
     Spi_Transfer(sendbuf, recvbuf, 3);
     //Flash memory repeats status register two times
     *Data = recvbuf[1];

     return retval;
 }

void Spi_Transfer(uint8_t * sendbuff, uint8_t * recvbuff, unsigned int ByteCount) {

    if (ByteCount > SPI_MAX_TRANSFER_SIZE) {
        // Handle error: ByteCount exceeds maximum transfer size
    	printf("FPGA_FLASH_ERROR: 1");
        return;
    }

	uint64_t send_data = 0;
	uint64_t rec_data = 0;
	// Convert array of uint8_t to uint64_t type variable end also reverse byte order
	for (int i =0; i < ByteCount; i++){
		send_data |= ((uint64_t)sendbuff[i]) << (8*(SPI_MAX_TRANSFER_SIZE-1-i));
	}

	while ((flash_spi_status_read() & 0x1) == 0);
	flash_spi_mosi_write(send_data);
	flash_spi_control_write(ByteCount*8*SPI_LENGTH | SPI_START);
	while ((flash_spi_status_read() & 0x1) == 0);


	if (recvbuff != NULL) {
		rec_data = flash_spi_miso_read();
		// Convert rec_data back to recvbuff array and reverse byte order
		for (unsigned int i = 0; i < ByteCount; i++) {
			recvbuff[i] = (rec_data >> (8 * (ByteCount - 1 - i))) & 0xFF;
			printf("0x%02x\n", recvbuff[i]);
		}
	}

}



void FlashQspi_CMD_WREN(void)
{
    uint8_t sendbuf[1]={0};
    //Write enable command
    sendbuf[0] = WREN;
    //return XSpi_Transfer(InstancePtr, sendbuf, NULL, 1);
    Spi_Transfer(sendbuf, NULL, 1);
}



void FlashQspi_CMD_WRDI(void)
{
    uint8_t sendbuf[1]={0};
    //Write disable command
    sendbuf[0] = WRDI;

    //return XSpi_Transfer(InstancePtr, sendbuf, NULL, 1);
    Spi_Transfer(sendbuf, NULL, 1);
}


/*
int FlashQspi_CMD_EnQPI(XSpi *InstancePtr)
{
	u8 sendbuf[1]={0};
	//Enable quad mode command
	sendbuf[0] = 0x35;

	return XSpi_Transfer(InstancePtr, sendbuf, NULL, 1);
}
*/

/*
int FlashQspi_CMD_DisQPI(XSpi *InstancePtr)
{
	u8 sendbuf[1]={0};
	//Enable quad mode command
	sendbuf[0] = 0xF5;

	return XSpi_Transfer(InstancePtr, sendbuf, NULL, 1);
}
*/

/*
int FlashQspi_CMD_WRSR(XSpi *InstancePtr, u8 StatusReg, u8 ConfigReg)
{
	u8 sendbuf[3]={0};
	//set command and output data
	sendbuf[0] = 0x1;
	sendbuf[1] = StatusReg;
	sendbuf[2] = ConfigReg;

	return XSpi_Transfer(InstancePtr, sendbuf, NULL, 3);
}
*/

/*
int FlashQspi_CMD_ReadRDCR(XSpi *InstancePtr, u8* Data)
{
    u8 sendbuf[3]={0};
    u8 recvbuf[3]={0};
    int retval;

    sendbuf[0] = 0x15;

    retval = XSpi_Transfer(InstancePtr, sendbuf, recvbuf, 3);
    //Flash memory repeats status register two times
    *Data = recvbuf[1];

    return retval;
}
*/

/*
int FlashQspi_CMD_ReadDataPage(XSpi *InstancePtr, u32 address, u8* buffer)
{
    // 256 bytes in a page, 1 byte command, 3 byte address
//    u8 sendbuf[260] = {0};
    u8 recvbuf[260] = {0};
    int retval;

    recvbuf[0] = 0x03;
    recvbuf[1] = (address >> 16)&0xff;
    recvbuf[2] = (address >> 8)&0xff;
    recvbuf[3] = 0;//(address )&0xff;

    retval = XSpi_Transfer(InstancePtr, recvbuf, recvbuf, 260);

    for(int i=4; i<260; i++)
    {
        buffer[i-4] = recvbuf[i];
    }
    
    return retval;
}
*/

void FlashQspi_CMD_ReadDataByte(uint32_t address, uint8_t* buffer)
{
	uint8_t sendbuf[5]={0};
	uint8_t recvbuf[5]={0};
    int retval;

    sendbuf[0] = 0x03;
    sendbuf[1] = (address >> 16) & 0xff;
    sendbuf[2] = (address >>  8) & 0xff;
    sendbuf[3] = (address      ) & 0xff;

    //retval = XSpi_Transfer(InstancePtr, recvbuf, recvbuf, 260);
    Spi_Transfer(sendbuf, recvbuf, 5);

    *buffer = recvbuf[4];
    /*
    for(int i=0; i<; i++)
    {
        buffer[i-4] = recvbuf[i];
    }
    */

}


/******************************************************************************
*
* Read One Time Programmable flash memory region
*
* @param	InstancePtr
* @param	address 	- starting address
* @param	bytes 		- bytes to read, up to 256bytes
* @param	buffer		- buffer where to store data
*
* @return	XST_SUCCESS if Successful else XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
/*
int FlashQspi_CMD_ReadOTPData(XSpi *InstancePtr, u32 address, u8 bytes, u8* buffer)
{
    // 256 bytes in a page, 1 byte command, 3 byte address
//    u8 sendbuf[260] = {0};
    u8 recvbuf[260] = {0};
    int TotalByteCount = bytes + 4;
    int retval;

    // Enter secured OTP
    retval = FlashQspi_CMD(InstancePtr, ENSO);

    recvbuf[0] = 0x03;
    recvbuf[1] = (address >> 16)&0xff;
    recvbuf[2] = (address >> 8)&0xff;
    recvbuf[3] = (address )&0xff;

    retval = XSpi_Transfer(InstancePtr, recvbuf, recvbuf, TotalByteCount);

    for(int i=4; i<TotalByteCount; i++)
    {
        buffer[i-4] = recvbuf[i];
    }

    // Exit secured OTP
    retval = FlashQspi_CMD(InstancePtr, EXSO);

    return retval;
}

*/

/*
int FlashQspi_CMD_WriteDataPage(XSpi *InstancePtr, u32 address, u8* buffer)
{
    // 256 bytes in a page, 1 byte command, 3 byte address
    u8 sendbuf[260] = {0};
//    u8 recvbuf[260] = {0};
    int retval;

    sendbuf[0] = 0x02;
    sendbuf[1] = (address >> 16)&0xff;
    sendbuf[2] = (address >> 8)&0xff;
    sendbuf[3] = 0;//(address)&0xff;

    for(int i=4; i<260; i++)
    {
        sendbuf[i] = buffer[i-4];
    }

    retval = XSpi_Transfer(InstancePtr, sendbuf, NULL, 260);


    
    return retval;
}
*/

/*
int FlashQspi_CMD_PageProgram(XSpi *InstancePtr, u32 address, u8 bytes, u8* buffer)
{
	// One to 256 bytes can be sent to the device to be programmed

    // sendbuf[260] = 256 bytes in a page, 1 byte command, 3 byte address
    u8 sendbuf[260] = {0};

    int TotalByteCount = bytes + 4;
    int retval;

    sendbuf[0] = 0x02;
    sendbuf[1] = (address >> 16)&0xff;
    sendbuf[2] = (address >> 8)&0xff;
    sendbuf[3] = (address)&0xff;

    for(int i=4; i<TotalByteCount; i++)
    {
        sendbuf[i] = buffer[i-4];
    }

    retval = XSpi_Transfer(InstancePtr, sendbuf, NULL, TotalByteCount);



    return retval;
}
*/

int FlashQspi_CMD_PageProgramByte(uint32_t address, uint8_t* byte)
{
	// One byte can be sent to the device to be programmed
    uint8_t sendbuf[5] = {0};
    uint8_t status = 0;

    int retval;

    sendbuf[0] = PP;
    sendbuf[1] = (address >> 16)&0xff;
    sendbuf[2] = (address >> 8)&0xff;
    sendbuf[3] = (address)&0xff;
    sendbuf[4] = *byte;

    Spi_Transfer(sendbuf, NULL, 5);
    retval = FlashQspi_CMD_ReadRDSR(&status);
    printf("Erase Status: 0x%02x\n", status);
    // Wait for program to complete
    while(status & 0x1) {
    	retval = FlashQspi_CMD_ReadRDSR(&status);
    	printf("Programm Status: 0x%02x\n", status);
    }




    return retval;
}


int FlashQspi_CMD_SectorErase(uint32_t address)
{
    uint8_t sendbuf[5] = {0};
    uint8_t status = 0;
    int retval;

    sendbuf[0] = 0x20;
    sendbuf[1] = (address >> 16) & 0xff;
    sendbuf[2] = (address >> 8)  & 0xff;
    sendbuf[3] = 0;//(address)&0xff;

    Spi_Transfer(sendbuf, NULL, 4);

    retval = FlashQspi_CMD_ReadRDSR(&status);
    printf("Erase Status: 0x%02x\n", status);
    // Wait for erase to complete
    while(status & 0x1) {
    	retval = FlashQspi_CMD_ReadRDSR(&status);
    	printf("Erase Status: 0x%02x\n", status);
    }
    return retval;

}



/*
int FlashQspi_ProgramPage(XSpi *InstancePtr, u32 address, u8* data)
{
    u8 status_reg = 0;
    int retval;
    //Set Write enable
    do
    {
        retval = FlashQspi_CMD_WREN(InstancePtr);
        if(retval != XST_SUCCESS) return retval;
        retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg); 
        if(retval != XST_SUCCESS) return retval;       
    } while((status_reg&2) == 0); //Check write enable
    //Perform write
    retval = FlashQspi_CMD_WriteDataPage(InstancePtr,address,data);
    if(retval != XST_SUCCESS) return retval;   
    //Check if flash is busy
    do
    {
        retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg); 
        if(retval != XST_SUCCESS) return retval;  
    } while((status_reg&1) == 1);
    //Return success if no problems
    return XST_SUCCESS;
}
*/

/*
int FlashQspi_ProgramOTP(XSpi *InstancePtr, u32 address, u8 bytes, u8* data)
{
    u8 status_reg = 0;
    int retval;

    // Enter secured OTP
    retval = FlashQspi_CMD(InstancePtr, ENSO);

    //Set Write enable
    do
    {
        retval = FlashQspi_CMD_WREN(InstancePtr);
        if(retval != XST_SUCCESS) return retval;
        retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);
        if(retval != XST_SUCCESS) return retval;
    } while((status_reg&2) == 0); //Check write enable

    //Perform write
    retval = FlashQspi_CMD_PageProgram(InstancePtr,address, bytes, data);
    if(retval != XST_SUCCESS) return retval;
    //Check if flash is busy
    do
    {
        retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg);
        if(retval != XST_SUCCESS) return retval;
    } while((status_reg&1) == 1);

    // Exit secured OTP
    retval = FlashQspi_CMD(InstancePtr, EXSO);
    //Return success if no problems
    return XST_SUCCESS;
}
*/

/*
int FlashQspi_EraseSector(XSpi *InstancePtr, u32 address)
{
    u8 status_reg = 0;
    int retval;
    //Set Write enable
    do
    {
        retval = FlashQspi_CMD_WREN(InstancePtr);
        if(retval != XST_SUCCESS) return retval;
        retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg); 
        if(retval != XST_SUCCESS) return retval;       
    } while((status_reg&2) == 0); //Check write enable
    // Perform erase
    retval = FlashQspi_CMD_SectorErase(InstancePtr,address);
    if(retval != XST_SUCCESS) return retval;   
    // Check if flash is busy
    do
    {
        retval = FlashQspi_CMD_ReadRDSR(InstancePtr,&status_reg); 
        if(retval != XST_SUCCESS) return retval;  
    } while((status_reg&1) == 1);
    //Return success if no problems
    return XST_SUCCESS;
}
*/

/*
int FlashQspi_ReadPage(XSpi *InstancePtr, u32 address, u8* data)
{
    //No additional operations are needed
    return FlashQspi_CMD_ReadDataPage(InstancePtr,address,data);
}
*/


