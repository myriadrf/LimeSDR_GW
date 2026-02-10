//
// Created by ts on 1/21/26.
//

#include "ADF4002.h"
#include "bsp.h"

void Control_TCXO_ADF (uint8_t spi_master, uint8_t spi_cs, uint8_t oe, uint8_t *data) //controls ADF4002
{
    unsigned char ADF_data[12], ADF_block;

    if (oe == 0) //set ADF4002 CP to three-state and MUX_OUT to DGND
    {
        ADF_data[0] = 0x1f;
        ADF_data[1] = 0x81;
        ADF_data[2] = 0xf3;
        ADF_data[3] = 0x1f;
        ADF_data[4] = 0x81;
        ADF_data[5] = 0xf2;
        ADF_data[6] = 0x00;
        ADF_data[7] = 0x01;
        ADF_data[8] = 0xf4;
        ADF_data[9] = 0x01;
        ADF_data[10] = 0x80;
        ADF_data[11] = 0x01;

        //Reconfigure_SPI_for_LMS();

        //write data to ADF
        for(ADF_block = 0; ADF_block < 4; ADF_block++)
        {
            bsp_spi_transfer(spi_master, spi_cs, &ADF_data[ADF_block*3], 3, 0, NULL);
        }
    }
    else //set PLL parameters
    {
        bsp_spi_transfer(spi_master, spi_cs, data, 3, 0, NULL);
    }
}