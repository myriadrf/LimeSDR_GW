//
// Created by ts on 6/16/25.
//

#include "TMP114.h"
#include <stdint.h>

void TMP114_Continuous_Conversion(const litei2c_regs *i2c_regs, const uint8_t addr, const bool enable) {
    uint32_t buf = 0x3;
    //Get existing register value
    litei2c_transfer(i2c_regs, addr, &buf, 1, 2);
    if (enable) {
        //Turn off Shutdown mode (turn on continuous conversion)
        //Two separate bit shifts for readability
        //Mode bit is bit 3, but in received data from litei2c_transfer,
        //LSB is actually bits [15:8] of buf
        buf = buf | (~((1 << 8) << 3));
        //Add register address value to buf
        //Address is shifted because of how litei2c_transfer works
        buf = buf | ((0x03) << 16);
        litei2c_transfer(i2c_regs, addr, &buf, 3, 0);
    }
    else {
        //Turn on Shutdown mode
        //Two separate bit shifts for readability
        //Mode bit is bit 3, but in received data from litei2c_transfer,
        //LSB is actually bits [15:8] of buf
        buf = buf | ((1 << 8) << 3);
        //Add register address value to buf
        //Address is shifted because of how litei2c_transfer works
        buf = buf | ((0x03) << 16);
        litei2c_transfer(i2c_regs, addr, &buf, 3, 0);
    }
}

uint16_t TMP114_Read_Temp(const litei2c_regs *i2c_regs, const uint8_t addr) {
    uint32_t buf = 0x0;
    const uint8_t* buf_point = (uint8_t*)&buf;
    litei2c_transfer(i2c_regs, addr, &buf, 1, 2);
    //rearrange data (refer to litei2c.h)
    const uint16_t temp = buf_point[1] | (buf_point[0] << 8);
    return temp;
}

uint16_t TMP114_Read_Temp_OneShot(const litei2c_regs *i2c_regs, const uint8_t addr) {
    uint32_t buf = 0x03;
    //Get existing register value
    litei2c_transfer(i2c_regs, addr, &buf, 1, 2);
    //Turn on One Shot bit
    //Two separate bit shifts for readability
    //OneShot bit is bit 4, but in received data from litei2c_transfer,
    //LSB is actually bits [15:8] of buf
    buf = buf | ((1 << 8) << 4);
    //Add register address value to buf
    //Address is shifted because of how litei2c_transfer works
    buf = buf | ((0x03) << 16);
    litei2c_transfer(i2c_regs, addr, &buf, 3, 0);
    while (1) {
        // Shutdown mode should be enabled after conversion is done, let's check the mode bit
        buf = 0x03;
        litei2c_transfer(i2c_regs, addr, &buf, 1, 2);
        if (buf & ((1 << 8) << 3)) {
            //Mode is set to Shutdown, One-Shot conversion finished
            //We can leave the loop, otherwise continue checking
            break;
        }
    }
    //Get the results and leave
    return TMP114_Read_Temp(i2c_regs, addr);
}

uint16_t TMP114_Convert_Temp(const uint16_t raw_temp) {
    // According to TMP114 datasheet, 1LSB of raw data is 0.0078125C
    // One of the example values is 3200 = 25 C
    // Doing the operations below we convert 3200 to 250
    // Which should be interpreted as 25.0 C
    uint16_t temp = raw_temp;
    temp = temp >> 4;
    temp = temp * 10;
    temp = temp >> 3;
    return temp;
}
