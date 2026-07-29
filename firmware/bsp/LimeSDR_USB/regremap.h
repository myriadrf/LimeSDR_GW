/*
 * regremap.h
 */

#ifndef LimeSDR_USB_REGREMAP_H
#define LimeSDR_USB_REGREMAP_H

#include <stdint.h>
#include "pll_ctrl.h"

#ifdef __cplusplus
extern "C"
{
#endif
    extern CLK_CTRL_ADDRS clk_ctrl_addrs;
    void readCSR(uint8_t *address, uint8_t *regdata_array);
    void writeCSR(uint8_t *address, uint8_t *wrdata_array);

#ifdef __cplusplus
}
#endif

#endif /* LimeSDR_USB_REGREMAP_H */
