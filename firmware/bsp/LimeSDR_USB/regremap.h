/*
 * regremap.h
 */

#ifndef LimeSDR_USB_REGREMAP_H
#define LimeSDR_USB_REGREMAP_H

#include <stdint.h>
#include <generated/csr.h>
#include <stdbool.h>
#include "pll_ctrl.h"

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef CSR_LIMETOP_LMS7002_TOP_LMS7002_CLK_CLK_CTRL_PLLCFG_DONE_ADDR
    extern CLK_CTRL_ADDRS clk_ctrl_addrs;
#endif
    // Return false for unsupported addresses; failed reads leave output unchanged.
    bool readCSR(uint8_t *address, uint8_t *regdata_array);
    bool writeCSR(uint8_t *address, uint8_t *wrdata_array);

#ifdef __cplusplus
}
#endif

#endif /* LimeSDR_USB_REGREMAP_H */
