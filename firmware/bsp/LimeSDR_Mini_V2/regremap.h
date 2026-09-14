/*
 * regremap.h
 */

#ifndef REGREMAP_H_
#define REGREMAP_H_

#include <stdint.h>
#include <stdbool.h>
#include "pll_ctrl.h"

#ifdef __cplusplus
extern "C"
{
#endif
    extern CLK_CTRL_ADDRS clk_ctrl_addrs;
    // Return false for unsupported addresses; failed reads leave output unchanged.
    bool readCSR(uint8_t *address, uint8_t *regdata_array);
    bool writeCSR(uint8_t *address, uint8_t *wrdata_array);
    uint16_t mini_get_bom_ver(void);

#ifdef __cplusplus
}
#endif

#endif /* REGREMAP_H_ */
