/*
 * regremap.h
 *
 *  Created on: May 10, 2024
 *      Author: lab
 */

#ifndef REGREMAP_H_
#define REGREMAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "Xil_clk_drp.h"
#include <stdbool.h>

    extern CLK_CTRL_ADDRS clk_ctrl_addrs; // = GENERATE_CLK_CTRL_ADDRS(CSR_LIME_TOP_LMS7002_CLK_CTRL);
    extern SMPL_CMP_ADDRS smpl_cmp_addrs;
    volatile extern uint8_t var_phcfg_start;
    volatile extern uint8_t var_pllcfg_start;
    volatile extern uint8_t var_pllrst_start;

    // Return false for unsupported addresses; failed reads leave output unchanged.
    bool readCSR(uint8_t *address, uint8_t *regdata_array);

    bool writeCSR(uint8_t *address, uint8_t *regdata_array);

#endif /* REGREMAP_H_ */
