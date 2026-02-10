//
// Created by ts on 8/7/24.
//
#include "stdint.h"
//#include <system.h>
#include <generated/csr.h>

#ifndef ED_LIMESDR_XTRX_LITEX_GW_XIL_CLK_DRP_H
#define ED_LIMESDR_XTRX_LITEX_GW_XIL_CLK_DRP_H


#define add_value(base,suffix) base##suffix

#define GENERATE_CLK_CTRL_ADDRS(base)        \
    {                                        \
    add_value(base,_PLLCFG_DONE_ADDR),       \
    add_value(base,_PHCFG_MODE_ADDR),        \
    add_value(base,_PHCFG_DONE_ADDR),        \
    add_value(base,_PHCFG_ERR_ADDR),         \
    add_value(base,_PLLCFG_BUSY_ADDR),       \
    add_value(base,_PLLCFG_START_ADDR),      \
    add_value(base,_PLLRST_START_ADDR),      \
    add_value(base,_PLL_IND_ADDR),           \
    add_value(base,_PHCFG_START_ADDR),       \
    add_value(base,_PLLCFG_ERROR_ADDR),	     \
	add_value(base,_VCO_MULT_BYP_ADDR),      \
    add_value(base,_VCO_DIV_BYP_ADDR ),      \
    add_value(base,_C0_DIV_BYP_ADDR ),       \
    add_value(base,_C1_DIV_BYP_ADDR ),       \
    add_value(base,_VCO_DIV_CNT_ADDR ),      \
    add_value(base,_VCO_MULT_CNT_ADDR ),     \
    add_value(base,_C0_DIV_CNT_ADDR ),       \
    add_value(base,_C1_DIV_CNT_ADDR ),       \
    add_value(base,_C1_PHASE_ADDR),          \
    add_value(base,_AUTO_PHCFG_SMPLS_ADDR)   \
    }

typedef struct {
    unsigned long pllcfg_done;
    unsigned long phcfg_mode;
    unsigned long phcfg_done;
    unsigned long phcfg_err;
    unsigned long pllcfg_busy;
    unsigned long pllcfg_start;
    unsigned long pllrst_start;
    unsigned long pll_ind;
    unsigned long phcfg_start;
    unsigned long pllcfg_error;
    unsigned long vco_mult_byp;
    unsigned long vco_div_byp;
    unsigned long c0_div_byp;
    unsigned long c1_div_byp;
    unsigned long vco_div_cnt;
    unsigned long vco_mult_cnt;
    unsigned long c0_div_cnt;
    unsigned long c1_div_cnt;
    unsigned long c1_phase;
    unsigned long phcfg_samples;
} CLK_CTRL_ADDRS;


#define GENERATE_MMCM_DRP_ADDRS(base)        \
    {                                        \
    add_value(base,_CSR_RESET_ADDR),         \
    add_value(base,_DRP_LOCKED_ADDR),        \
    add_value(base,_DRP_READ_ADDR),          \
    add_value(base,_DRP_WRITE_ADDR),         \
    add_value(base,_DRP_ADR_ADDR),           \
    add_value(base,_DRP_DAT_W_ADDR),         \
    add_value(base,_DRP_DAT_R_ADDR),         \
    add_value(base,_LATCHED_DRDY_ADDR),      \
    add_value(base,_LATCHED_DRDY_RESET_ADDR) \
    }

typedef struct {
    unsigned long reset;
    unsigned long locked;
    unsigned long read;
    unsigned long write;
    unsigned long adr;
    unsigned long dat_w;
    unsigned long dat_r;
    unsigned long drdy;
    unsigned long drdy_reset;
} PLL_ADDRS;

#define GENERATE_SMPL_CMP_ADDRS(base) \
    {                                 \
    add_value(base,_CMP_START_ADDR),  \
    add_value(base,_CMP_LENGTH_ADDR), \
    add_value(base,_CMP_DONE_ADDR),   \
    add_value(base,_CMP_ERROR_ADDR)   \
    }

typedef struct {
    unsigned long cmp_start;
    unsigned long cmp_length;
    unsigned long cmp_done;
    unsigned long cmp_error;
} SMPL_CMP_ADDRS;


typedef struct {
    uint8_t DIVCLK_DIVIDE;
    uint8_t CLKFBOUT_MULT;
    //    uint32_t CLKFBOUT_FRAC; --NOT IMPLEMENTED
    uint16_t CLKFBOUT_PHASE;

    uint8_t CLKOUT0_DIVIDE;
    //    uint32_t CLKOUT0_FRAC; --NOT IMPLEMENTED
    int16_t CLKOUT0_PHASE;
    //    uint32_t CLKOUT0_DUTY; --NOT IMPLEMENTED

    uint8_t CLKOUT1_DIVIDE;
    uint16_t CLKOUT1_PHASE;
    //    uint32_t CLKOUT1_DUTY; --NOT IMPLEMENTED

    uint8_t CLKOUT2_DIVIDE;
    uint16_t CLKOUT2_PHASE;
    //    uint32_t CLKOUT2_DUTY; --NOT IMPLEMENTED

    uint8_t CLKOUT3_DIVIDE;
    uint16_t CLKOUT3_PHASE;
    //    uint32_t CLKOUT3_DUTY; --NOT IMPLEMENTED

    uint8_t CLKOUT4_DIVIDE;
    uint16_t CLKOUT4_PHASE;
    //    uint32_t CLKOUT4_DUTY; --NOT IMPLEMENTED

    uint8_t CLKOUT5_DIVIDE;
    uint16_t CLKOUT5_PHASE;
    //    uint32_t CLKOUT5_DUTY; --NOT IMPLEMENTED

    uint8_t CLKOUT6_DIVIDE;
    uint16_t CLKOUT6_PHASE;
    //    uint32_t CLKOUT6_DUTY; --NOT IMPLEMENTED
} tXPLL_CFG;


#define ClkReg1_CLKOUT5 0x06
#define ClkReg2_CLKOUT5 0x07
#define ClkReg1_CLKOUT0 0x08
#define ClkReg2_CLKOUT0 0x09
#define ClkReg1_CLKOUT1 0x0A
#define ClkReg2_CLKOUT1 0x0B
#define ClkReg1_CLKOUT2 0x0C
#define ClkReg2_CLKOUT2 0x0D
#define ClkReg1_CLKOUT3 0x0E
#define ClkReg2_CLKOUT3 0x0F
#define ClkReg1_CLKOUT4 0x10
#define ClkReg2_CLKOUT4 0x11
#define ClkReg1_CLKOUT6 0x12
#define ClkReg2_CLKOUT6 0x13
#define ClkReg1_CLKFBOUT 0x14
#define ClkReg2_CLKFBOUT 0x15
#define DivReg_DIVCLK 0x16
#define LockReg1 0x18

#define LockReg2 0x19
#define LockReg3 0x1A
#define PowerReg_UltraScale 0x27
#define PowerReg_7Series 0x28
#define FiltReg1 0x4E
#define FiltReg2 0x4F

typedef enum {
    AUTO_PH_MMCM_CFG_SUCCESS,
    AUTO_PH_MMCM_CFG_FAILURE,
    AUTO_PH_MMCM_CFG_TIMEOUT
} AutoPH_MMCM_CFG_Status;

uint16_t Read_MMCM_DRP(PLL_ADDRS *addresses, uint16_t Addr);

void Write_MMCM_DRP(PLL_ADDRS *addresses, uint16_t Addr, uint16_t Val);

void SetPhase_DRP(PLL_ADDRS *addresses, uint8_t phase_mux, uint8_t delay_time, uint16_t clkreg1_adr,
                  uint16_t clkreg2_adr);

void Reset_Drdy_latch(uint16_t reset_addr);

void SetMMCM_CLKREG(PLL_ADDRS *addresses, uint8_t DIVIDE, uint32_t PHASE, uint16_t clkreg1_adr, uint16_t clkreg2_adr);

void Update_MMCM_CFG(PLL_ADDRS *pll_addresses, CLK_CTRL_ADDRS *ctrl_addresses);

int AutoPH_MMCM_CFG(PLL_ADDRS *pll_addresses, CLK_CTRL_ADDRS *ctrl_addresses, SMPL_CMP_ADDRS *smpl_cmp_addrs);


#endif //ED_LIMESDR_XTRX_LITEX_GW_XIL_CLK_DRP_H
