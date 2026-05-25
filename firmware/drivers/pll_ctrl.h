//
// Created by ts on 5/15/26.
//

#ifndef LIMESDR_GW_PLL_CTRL_H
#define LIMESDR_GW_PLL_CTRL_H

#    define add_value(base, suffix) base##suffix

#    define GENERATE_CLK_CTRL_ADDRS(base)                                                                              \
        {                                                                                                              \
            add_value(base, _PLLCFG_DONE_ADDR), add_value(base, _PHCFG_MODE_ADDR), add_value(base, _PHCFG_DONE_ADDR),  \
                add_value(base, _PHCFG_ERR_ADDR), add_value(base, _PLLCFG_BUSY_ADDR),                                  \
                add_value(base, _PLLCFG_START_ADDR),\
                add_value(base, _CNT_PHASE_ADDR),\
                add_value(base, _PLLCFG_VCODIV_ADDR),\
                add_value(base, _PLLRST_START_ADDR),                              \
                add_value(base, _PLL_IND_ADDR),\
                add_value(base, _CNT_IND_ADDR),\
                add_value(base, _PLL_LOCK_ADDR),\
                add_value(base, _PHCFG_START_ADDR),                                    \
                add_value(base, _PHCFG_UPDN_ADDR),\
                add_value(base, _PLLCFG_ERROR_ADDR), add_value(base, _VCO_MULT_BYP_ADDR),                              \
                add_value(base, _VCO_DIV_BYP_ADDR),\
                add_value(base, _M_ODD_DIV_ADDR),\
                add_value(base, _M_DIV_BYP_ADDR),\
                add_value(base, _N_ODD_DIV_ADDR),\
                add_value(base, _N_DIV_BYP_ADDR),\
                add_value(base, _C0_DIV_BYP_ADDR),                                                                     \
                add_value(base, _C1_DIV_BYP_ADDR),                                                                     \
                add_value(base, _C2_DIV_BYP_ADDR),                                                                     \
                add_value(base, _C3_DIV_BYP_ADDR),                                                                     \
                add_value(base, _C4_DIV_BYP_ADDR),                                                                     \
                add_value(base, _C0_ODDDIV_ADDR),\
                add_value(base, _C1_ODDDIV_ADDR),\
                add_value(base, _C2_ODDDIV_ADDR),\
                add_value(base, _C3_ODDDIV_ADDR),\
                add_value(base, _C4_ODDDIV_ADDR),\
                add_value(base, _VCO_DIV_CNT_ADDR),                                                                    \
                add_value(base, _VCO_MULT_CNT_ADDR),                                                                   \
                add_value(base, _C0_DIV_CNT_ADDR),                                                                     \
                add_value(base, _C1_DIV_CNT_ADDR),                                                                     \
                add_value(base, _C2_DIV_CNT_ADDR),                                                                     \
                add_value(base, _C3_DIV_CNT_ADDR),                                                                     \
                add_value(base, _C4_DIV_CNT_ADDR),                                                                     \
                add_value(base, _N_CNT_ADDR),\
                add_value(base, _M_CNT_ADDR),\
                add_value(base, _C1_PHASE_ADDR),                                                                       \
                add_value(base, _AUTO_PHCFG_SMPLS_ADDR),                                                                \
                add_value(base, _AUTO_PHCFG_STEP_ADDR)\
        }

typedef struct
{
    unsigned long pllcfg_done;
    unsigned long phcfg_mode;
    unsigned long phcfg_done;
    unsigned long phcfg_err;
    unsigned long pllcfg_busy;
    unsigned long pllcfg_start;
    unsigned long cnt_phase;
    unsigned long pllcfg_vcodiv;
    unsigned long pllrst_start;
    unsigned long pll_ind;
    unsigned long cnt_ind;
    unsigned long pll_lock;
    unsigned long phcfg_start;
    unsigned long phcfg_updn;
    unsigned long pllcfg_error;
    unsigned long vco_mult_byp;
    unsigned long vco_div_byp;
    unsigned long m_odd_div;
    unsigned long m_div_byp;
    unsigned long n_odd_div;
    unsigned long n_div_byp;
    unsigned long c0_div_byp;
    unsigned long c1_div_byp;
    unsigned long c2_div_byp;
    unsigned long c3_div_byp;
    unsigned long c4_div_byp;
    unsigned long c0_odddiv;
    unsigned long c1_odddiv;
    unsigned long c2_odddiv;
    unsigned long c3_odddiv;
    unsigned long c4_odddiv;
    unsigned long vco_div_cnt;
    unsigned long vco_mult_cnt;
    unsigned long c0_div_cnt;
    unsigned long c1_div_cnt;
    unsigned long c2_div_cnt;
    unsigned long c3_div_cnt;
    unsigned long c4_div_cnt;
    unsigned long n_cnt;
    unsigned long m_cnt;
    unsigned long c1_phase;
    unsigned long phcfg_samples;
    unsigned long phcfg_step;
} CLK_CTRL_ADDRS;

#endif //LIMESDR_GW_PLL_CTRL_H
