#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.common            import add_vhd2v_converter

# MAX10 PLL Top ------------------------------------------------------------------------------------

class MAX10PLLTop(LiteXModule):
    def __init__(self, platform, pads, pllcfg_manager):

        self.platform              = platform

        self.pll_c1      = Signal()
        self.pll_c3      = Signal()
        self.pll_locked  = Signal()

        # fpgacfg
        self.clk_ena     = Signal(4)
        self.drct_clk_en = Signal(4)

        # smpl cmp
        self.smpl_cmp_en    = Signal()
        self.smpl_cmp_done  = Signal()
        self.smpl_cmp_error = Signal()
        self.smpl_cmp_cnt   = Signal(16)

        # # #

        # pll_top instance.
        # -----------------
        self.specials += Instance("pll_top",
            # Parameters
            p_N_PLL                  = 1,
            # PLL parameters       
            p_BANDWIDTH_TYPE         = "AUTO",
            p_CLK0_DIVIDE_BY         = 1,
            p_CLK0_DUTY_CYCLE        = 50,
            p_CLK0_MULTIPLY_BY       = 1,
            p_CLK0_PHASE_SHIFT       = "0",
            p_CLK1_DIVIDE_BY         = 1,
            p_CLK1_DUTY_CYCLE        = 50,
            p_CLK1_MULTIPLY_BY       = 1,
            p_CLK1_PHASE_SHIFT       = "0",
            p_CLK2_DIVIDE_BY         = 1,
            p_CLK2_DUTY_CYCLE        = 50,
            p_CLK2_MULTIPLY_BY       = 1,
            p_CLK2_PHASE_SHIFT       = "0",
            p_CLK3_DIVIDE_BY         = 1,
            p_CLK3_DUTY_CYCLE        = 50,
            p_CLK3_MULTIPLY_BY       = 1,
            p_CLK3_PHASE_SHIFT       = "0",
            p_COMPENSATE_CLOCK       = "CLK3",
            p_INCLK0_INPUT_FREQUENCY = 6250,
            p_INTENDED_DEVICE_FAMILY = "MAX 10",
            p_OPERATION_MODE         = "NORMAL",
            p_SCAN_CHAIN_MIF_FILE    = "ip/pll/pll.mif",
            p_DRCT_C0_NDLY           = 1,
            p_DRCT_C1_NDLY           = 8,
            p_DRCT_C2_NDLY           = 1,
            p_DRCT_C3_NDLY           = 8,
            # PLL ports
            i_pll_inclk              = pads.MCLK2,
            i_pll_reconfig_clk       = ClockSignal("sys"), # LMK_CLK
            i_pll_logic_reset_n      = ~ResetSignal("sys"),
            i_pll_clk_ena            = self.clk_ena,
            i_pll_drct_clk_en        = self.drct_clk_en,
            o_pll_c0                 = pads.FCLK1,
            o_pll_c1                 = self.pll_c1,
            o_pll_c2                 = pads.FCLK2,
            o_pll_c3                 = self.pll_c3,
            o_pll_locked             = self.pll_locked,
            o_pll_smpl_cmp_en        = self.smpl_cmp_en,#inst1_pll_smpl_cmp_en,
            i_pll_smpl_cmp_done      = self.smpl_cmp_done,#inst6_rx_smpl_cmp_done,
            i_pll_smpl_cmp_error     = self.smpl_cmp_error,#inst6_rx_smpl_cmp_err,
            o_pll_smpl_cmp_cnt       = self.smpl_cmp_cnt,#inst1_pll_smpl_cmp_cnt,
            # pllcfg ports
            # from pllcfg
            i_phcfg_start            = pllcfg_manager.phcfg_start,
            i_pllcfg_start           = pllcfg_manager.pllcfg_start,
            i_pllrst_start           = pllcfg_manager.pllrst_start,
            i_phcfg_updn             = pllcfg_manager.phcfg_updn,
            i_cnt_ind                = pllcfg_manager.cnt_ind,
            i_pll_ind                = pllcfg_manager.pll_ind,
            i_phcfg_mode             = pllcfg_manager.phcfg_mode,
            i_phcfg_tst              = pllcfg_manager.phcfg_tst,
            i_cnt_phase              = pllcfg_manager.cnt_phase,
            i_chp_curr               = pllcfg_manager.chp_curr,
            i_pllcfg_vcodiv          = pllcfg_manager.pllcfg_vcodiv,
            i_pllcfg_lf_res          = pllcfg_manager.pllcfg_lf_res,
            i_pllcfg_lf_cap          = pllcfg_manager.pllcfg_lf_cap,
            i_m_odddiv               = pllcfg_manager.m_odddiv,
            i_m_byp                  = pllcfg_manager.m_byp,
            i_n_odddiv               = pllcfg_manager.n_odddiv,
            i_n_byp                  = pllcfg_manager.n_byp,
            i_c0_odddiv              = pllcfg_manager.c0_odddiv,
            i_c0_byp                 = pllcfg_manager.c0_byp,
            i_c1_odddiv              = pllcfg_manager.c1_odddiv,
            i_c1_byp                 = pllcfg_manager.c1_byp,
            i_c2_odddiv              = pllcfg_manager.c2_odddiv,
            i_c2_byp                 = pllcfg_manager.c2_byp,
            i_c3_odddiv              = pllcfg_manager.c3_odddiv,
            i_c3_byp                 = pllcfg_manager.c3_byp,
            i_c4_odddiv              = pllcfg_manager.c4_odddiv,
            i_c4_byp                 = pllcfg_manager.c4_byp,
            i_n_cnt                  = pllcfg_manager.n_cnt,
            i_m_cnt                  = pllcfg_manager.m_cnt,
            i_c0_cnt                 = pllcfg_manager.c0_cnt,
            i_c1_cnt                 = pllcfg_manager.c1_cnt,
            i_c2_cnt                 = pllcfg_manager.c2_cnt,
            i_c3_cnt                 = pllcfg_manager.c3_cnt,
            i_c4_cnt                 = pllcfg_manager.c4_cnt,
            i_auto_phcfg_smpls       = pllcfg_manager.auto_phcfg_smpls,
            i_auto_phcfg_step        = pllcfg_manager.auto_phcfg_step,

            # to pllcfg
            # Status Inputs
            o_pllcfg_busy            = pllcfg_manager.pllcfg_busy,
            o_pllcfg_done            = pllcfg_manager.pllcfg_done,
            o_phcfg_done             = pllcfg_manager.phcfg_done,
            o_phcfg_error            = pllcfg_manager.phcfg_error,
            # PLL Lock flags
            o_pll_lock               = pllcfg_manager.pll_lock,
        )

        self.add_sources(platform)

    def add_sources(self, platform):
        pll_top_files = [
            "gateware/max10_pll_top/pll_ctrl.vhd",
            "gateware/max10_pll_top/pll_top.vhd",
            "gateware/max10_pll_top/rxtx_pll.vhd",
        ]

        for file in pll_top_files:
            platform.add_source(file)
