#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.common            import add_vhd2v_converter

# MAX10 PLL Top ------------------------------------------------------------------------------------

class MAX10PLLTop(LiteXModule):
    def __init__(self, platform, pads, pllcfg_manager,
        drct_c0_ndly = 1,
        drct_c1_ndly = 2,
        drct_c2_ndly = 1,
        drct_c3_ndly = 2,
        ):

        # c0: FCLK1
        # c1: EP03 PC -> FPGA (TX) (tx_clk)
        # c2: FCLK2
        # c3: EP83 FPGA -> PC (RX) (rx_clk)

        self.platform    = platform

        self.c0_global   = Signal()
        self.c2_global   = Signal()
        self.rx_clk      = Signal()
        self.tx_clk      = Signal()
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

        # Signals.
        # --------
        pll_c0 = Signal()
        pll_c1 = Signal()
        pll_c2 = Signal()
        pll_c3 = Signal()

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
            o_pll_c0                 = pll_c0,
            o_pll_c1                 = pll_c1,
            o_pll_c2                 = pll_c2,
            o_pll_c3                 = pll_c3,
            o_pll_locked             = self.pll_locked,
            o_pll_smpl_cmp_en        = self.smpl_cmp_en,
            i_pll_smpl_cmp_done      = self.smpl_cmp_done,
            i_pll_smpl_cmp_error     = self.smpl_cmp_error,
            o_pll_smpl_cmp_cnt       = self.smpl_cmp_cnt,
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
            o_pllcfg_busy_bit        = pllcfg_manager.pllcfg_busy,
            o_pllcfg_done_bit        = pllcfg_manager.pllcfg_done,
            o_auto_phcfg_done_bit    = pllcfg_manager.phcfg_done,
            o_auto_phcfg_err_bit     = pllcfg_manager.phcfg_error,
            # PLL Lock flags
            o_pll_lock_vect          = pllcfg_manager.pll_lock,
        )

        inst3_clk        = Signal(3)
        pll_inclk_global = Signal()

        # c0: FCLK1
        # c1: EP03 PC -> FPGA (TX) (tx_clk)
        # c2: FCLK2
        # c3: EP83 FPGA -> PC (RX) (rx_clk)

        # Clk.
        # ----
        self.specials += Instance("fiftyfivenm_clkctrl",
            p_clock_type        = "Global Clock",
            p_ena_register_mode = "falling edge",
            p_lpm_type          = "fiftyfivenm_clkctrl",

            i_inclk             = pads.MCLK2,
            i_clkselect         = Constant(0, 2),
            i_ena               = Constant(1, 1),
            o_outclk            = pll_inclk_global,
        )

        # TX.
        # ---
        drct_c0_dly_chain = Signal(drct_c0_ndly)
        c0_mux            = Signal()
        drct_c1_dly_chain = Signal(drct_c1_ndly)
        c1_mux            = Signal()
        c1_global         = Signal()

        for i in range(drct_c0_ndly):
            self.specials += Instance("lcell",
                i_in  = {True:pads.MCLK2, False:drct_c0_dly_chain[i-1]}[i==0],
                o_out = drct_c0_dly_chain[i],
            )
        for i in range(drct_c1_ndly):
            self.specials += Instance("lcell",
                i_in  = {True:pads.MCLK2, False:drct_c1_dly_chain[i-1]}[i==0],
                o_out = drct_c1_dly_chain[i],
            )
        self.specials += [
            Instance("fiftyfivenm_clkctrl",
                p_clock_type        = "Global Clock",
                p_ena_register_mode = "falling edge",
                p_lpm_type          = "fiftyfivenm_clkctrl",

                i_inclk             = c0_mux,
                i_clkselect         = Constant(0, 2),
                i_ena               = self.clk_ena[0],
                o_outclk            = self.c0_global,
            ),
            Instance("fiftyfivenm_clkctrl",
                p_clock_type        = "Global Clock",
                p_ena_register_mode = "falling edge",
                p_lpm_type          = "fiftyfivenm_clkctrl",

                i_inclk             = c1_mux,
                i_clkselect         = Constant(0, 2),
                i_ena               = self.clk_ena[0],
                o_outclk            = c1_global,
            ),
        ]

        self.comb += [
            If(self.drct_clk_en[0],
                c0_mux.eq(drct_c0_dly_chain[drct_c0_ndly-1]),
            ).Else(
                c0_mux.eq(pll_c0)
            ),
            If(self.drct_clk_en[1],
                c1_mux.eq(drct_c1_dly_chain[drct_c1_ndly-1]),
            ).Else(
                c1_mux.eq(pll_c1)
            ),
            self.tx_clk.eq(c1_global),
        ]

        # RX.
        # ---
        drct_c2_dly_chain = Signal(drct_c2_ndly)
        c2_mux            = Signal()
        drct_c3_dly_chain = Signal(drct_c3_ndly)
        c3_mux            = Signal()
        c3_global         = Signal()

        for i in range(drct_c2_ndly):
            self.specials += Instance("lcell",
                i_in  = {True:pll_inclk_global, False:drct_c2_dly_chain[i-1]}[i==0],
                o_out = drct_c2_dly_chain[i],
            )
        for i in range(drct_c2_ndly):
            self.specials += Instance("lcell",
                i_in  = {True:pll_inclk_global, False:drct_c3_dly_chain[i-1]}[i==0],
                o_out = drct_c3_dly_chain[i],
            )
        self.specials += [
            Instance("fiftyfivenm_clkctrl",
                p_clock_type        = "Global Clock",
                p_ena_register_mode = "falling edge",
                p_lpm_type          = "fiftyfivenm_clkctrl",

                i_inclk             = c2_mux,
                i_clkselect         = Constant(0, 2),
                i_ena               = self.clk_ena[2],
                o_outclk            = self.c2_global,
            ),
            Instance("fiftyfivenm_clkctrl",
                p_clock_type        = "Global Clock",
                p_ena_register_mode = "falling edge",
                p_lpm_type          = "fiftyfivenm_clkctrl",

                i_inclk             = c3_mux,
                i_clkselect         = Constant(0, 2),
                i_ena               = self.clk_ena[3],
                o_outclk            = c3_global,
            ),
        ]
        self.comb += [
            If(self.drct_clk_en[2],
                c2_mux.eq(drct_c2_dly_chain[drct_c2_ndly-1]),
            ).Else(
                c2_mux.eq(pll_c2),
            ),
            If(self.drct_clk_en[3],
                c3_mux.eq(drct_c3_dly_chain[drct_c3_ndly-1]),
            ).Else(
                c3_mux.eq(pll_c3),
            ),
            self.rx_clk.eq(c3_global),
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        pll_top_files = [
            "gateware/max10_pll_top/pll_ctrl.vhd",
            "gateware/max10_pll_top/pll_top.vhd",
            "gateware/max10_pll_top/rxtx_pll.vhd",
            "gateware/max10_pll_top/pll_reconfig_ctrl/config_ctrl.vhd",
            "gateware/max10_pll_top/pll_reconfig_ctrl/pll_reconfig_status.vhd",
            "gateware/max10_pll_top/pll_reconfig_module/pll_reconfig_module.vhd",
            "gateware/max10_pll_top/dyn_ps/synth/pll_ps_fsm.vhd",
            "gateware/max10_pll_top/dyn_ps/synth/pll_ps_top.vhd",
            "gateware/max10_pll_top/dyn_ps/synth/pll_ps.vhd",
        ]

        for file in pll_top_files:
            platform.add_source(file)
