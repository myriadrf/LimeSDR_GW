from litex.gen import *
from litex.soc.interconnect.csr import CSRStorage, CSRStatus


# tst_top (test top) -----------------------------------------------------------------

class TST_TOP_LimeSDR_USB(LiteXModule):
    def __init__(self, platform, ddr_test_pads=None, add_ddr_test=False):
      self.ddr_test_pads = ddr_test_pads
      self.add_ddr_test = add_ddr_test
      # ASSIGN value to this OUTSIDE this module
      self.adf_muxout = Signal()
      # tstcfg
      self.test_en          = CSRStorage(size=6, description="Test enable")
      self.test_frc_err     = CSRStorage(size=6, description="Test force error")
      self.test_cmplt       = CSRStatus(size=6, description="Test complete")
      self.test_rez         = CSRStatus(size=6, description="Test result")
      self.ddr2_2_tst_fail  = CSRStatus(size=1, description="DDR2 2 test fail")
      self.ddr2_pnf_per_bit = CSRStatus(size=32, description="DDR2 2 PNF per bit")
      self.ddr2_1_tst_fail  = CSRStatus(size=1, description="DDR2 1 test fail")
      self.ddr2_1_pnf_per_bit = CSRStatus(size=32, description="DDR2 1 PNF per bit")

      self.fx3_clk_cnt    = CSRStatus(size=16, description="FX3 clock counter")
      self.si_clk0_cnt    = CSRStatus(size=16, description="Si5351C clock 0 counter")
      self.si_clk1_cnt    = CSRStatus(size=16, description="Si5351C clock 1 counter")
      self.si_clk2_cnt    = CSRStatus(size=16, description="Si5351C clock 2 counter")
      self.si_clk3_cnt    = CSRStatus(size=16, description="Si5351C clock 3 counter")
      self.si_clk5_cnt    = CSRStatus(size=16, description="Si5351C clock 5 counter")
      self.si_clk6_cnt    = CSRStatus(size=16, description="Si5351C clock 6 counter")
      self.si_clk7_cnt    = CSRStatus(size=16, description="Si5351C clock 7 counter")
      self.lmk_clk_cnt    = CSRStatus(size=23, description="LMK clock counter")
      self.adf_muxout_cnt = CSRStatus(size=16, description="ADF MUXOUT counter")

      # workaround to enable assigning values outside this module
      self.test_cmplt_clk_test = Signal(4)
      self.test_rez_clk_test = Signal(4)
      self.test_cmplt_ddr_test = Signal(1)
      self.test_rez_ddr_test = Signal(1)
      self.test_cmplt_wfm_player = Signal(1)
      self.test_rez_wfm_player = Signal(1)
      self.comb += [
          self.test_cmplt.status.eq(Cat(self.test_cmplt_clk_test, self.test_cmplt_wfm_player, self.test_cmplt_ddr_test)),
          self.test_rez.status.eq(Cat(self.test_rez_clk_test, self.test_rez_wfm_player, self.test_rez_ddr_test)),
      ]

      self.specials += Instance("clock_test",

      # --input ports
        i_FX3_clk       		= ClockSignal("sys"),
        i_reset_n   	 		= ~ResetSignal("sys"),
        i_test_en				= self.test_en.storage[0:4],
        i_test_frc_err		    = self.test_frc_err.storage[0:4],
        o_test_cmplt			= self.test_cmplt_clk_test,
        o_test_rez			    = self.test_rez_clk_test,

        i_Si5351C_clk_0 		= ClockSignal("si0"),
        i_Si5351C_clk_1 		= ClockSignal("si1"),
        i_Si5351C_clk_2 		= ClockSignal("si2"),
        i_Si5351C_clk_3 		= ClockSignal("si3"),
        #  No Si4 clock
        i_Si5351C_clk_5 		= ClockSignal("si5"),
        i_Si5351C_clk_6 		= ClockSignal("si6"),
        i_Si5351C_clk_7 		= ClockSignal("si7"),
        i_LMK_CLK		 		= ClockSignal("lmk"),
        i_ADF_MUXOUT	 		= self.adf_muxout,

        o_FX3_clk_cnt   		= self.fx3_clk_cnt.status,
        o_Si5351C_clk_0_cnt 	= self.si_clk0_cnt.status,
        o_Si5351C_clk_1_cnt 	= self.si_clk1_cnt.status,
        o_Si5351C_clk_2_cnt 	= self.si_clk2_cnt.status,
        o_Si5351C_clk_3_cnt 	= self.si_clk3_cnt.status,
        o_Si5351C_clk_5_cnt 	= self.si_clk5_cnt.status,
        o_Si5351C_clk_6_cnt 	= self.si_clk6_cnt.status,
        o_Si5351C_clk_7_cnt 	= self.si_clk7_cnt.status,
        o_LMK_CLK_cnt		 	= self.lmk_clk_cnt.status,
        o_ADF_MUXOUT_cnt	 	= self.adf_muxout_cnt.status,
      )

      self.add_sources_clock_test(platform)

      if self.add_ddr_test:
        assert ddr_test_pads is not None

        self.specials += Instance("ddr2_tester",
            # Inputs
            i_global_reset_n    = self.test_en.storage[5],
            i_pll_ref_clk       = ClockSignal("si1"),
            i_soft_reset_n      = self.test_en.storage[5],
            i_begin_test        = Constant(0),
            i_insert_error      = self.test_frc_err.storage[5],

            # Outputs
            o_mem_odt           = self.ddr_test_pads.odt,
            o_mem_cs_n          = self.ddr_test_pads.cs_n,
            o_mem_cke           = self.ddr_test_pads.cke,
            o_mem_addr          = self.ddr_test_pads.a,
            o_mem_ba            = self.ddr_test_pads.ba,
            o_mem_ras_n         = self.ddr_test_pads.ras_n,
            o_mem_cas_n         = self.ddr_test_pads.cas_n,
            o_mem_we_n          = self.ddr_test_pads.we_n,
            o_mem_dm            = self.ddr_test_pads.dm,
            io_mem_clk           = self.ddr_test_pads.clk,
            io_mem_clk_n         = self.ddr_test_pads.clk_n,
            io_mem_dq            = self.ddr_test_pads.dq,
            io_mem_dqs           = self.ddr_test_pads.dqs,

            o_pnf_per_bit         = Open(),
            o_pnf_per_bit_persist = self.ddr2_pnf_per_bit.status,
            o_pass                = self.test_rez_ddr_test,
            o_fail                = self.ddr2_2_tst_fail.status,
            o_test_complete       = self.test_cmplt_ddr_test,
        )



    def add_sources_clock_test(self, platform):

      # board specific files
      tst_top_files = [
        "gateware/board_specific/limesdr_usb/clock_test.vhd",
      ]
      # LimeDFB files
      tst_top_files += [
        "gateware/LimeDFB/self_test/transition_count.vhd",
        "gateware/LimeDFB/self_test/clk_no_ref_test.vhd",
        "gateware/LimeDFB/self_test/singl_clk_with_ref_test.vhd",
        "gateware/LimeDFB/self_test/clk_with_ref_test.vhd",
      ]

      for file in tst_top_files:
        platform.add_source(file)

