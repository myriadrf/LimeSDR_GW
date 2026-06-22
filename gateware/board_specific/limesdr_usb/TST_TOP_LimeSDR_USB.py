from litex.gen import *
from litex.soc.interconnect.csr import CSRStorage, CSRStatus


# tst_top (test top) -----------------------------------------------------------------

class TST_TOP_LimeSDR_USB(LiteXModule):
    def __init__(self, platform):

      # ASSIGN value to this OUTSIDE this module
      self.adf_muxout = Signal()

      self.test_en      = CSRStorage(size=4, description="Test enable")
      self.test_frc_err = CSRStorage(size=4, description="Test force error")
      self.test_cmplt   = CSRStatus(size=4, description="Test complete")
      self.test_rez     = CSRStatus(size=4, description="Test result")

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

      self.specials += Instance("clock_test",

      # --input ports
        i_FX3_clk       		= ClockSignal("sys"),
        i_reset_n   	 		= ~ResetSignal("sys"),
        i_test_en				= self.test_en.storage,
        i_test_frc_err		    = self.test_frc_err.storage,
        o_test_cmplt			= self.test_cmplt.status,
        o_test_rez			    = self.test_rez.status,

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

      self.add_sources(platform)

    def add_sources(self, platform):

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

