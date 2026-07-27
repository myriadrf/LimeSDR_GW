from gateware.GpioTop import GpioTop
from gateware.board_specific.limesdr_usb.TST_TOP_LimeSDR_USB import TST_TOP_LimeSDR_USB
from gateware.board_specific.limesdr_usb.wfm_ram_buffer.wfm_player_top import WFMPlayerTop
from litei2c import LiteI2C
from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *

# PSS (Peripheral Support Subsystem) -----------------------------------------------------------------

class PSS_LimeSDR_Usb(LiteXModule):
    def __init__(self, soc, platform, sys_clk_freq, pll_ref_clk, add_ddr_modules=True, wfm_infifo_usedw_width=11):
        self.platform = platform

        self.adf_muxout = platform.request("ADF_MUXOUT")

        # SPI1 - TCXO DAC, ADF4002
        # Need to do some trickery here to add a dummy miso
        # as well as to deal with two differently named chip selects
        self.spi1_phy_pads = spi1_phy_pads = platform.request("FPGA_SPI1")
        spi1_pads = Record([
            ("clk",  1),
            ("cs_n", 2),
            ("mosi", 1),
            ("miso", 1),
        ])
        self.comb += [
            spi1_phy_pads.clk.eq(spi1_pads.clk),
            spi1_phy_pads.adf_cs_n.eq(spi1_pads.cs_n[0]),
            spi1_phy_pads.dac_cs_n.eq(spi1_pads.cs_n[1]),
            spi1_phy_pads.mosi.eq(spi1_pads.mosi),
            spi1_pads.miso.eq(Constant(0)),
        ]
        soc.add_spi_master(name="fpga_spi1", pads=spi1_pads, data_width=32, spi_clk_freq=1e6)

        # BRDG SPI - I2C-SPI bridge
        # bridge isn't actually fitted onboard, commenting out spimaster to not waste resources
        # self.add_spi_master(name="brdg_spi", pads=platform.request("BRDG_SPI"), data_width=32, spi_clk_freq=1e6)

        # I2C - SI5351C, Temperature sensor, EEPROM, Port Expander (Bus shared with FX3)
        self.i2c = LiteI2C(sys_clk_freq=sys_clk_freq,pads=platform.request("FPGA_I2C"),clock_domain="sys")

        # GPIO
        self.gpio = GpioTop(platform=platform,pads=platform.request("FPGA_GPIO"))

        # TST Top
        if add_ddr_modules:
            self.ddr_test_pads = platform.request("ddram",1)
        else:
            self.ddr_test_pads = None
        self.tst_top = TST_TOP_LimeSDR_USB(self.platform, add_ddr_test=add_ddr_modules, ddr_test_pads=self.ddr_test_pads)
        self.comb += self.tst_top.adf_muxout.eq(self.adf_muxout)

        if add_ddr_modules:
            soc.add_constant("DDR_MODULES_PRESENT")
            self.add_sources_ddr(platform)

            # wfmplayer
            self.wfm_load = CSRStorage(size=1, description="Load WFM data")
            self.wfm_play = CSRStorage(size=1, description="Play WFM data")
            self.wfm_smpl_width = CSRStorage(size=2, description="WFM sample width")
            self.wfm_ch_en = CSRStorage(size=2, description="WFM channel enable")

            self.wfm_ddr_pads = platform.request("ddram",0)
            self.wfm_player = WFMPlayerTop(self.platform, self.wfm_ddr_pads, pll_ref_clk, wfm_infifo_size=wfm_infifo_usedw_width)

            self.comb += [
                self.wfm_player.wfm_load.eq(self.wfm_load.storage),
                self.wfm_player.wfm_play.eq(self.wfm_play.storage),
                self.wfm_player.wfm_smpl_width.eq(self.wfm_smpl_width.storage),
                self.wfm_player.wfm_ch_en.eq(self.wfm_ch_en.storage),

                self.wfm_player.begin_test.eq(self.tst_top.test_en.storage[4]),
                self.wfm_player.insert_error.eq(self.tst_top.test_frc_err.storage[4]),
                self.tst_top.ddr2_1_pnf_per_bit.status.eq(self.wfm_player.pnf_per_bit),
                self.tst_top.test_rez_wfm_player.eq(self.wfm_player.tst_pass),
                self.tst_top.ddr2_1_tst_fail.status.eq(self.wfm_player.tst_fail),
                self.tst_top.test_cmplt_wfm_player.eq(self.wfm_player.tst_complete)
            ]




    def add_sources_ddr(self, platform):
      # ---------------------------
      ddr2_tester_files = [
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/ddr2_tester.vhd",
      ]

      for file in ddr2_tester_files:
          platform.add_source(file)

      # ---------------------------
      ddr2_ips = [
          "gateware/board_specific/limesdr_usb/ddr2_traffic_gen/ddr2_traffic_gen.qsys",
          "gateware/board_specific/limesdr_usb/ddr2/ddr2.qip",
      ]

      for ip in ddr2_ips:
          platform.add_ip(ip)

      # ---------------------------
      wfm_player_files = [
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/wfm_player_top.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/wfm_player.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/wfm_wcmd_fsm.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/wfm_rcmd_fsm.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/DDR2_ctrl_top.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/DDR2_arb.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/decompress.vhd",
          "gateware/board_specific/limesdr_usb/wfm_ram_buffer/rd_tx_fifo.vhd",
          "gateware/board_specific/limesdr_usb/altera_inst/fifo_inst.vhd",
      ]

      for file in wfm_player_files:
          platform.add_source(file)




