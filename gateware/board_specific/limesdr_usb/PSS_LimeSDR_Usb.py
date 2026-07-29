from gateware.IoOverrideTop import IoOverrideTop
from gateware.LedCtrl import Heartbeat, AdfDacLedStatus
from gateware.LimeDFB.general.busy_delay import BusyDelay
from gateware.board_specific.limesdr_usb.TST_TOP_LimeSDR_USB import TST_TOP_LimeSDR_USB
from gateware.board_specific.limesdr_usb.wfm_ram_buffer.wfm_player_top import WFMPlayerTop
from litei2c import LiteI2C
from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *

# PSS (Peripheral Support Subsystem) -----------------------------------------------------------------

class PSS_LimeSDR_Usb(LiteXModule):
    def __init__(self, soc, platform, sys_clk_freq, pll_ref_clk, revision_pads, fx3_busy, add_ddr_modules=True, wfm_infifo_usedw_width=11):
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
        self.gpio_io = IoOverrideTop(platform, name="gpio", inout_pads=platform.request("FPGA_GPIO"))

        # Plain default-value inputs, driven post-hoc from the target once LimeTop exists.
        self.tx_txant_en     = Signal()
        self.tx_pct_loss_flg = Signal()
        self.tx_pll_lock     = Signal()
        self.rx_pll_lock     = Signal()

        # FPGA_GPIO[0:7] default (non-overridden) functions: all outputs.
        #   [0]=TX_TXANT_EN, [1]=RX_PLL_LOCKED, [2]=TX_PLL_LOCKED, [3]=TX_PCT_LOSS_FLG, [4:7]=Reserved (Low).
        self.comb += [
            self.gpio_io.dir.eq(0),
            self.gpio_io.out_val.eq(Cat(
                self.tx_txant_en, self.rx_pll_lock, self.tx_pll_lock, self.tx_pct_loss_flg,
                0, 0, 0, 0,
            )),
        ]

        # LEDs (FPGA LED1/LED2/FX3 LED) ------------------------------------------------------------
        self.heartbeat = Heartbeat()

        self.adf_dac_led = AdfDacLedStatus(
            adf_muxout = self.adf_muxout,
            dac_cs_n   = spi1_phy_pads.dac_cs_n,
            adf_cs_n   = spi1_phy_pads.adf_cs_n,
        )

        # LED1 (Clock & PLL Status) default value: green = heartbeat, red = ON (blinking
        # anti-phase to green) when either PLL is unlocked.
        led1_g_default = Signal()
        led1_r_default = Signal()
        self.comb += [
            led1_g_default.eq(self.heartbeat.beat),
            led1_r_default.eq(~self.heartbeat.beat & ~(self.tx_pll_lock & self.rx_pll_lock)),
        ]

        # LED2 (ADF & DAC Status) default value.
        led2_g_default = self.adf_dac_led.default_g
        led2_r_default = self.adf_dac_led.default_r

        # FX3 LED (USB & System Activity) default value: red = busy, green = idle, forced off
        # unless 3 <= HW_VER < 15.
        self.cpu_busy = CSRStorage(description="GPO interface", fields=[
            CSRField("cpu_busy", size=1, offset=0, description="CPU state.", values=[
                ("``0b0``", "IDLE."),
                ("``0b1``", "BUSY."),
            ])
        ])
        self.busy_delay = BusyDelay(platform, "sys")
        self.comb += self.busy_delay.busy_in.eq(fx3_busy | self.cpu_busy.fields.cpu_busy)

        hw_ver_ok = Signal()
        self.comb += hw_ver_ok.eq((revision_pads.HW_VER >= 3) & (revision_pads.HW_VER < 15))

        led3_g_default = Signal()
        led3_r_default = Signal()
        self.comb += [
            led3_g_default.eq(~self.busy_delay.busy_out & hw_ver_ok),
            led3_r_default.eq( self.busy_delay.busy_out & hw_ver_ok),
        ]

        # Fan (default = temperature-sensor passthrough, override via IoOverrideTop's CSRs).
        fan_default = platform.request("LM75_OS")

        self.led_fan_io = IoOverrideTop(platform, name="led_fan",
            out_pads = Cat(
                platform.request("FPGA_LED1_G"), platform.request("FPGA_LED1_R"),
                platform.request("FPGA_LED2_G"), platform.request("FPGA_LED2_R"),
                platform.request("FX3_LED_G"),   platform.request("FX3_LED_R"),
                platform.request("FAN_CTRL"),
            ),
        )
        self.comb += self.led_fan_io.out_default.eq(Cat(
            led1_g_default, led1_r_default,
            led2_g_default, led2_r_default,
            led3_g_default, led3_r_default,
            fan_default,
        ))

        # RF loopback
        lb_pads = platform.request("LB")
        lb_bus  = Signal(8)
        self.lb_io = IoOverrideTop(platform, name="lb", out_pads=lb_bus)
        self.comb += [
            lb_pads.TX1_L.eq(~lb_bus[0]),
            lb_pads.TX1_H.eq( lb_bus[0]),
            lb_pads.TX1_AT.eq(lb_bus[1]),
            lb_pads.TX1_SH.eq(lb_bus[2]),
            lb_pads.TX2_L.eq(~lb_bus[4]),
            lb_pads.TX2_H.eq( lb_bus[4]),
            lb_pads.TX2_AT.eq(lb_bus[5]),
            lb_pads.TX2_SH.eq(lb_bus[6]),
        ]
        self.comb += self.lb_io.out_default.eq(0x00)

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




