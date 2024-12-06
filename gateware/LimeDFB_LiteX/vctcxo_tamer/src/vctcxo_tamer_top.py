#!/usr/bin/env python3

from migen import *

from litex.gen import *

from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *
from litex.soc.cores.clock     import *

from litescope import LiteScopeAnalyzer

from gateware.LimeDFB_LiteX.lms7002.src.lms7002_pll import ClkCfgRegs
from gateware.LimeDFB_LiteX.lms7002.src.lms7002_pll import XilinxLmsMMCM


class vctcxo_tamer_top(LiteXModule):
    def __init__(self, platform, vctcxo_tamer_pads, clk100_domain="sys", vctcxo_clk_domain="clk26", baudrate=9600,
                 with_debug=False):
        # Add CSRs
        self.control = CSRStorage(fields=[
            CSRField(name="LMS1_TXNRX1", size=1, offset=12, values=[
                ("``0b0``", "Port 1 TXIQ"),
                ("``0b1``", "Port 1 RXIQ")
            ], reset=0b1),
            CSRField(name="LMS1_TXNRX2", size=1, offset=13, values=[
                ("``0b0``", "Port 2 TXIQ"),
                ("``0b1``", "Port 2 RXIQ")
            ], reset=0b0),
        ])

        # Add sources
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/edge_detector.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/gnss_led.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/handshake.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/nmea_mm_driver.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/nmea_parser_pkg.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/nmea_parser_tb.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/nmea_parser.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/nmea_str_to_bcd.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/pps_counter.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/reset_synchronizer.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/str_to_bcd.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/synchronizer.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/vctcxo_tamer_log.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/vctcxo_tamer_top.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/vctcxo_tamer/src/vctcxo_tamer.vhd")

        platform.add_source("./gateware/LimeDFB_LiteX/packages/src/string_pkg.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/uart/src/uart.vhd")



        # Reset
        self.RESET_N = Signal()

        self.UART_RX   = Signal()
        self.UART_TX   = Signal()

        # Create params
        self.params_ios = dict()

        # Assign generics
        self.params_ios.update(
            p_BAUDRATE=baudrate
        )

        # Assign ports
        self.params_ios.update(
            i_tune_ref     = vctcxo_tamer_pads.tune_ref,
            i_vctcxo_clock = ClockSignal(vctcxo_clk_domain),
            i_CLK100       = ClockSignal(clk100_domain),
            i_RESET_N      = self.RESET_N,
            # UART serial
            i_UART_RX      = self.UART_RX,
            o_UART_TX      = self.UART_TX
        )

        # Create instance and assign params
        self.specials += Instance("vctcxo_tamer_top", **self.params_ios)
