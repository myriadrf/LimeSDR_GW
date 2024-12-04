#!/usr/bin/env python3

from migen import *

from litex.gen import *

from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *
from litex.soc.cores.clock     import *

from litescope import LiteScopeAnalyzer

from gateware.lms7002_clk import ClkCfgRegs
from gateware.lms7002_clk import XilinxLmsMMCM


class uart_rtl(LiteXModule):
    def __init__(self, platform, clk100_domain="sys", uart_baudrate=9600,
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
        platform.add_source("./gateware/LimeDFB/uart/src/uart.vhd")

        # Reset
        self.RESET = Signal()

        self.DATA_STREAM_IN       = Signal(8)
        self.DATA_STREAM_IN_STB   = Signal()
        self.DATA_STREAM_IN_ACK   = Signal()
        self.DATA_STREAM_OUT      = Signal(8)
        self.DATA_STREAM_OUT_STB  = Signal()
        self.DATA_STREAM_OUT_ACK  = Signal()

        self.rx = Signal()
        self.tx = Signal()

        # Create params
        self.params_ios = dict()

        # Assign generics
        self.params_ios.update(
            p_BAUD_RATE=uart_baudrate,
            p_CLOCK_FREQUENCY=100000000
        )

        # Assign ports
        self.params_ios.update(
            i_CLOCK   = ClockSignal(clk100_domain),
            i_RESET   = self.RESET,
            # UART serial with internal data interface
            i_DATA_STREAM_IN      = self.DATA_STREAM_IN,
            i_DATA_STREAM_IN_STB  = self.DATA_STREAM_IN_STB,
            o_DATA_STREAM_IN_ACK  = self.DATA_STREAM_IN_ACK,
            o_DATA_STREAM_OUT     = self.DATA_STREAM_OUT,
            o_DATA_STREAM_OUT_STB = self.DATA_STREAM_OUT_STB,
            i_DATA_STREAM_OUT_ACK = self.DATA_STREAM_OUT_ACK,
            # UART serial
            i_RX=self.rx,
            o_TX=self.tx
        )

        # Create instance and assign params
        self.specials += Instance("uart", **self.params_ios)
