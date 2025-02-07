#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import math

from migen import *

from litex.gen import *

from litex.soc.interconnect                import stream
from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect.csr            import *

from gateware.fpgacfg  import FPGACfg
from gateware.pllcfg   import PLLCfg
from gateware.rxtx_top import RXTXTop

from gateware.LimeDFB_LiteX.lms7002.src.lms7002_top           import LMS7002Top
from gateware.LimeDFB_LiteX.general.busy_delay                import BusyDelay
from gateware.LimeDFB_LiteX.general_periph.src.general_periph import GeneralPeriphTop
from gateware.LimeDFB_LiteX.self_test.src.tst_top             import TstTop

# LimeTop ------------------------------------------------------------------------------------------

class LimeTop(LiteXModule):
    def __init__(self, platform,
        # Configuration.
        LMS_DIQ_WIDTH        = 12,
        sink_width           = 128,
        sink_clk_domain      = "sys",
        source_width         = 64,
        source_clk_domain    = "sys",
        TX_N_BUFF            = 5,
        TX_PCT_SIZE          = 4096,
        TX_IN_PCT_HDR_SIZE   = 16,

        with_rx_tx_top       = True,

        # FPGACFG.
        board_id             = 0x0011,
        major_rev            = 2,
        compile_rev          = 7,
        revision_pads        = None,

        ):

        self.sink      = AXIStreamInterface(sink_width,   clock_domain=sink_clk_domain)
        self.source    = AXIStreamInterface(source_width, clock_domain=source_clk_domain)

        self.rd_active = Signal() # From FT601
        self.wr_active = Signal() # From FT601

        if not platform.name.startswith("limesdr_mini"):
            self.ev = EventManager()
            self.ev.clk_ctrl_irq = EventSourceProcess()
            self.ev.finalize()

        # # #

        # cpu_busy(gpo) & busy_delay ---------------------------------------------------------------
        self._gpo = CSRStorage(description="GPO interface", fields=[
            CSRField("cpu_busy", size=1, offset=0, description="CPU state.", values=[
                ("``0b0``", "IDLE."),
                ("``0b1``", "BUSY."),
            ])
        ])

        self.busy_delay  = BusyDelay(platform, "sys", 25, 100) # FIXME: freq?
        self.comb       += self.busy_delay.busy_in.eq(self._gpo.fields.cpu_busy)

        # FPGA Cfg ---------------------------------------------------------------------------------
        self.fpgacfg  = FPGACfg(platform,
            board_id    = board_id,
            major_rev   = major_rev,
            compile_rev = compile_rev,
            pads        = revision_pads
        )
        self.comb += self.fpgacfg.pwr_src.eq(0)

        # PLL Cfg ----------------------------------------------------------------------------------

        if platform.name.startswith("limesdr_mini"):
            self.pllcfg = PLLCfg()
        else:
            self.pllcfg = None

        # LMS7002 Top ------------------------------------------------------------------------------
        self.lms7002_top = LMS7002Top(
            platform        = platform,
            pads            = platform.request("LMS"),
            hw_ver          = {True: Constant(0, 4), False:revision_pads.HW_VER}[revision_pads==None],
            add_csr         = True,
            fpgacfg_manager = self.fpgacfg,
            pllcfg_manager  = self.pllcfg,
            diq_width       = LMS_DIQ_WIDTH,
        )

        # Tst Top / Clock Test ---------------------------------------------------------------------

        if platform.name.startswith("limesdr_mini"):
            self.tst_top = TstTop(platform, ClockSignal("ft601"), ClockSignal("lmk"))

        # General Periph ---------------------------------------------------------------------------

        if platform.name.startswith("limesdr_mini"):
            gpio_pads     = platform.request("FPGA_GPIO")
            #egpio_pads    = platform.request("FPGA_EGPIO")

            self.general_periph = GeneralPeriphTop(platform,
                revision_pads = revision_pads,
                gpio_pads     = gpio_pads,
                gpio_len      = len(gpio_pads),
                egpio_pads    = None,
                egpio_len     = 2,
            )

            self.comb += [
                self.general_periph.led1_cpu_busy.eq(self.busy_delay.busy_out),
                self.general_periph.ep03_active.eq(self.rd_active),
                self.general_periph.ep83_active.eq(self.wr_active),
            ]

        # RXTX Top ---------------------------------------------------------------------------------

        if with_rx_tx_top:
            self.rxtx_top = RXTXTop(platform, self.fpgacfg,
                # TX parameters
                TX_IQ_WIDTH            = LMS_DIQ_WIDTH,
                TX_N_BUFF              = TX_N_BUFF,
                TX_IN_PCT_SIZE         = TX_PCT_SIZE,
                TX_IN_PCT_HDR_SIZE     = TX_IN_PCT_HDR_SIZE,
                TX_IN_PCT_DATA_W       = sink_width,
                tx_s_clk_domain        = "sys",

                # RX parameters
                RX_IQ_WIDTH            = LMS_DIQ_WIDTH,
                rx_int_clk_domain      = "sys",
                rx_m_clk_domain        = "sys",
            )

            # LMS7002 <-> RXTX Top.
            self.comb += self.rxtx_top.rx_path.smpl_cnt_en.eq(self.lms7002_top.smpl_cnt_en)

            if platform.name.startswith("limesdr_mini"):
                self.comb += [
                    # LMS7002 <-> TstTop.
                    self.lms7002_top.from_tstcfg_tx_tst_i.eq(self.tst_top.tx_tst_i),
                    self.lms7002_top.from_tstcfg_tx_tst_q.eq(self.tst_top.tx_tst_q),
                    self.lms7002_top.from_tstcfg_test_en.eq( self.tst_top.test_en),

                    # General Periph <-> RXTX Top.
                    self.general_periph.tx_txant_en.eq(self.rxtx_top.tx_path.tx_txant_en),

                    # General Periph <-> LMS7002
                    self.lms7002_top.periph_output_val_1.eq(self.general_periph.periph_output_val_1),
                ]

            if platform.name in ["limesdr_mini_v2"]:
                # LMS7002 <-> PLLCFG
                self.comb += self.lms7002_top.smpl_cmp_length.eq(self.pllcfg.auto_phcfg_smpls)

            # LMS7002 -> RX Path -> Sink Pipeline.
            self.rx_pipeline = stream.Pipeline(
                self.lms7002_top.source,
                self.rxtx_top.rx_path,
                self.source,
            )

            # Source -> TX Path -> LMS7002 Pipeline.
            self.tx_pipeline = stream.Pipeline(
                self.sink,
                self.rxtx_top.tx_path,
                self.lms7002_top.sink,
            )

        # RF Switches ------------------------------------------------------------------------------

        if platform.name.startswith("limesdr_mini"):
            rfsw_pads  = platform.request("RFSW")
            tx_lb_pads = platform.request("TX_LB")

            self.gpio = CSRStorage(16, reset=0b0001000101000100) # fpgacfg @23
            self.comb += [
                # RF Switch.
                rfsw_pads.RX_V1.eq(self.gpio.storage[8]),
                rfsw_pads.RX_V2.eq(self.gpio.storage[9]),
                rfsw_pads.TX_V1.eq(self.gpio.storage[12]),
                rfsw_pads.TX_V2.eq(self.gpio.storage[13]),

                # TX
                tx_lb_pads.AT.eq(  self.gpio.storage[1]),
                tx_lb_pads.SH.eq(  self.gpio.storage[2]),
            ]
        else:
            rfsw_pads = platform.request("rf_switches")
            self.rfsw_control = xtrx_rfsw(platform, rfsw_pads)

