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

from litex.soc.interconnect                  import stream
from litex.soc.interconnect.axi.axi_stream   import AXIStreamInterface
from litex.soc.interconnect.csr              import *
from litex.soc.interconnect.csr_eventmanager import *

from gateware.fpgacfg   import FPGACfg
from gateware.pllcfg    import PLLCfg
from gateware.rxtx_top  import RXTXTop
from gateware.xtrx_rfsw import xtrx_rfsw

from gateware.LimeDFB_LiteX.lms7002.src.lms7002_top           import LMS7002Top
from gateware.LimeDFB_LiteX.general.busy_delay                import BusyDelay
from gateware.LimeDFB_LiteX.general_periph.src.general_periph import GeneralPeriphTop
from gateware.LimeDFB_LiteX.self_test.src.tst_top             import TstTop

# LimeTop ------------------------------------------------------------------------------------------

class LimeTop(LiteXModule):
    def __init__(self, soc, platform,
        # Configuration.
        LMS_DIQ_WIDTH        = 12,
        sink_width           = 128,
        sink_clk_domain      = "sys",
        source_width         = 64,
        source_clk_domain    = "sys",
        TX_N_BUFF            = 5,
        TX_PCT_SIZE          = 4096,
        TX_IN_PCT_HDR_SIZE   = 16,
        tx_buffer_size       = 0,

        with_lms7002         = True,
        with_rx_tx_top       = True,
        with_fft             = False,

        # FPGACFG.
        board_id             = 0x0011,
        major_rev            = 2,
        compile_rev          = 7,
        revision_pads        = None,

        soc_has_timesource   = False,

        ):

        self.sink      = AXIStreamInterface(sink_width,   clock_domain=sink_clk_domain)
        self.source    = AXIStreamInterface(source_width, clock_domain=source_clk_domain)

        self.rd_active = Signal() # From FT601
        self.wr_active = Signal() # From FT601

        self.pps       = Signal()



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
        if with_lms7002:
            soc.add_constant("WITH_LMS7002")
            if revision_pads is None:
                hw_ver = Constant(0, 4)
            else:
                hw_ver = revision_pads.HW_VER
            self.lms7002_top = lms7002_top = LMS7002Top(
                platform        = platform,
                pads            = platform.request("LMS"),
                hw_ver          = hw_ver,
                add_csr         = True,
                fpgacfg_manager = self.fpgacfg,
                pllcfg_manager  = self.pllcfg,
                diq_width       = LMS_DIQ_WIDTH,
                with_max10_pll  = True,
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
            # TODO: This should probably be moved to top level
            if platform.name.startswith("limesdr_mini_v1"):
                self.comb +=[
                    self.general_periph.led1_r_in.eq(self.lms7002_top.lms7002_clk.pll_locked),
                ]
            else:
                self.comb +=[
                    self.general_periph.led1_r_in.eq(~self.busy_delay.busy_out),
                ]
            self.comb += [
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
                tx_buffer_size         = tx_buffer_size,

                # RX parameters
                RX_IQ_WIDTH            = LMS_DIQ_WIDTH,
                # rx_int_clk_domain      = "sys", # uses the default lms_rx
                rx_m_clk_domain        = "sys",

                # Misc
                soc_has_timesource     = soc_has_timesource,
            )

            if soc_has_timesource:
                # TODO: move notes from here and other similar locations to top of limetop to act as a checklist
                #       when using for new board.
                self.rx_delay_mode = CSRStorage(size=2, description="RX enable signal delay mode", fields=[
                    CSRField("rx_del_sel", size=2, offset=0, description="RX enable signal delay mode",reset=0, values=[
                        ("``0b0``", "No Delay."),
                        ("``0b1``", "Delay until PPS"), # NOTE: tx_en_delay_signal must be assigned at top level!
                        ("``0b2``", "Delay until PPS and Valid"), # NOTE: tx_en_delay_signal must be assigned at top level!
                    ])
                ])
                self.tx_delay_mode = CSRStorage(size=2, description="TX enable signal delay mode", fields=[
                    CSRField("tx_del_sel", size=2, offset=0, description="TX enable signal delay mode",reset=0, values=[
                        ("``0b0``", "No Delay."),
                        ("``0b1``", "Delay until PPS"), # NOTE: rx_en_delay_signal must be assigned at top level!
                        ("``0b2``", "Delay until PPS and Valid"), # NOTE: rx_en_delay_signal must be assigned at top level!
                    ])
                ])

                self.comb += [
                        self.fpgacfg.tx_en_delay_mode.eq(self.tx_delay_mode.fields.tx_del_sel),
                        self.fpgacfg.rx_en_delay_mode.eq(self.rx_delay_mode.fields.rx_del_sel),
                ]

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

        # FFT --------------------------------------------------------------------------------------

        if with_fft:
            # define Reset signal and adds a MultiReg
            fft_reset_n = Signal()
            self.specials += MultiReg(self.fpgacfg.rx_en, fft_reset_n, odomain=self.lms7002_top.source.clock_domain)

            # Declare FFT AXI Stream interfaces.
            self.fft_s_axis = AXIStreamInterface(data_width=64, clock_domain=self.lms7002_top.source.clock_domain)
            self.fft_m_axis = AXIStreamInterface(data_width=64, clock_domain=self.lms7002_top.source.clock_domain)

            # Instantiate the FFT wrapper.
            self.specials += Instance("fft_wrap",
                i_CLK           = ClockSignal(self.lms7002_top.source.clock_domain),
                i_RESET_N       = fft_reset_n,
                i_S_AXIS_TVALID = self.fft_s_axis.valid,
                i_S_AXIS_TDATA  = self.fft_s_axis.data,
                o_S_AXIS_TREADY = self.fft_s_axis.ready,
                i_S_AXIS_TLAST  = self.fft_s_axis.last,
                i_S_AXIS_TKEEP  = self.fft_s_axis.keep,
                o_M_AXIS_TDATA  = self.fft_m_axis.data,
                o_M_AXIS_TVALID = self.fft_m_axis.valid,
                i_M_AXIS_TREADY = self.fft_m_axis.ready,
                o_M_AXIS_TLAST  = self.fft_m_axis.last,
                o_M_AXIS_TKEEP  = self.fft_m_axis.keep,
            )

            # Add FFT sources to the platform.
            platform.add_source("./gateware/examples/fft/fft.v")
            platform.add_source("./gateware/examples/fft/fft_wrap.vhd")

        if with_fft:
            # LMS7002 -> FFT -> RX Path -> PCIe DMA Pipeline.
            # Connect the LMS7002 master interface to the FFT wrapper slave interface
            self.comb += self.lms7002_top.source.connect(self.fft_s_axis)
            # Connect the FFT wrapper master interface to the RX path slave interface
            self.comb += self.fft_m_axis.connect(self.rxtx_top.rx_path.sink)
        elif with_lms7002 and with_rx_tx_top: # Disabled for golden LimeSDR Mini v1
            # LMS7002 -> RX Path -> PCIe DMA Pipeline.
            self.rx_pipeline = stream.Pipeline(
                self.lms7002_top.source,
                self.rxtx_top.rx_path.sink,
            )

        # VCTCXO -----------------------------------------------------------------------------------

        if not platform.name.startswith("limesdr_mini"):
            vctcxo_pads = platform.request("vctcxo")
            self.comb  += vctcxo_pads.sel.eq(self.fpgacfg.ext_clk)
            self.comb  += vctcxo_pads.en.eq(self.fpgacfg.tcxo_en)

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
            rfsw_pads         = platform.request("rf_switches")
            self.rfsw_control = xtrx_rfsw(platform, rfsw_pads)
            #self.comb += rfsw_pads.tx.eq(1)

        # Interrupt --------------------------------------------------------------------------------

        if not platform.name.startswith("limesdr_mini"):
            self.comb += self.ev.clk_ctrl_irq.trigger.eq((lms7002_top.lms7002_clk.CLK_CTRL.PHCFG_START.re & lms7002_top.lms7002_clk.CLK_CTRL.PHCFG_START.storage == 1)
                | (lms7002_top.lms7002_clk.CLK_CTRL.PLLCFG_START.re & lms7002_top.lms7002_clk.CLK_CTRL.PLLCFG_START.storage == 1)
                | (lms7002_top.lms7002_clk.CLK_CTRL.PLLRST_START.re & lms7002_top.lms7002_clk.CLK_CTRL.PLLRST_START.storage == 1) )

        if soc_has_timesource:
            # TODO: maybe move this to a separate module
            # Time Sync
            # NOTE: Must be connected to something in top level!
            self.time_seconds = Signal(6)
            self.time_minutes = Signal(6)
            self.time_hours   = Signal(5)
            self.time_day     = Signal(5)
            self.time_month   = Signal(4)
            self.time_year    = Signal(12)
            ## Edge detection signals
            rx_en_reg = Signal(reset=0)
            tx_en_reg = Signal(reset=0)
            self.sync.sys +=[
                rx_en_reg.eq(self.fpgacfg.rx_en),
                tx_en_reg.eq(self.fpgacfg.tx_en),
            ]
            ####
            # RX stream start time registers
            self.rx_time_min_sec = CSRStatus(size= 16, description="Time in minutes and seconds, when RX stream started", fields=[
                CSRField("sec", size=6, offset=0, description="RX stream start time, seconds"),
                CSRField("min", size=6, offset=6, description="RX stream start time, minutes")
            ])
            self.rx_time_mon_day_hrs = CSRStatus(size= 16, description="Time in months, days and hours, when RX stream started", fields=[
                CSRField("hrs", size=5, offset=0, description="RX stream start time, hours"),
                CSRField("day", size=5, offset=5, description="RX stream start time, days"),
                CSRField("mon", size=4, offset=10, description="RX stream start time, months"),
            ])
            self.rx_time_yrs = CSRStatus(size= 16, description="Time in years, when RX stream started", fields=[
                CSRField("yrs", size=12, offset=0, description="RX stream start time, years")
            ])
            self.sync.sys +=[
                ## RX time store
                If((self.fpgacfg.rx_en == 1) & (rx_en_reg == 0),[
                    self.rx_time_min_sec.fields.sec.eq    (self.time_seconds),
                    self.rx_time_min_sec.fields.min.eq    (self.time_minutes),
                    self.rx_time_mon_day_hrs.fields.hrs.eq(self.time_hours  ),
                    self.rx_time_mon_day_hrs.fields.day.eq(self.time_day    ),
                    self.rx_time_mon_day_hrs.fields.mon.eq(self.time_month  ),
                    self.rx_time_yrs.fields.yrs.eq        (self.time_year   ),
                ]).Elif(self.fpgacfg.rx_en == 0,[
                    self.rx_time_min_sec.fields.sec.eq    (0),
                    self.rx_time_min_sec.fields.min.eq    (0),
                    self.rx_time_mon_day_hrs.fields.hrs.eq(0),
                    self.rx_time_mon_day_hrs.fields.day.eq(0),
                    self.rx_time_mon_day_hrs.fields.mon.eq(0),
                    self.rx_time_yrs.fields.yrs.eq        (0),
                ]),
            ]
            ####
            # TX stream start time registers
            self.tx_time_min_sec = CSRStatus(size= 16, description="Time in minutes and seconds, when TX stream started", fields=[
                CSRField("sec", size=6, offset=0, description="TX stream start time, seconds"),
                CSRField("min", size=6, offset=6, description="TX stream start time, minutes")
            ])
            self.tx_time_mon_day_hrs = CSRStatus(size= 16, description="Time in months, days and hours, when TX stream started", fields=[
                CSRField("hrs", size=5, offset=0, description="TX stream start time, hours"),
                CSRField("day", size=5, offset=5, description="TX stream start time, days"),
                CSRField("mon", size=4, offset=10, description="TX stream start time, months"),
            ])
            self.tx_time_yrs = CSRStatus(size= 16, description="Time in years, when TX stream started", fields=[
                CSRField("yrs", size=12, offset=0, description="TX stream start time, years")
            ])
            self.sync.sys +=[
                ## TX time store
                If((self.fpgacfg.tx_en == 1) & (tx_en_reg == 0),[
                    self.tx_time_min_sec.fields.sec.eq    (self.time_seconds),
                    self.tx_time_min_sec.fields.min.eq    (self.time_minutes),
                    self.tx_time_mon_day_hrs.fields.hrs.eq(self.time_hours  ),
                    self.tx_time_mon_day_hrs.fields.day.eq(self.time_day    ),
                    self.tx_time_mon_day_hrs.fields.mon.eq(self.time_month  ),
                    self.tx_time_yrs.fields.yrs.eq        (self.time_year   ),
                ]).Elif(self.fpgacfg.tx_en == 0,[
                    self.tx_time_min_sec.fields.sec.eq    (0),
                    self.tx_time_min_sec.fields.min.eq    (0),
                    self.tx_time_mon_day_hrs.fields.hrs.eq(0),
                    self.tx_time_mon_day_hrs.fields.day.eq(0),
                    self.tx_time_mon_day_hrs.fields.mon.eq(0),
                    self.tx_time_yrs.fields.yrs.eq        (0),
                ]),
            ]
            ####