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
from migen.genlib.cdc import MultiReg
from litex.gen import *

from litex.soc.interconnect                  import stream
from litex.soc.interconnect.axi.axi_stream   import AXIStreamInterface
from litex.soc.interconnect.csr              import *
from litex.soc.interconnect.csr_eventmanager import *

from gateware.fpgacfg   import FPGACfg
from gateware.rxtx_top  import RXTXTop

from gateware.LimeDFB.lms7002.src.lms7002_top           import LMS7002Top

from gateware.examples.fft.LimeFFT                      import LimeFFT

from gateware.common import *
import warnings

# LimeTop ------------------------------------------------------------------------------------------

class LimeTop(LiteXModule):
    def __init__(self, soc, platform, vendor, family,
        # Configuration.
        double_channels_mode = False,
        one_chnl             = False,
        LMS_DIQ_WIDTH        = 12,
        sink_width           = 128,
        sink_clk_domain      = "sys",
        source_width         = 64,
        source_clk_domain    = "sys",
        rx_sys_clk_domain    = "sys",
        rx_fixed_packet_size = False,
        TX_N_BUFF            = 5, # size of tx packet buffer packet queue
        TX_MAX_PCT_SIZE      = 8192, # Total tx packet buffer capacity in bytes
        tx_buffer_size       = 512, #TX buffer acts as CDC, so a minimum of 512 (4 cycles of 128bit) is required to instantiate the async FIFO
        TX_WITHTXIQ_MUX        = False,

        with_lms7002         = True,
        # These clocks are only used if with_lms7002 is False
        phy_tx_source_clk    = "sys",
        phy_tx_sink_width    = 128,
        phy_rx_sink_clk      = "sys",
        phy_rx_sink_width    = 128,
        with_rx_tx_top       = True,
        fft_pts              = 512,     #Changing FFT points requires FFT src rebuild, use rebuild_fft_rtl=True
        rebuild_fft_rtl      = False,
        with_fft             = False,

        # FPGACFG.
        board_id             = 0x0011,
        major_rev            = 2,
        compile_rev          = 7,
        revision_pads        = None,

        with_event_manager   = True,
        with_clk_cfg_irq     = True,
        soc_has_timesource   = False,


        ):

        self.sink      = AXIStreamInterface(sink_width,   clock_domain=sink_clk_domain)
        self.source    = AXIStreamInterface(source_width, clock_domain=source_clk_domain)

        self.platform              = platform

        # vendor must be "lattice", "altera", or "xilinx"
        if vendor not in ["lattice", "altera", "xilinx"]:
            raise ValueError(f"Unsupported vendor: {vendor}")
        self.vendor = vendor

        self.pps       = Signal()



        if with_event_manager:
            self.ev = EventManager()
            self.ev.clk_ctrl_irq = EventSourceProcess()
            self.ev.finalize()

        # # #

        # FPGA Cfg ---------------------------------------------------------------------------------
        self.fpgacfg  = FPGACfg(
            board_id           = board_id,
            major_rev          = major_rev,
            compile_rev        = compile_rev,
            pads               = revision_pads,
            soc_has_timesource = soc_has_timesource,
        )

        # LMS7002 Top ------------------------------------------------------------------------------
        if with_lms7002:
            soc.add_constant("WITH_LMS7002")
            self.lms7002_top = lms7002_top = LMS7002Top(
                platform        = platform,
                vendor          = vendor,
                family          = family,
                pads            = platform.request("LMS"),
                fpgacfg_manager = self.fpgacfg,
                diq_width       = LMS_DIQ_WIDTH,
                one_chnl        = one_chnl,
                with_txiq_mux   = TX_WITHTXIQ_MUX,
            )
        else:
            # Create ports to interface with rxtx top, if lms7002 is not used.
            # Assuming a different phy module is used instead.
            self.phy_tx_source = AXIStreamInterface(phy_tx_sink_width, clock_domain=phy_tx_source_clk)
            self.phy_rx_sink   = AXIStreamInterface(phy_rx_sink_width, clock_domain=phy_rx_sink_clk)


        # RXTX Top ---------------------------------------------------------------------------------

        if with_rx_tx_top:

            self.rxtx_top = RXTXTop(platform, self.fpgacfg,
                # TX parameters
                TX_N_BUFF              = TX_N_BUFF,
                TX_IN_MAX_PCT_SIZE     = TX_MAX_PCT_SIZE,
                TX_IN_PCT_DATA_W       = sink_width,
                tx_s_clk_domain        = sink_clk_domain,
                tx_m_clk_domain        = "lms_tx" if with_lms7002 else phy_tx_source_clk,
                tx_buffer_size         = tx_buffer_size,
                tx_4ch_mode            = double_channels_mode,

                # RX parameters
                rx_sink_width          = 64 if with_lms7002 else phy_rx_sink_width,
                RX_OUT_PCT_DATA_W      = source_width,
                # "sys" uses less resources, presumably due to less clock domain crossings
                # but lms_rx is necessary if timesource syncrhonisation is needed
                # TODO: Investigate WHY exactly "sys" uses less resources, because it shouldn't
                rx_int_clk_domain      = "lms_rx" if with_lms7002 else phy_rx_sink_clk if soc_has_timesource else rx_sys_clk_domain,
                rx_s_clk_domain        = "lms_rx" if with_lms7002 else phy_rx_sink_clk,
                rx_m_clk_domain        = source_clk_domain,
                rx_fixed_packet_size   = rx_fixed_packet_size,

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
            if with_lms7002:
                # LMS7002 <-> RXTX Top.
                self.comb += self.rxtx_top.rx_path.smpl_cnt_en.eq(self.lms7002_top.smpl_cnt_en)

            # FFT example --------------------------------------------------------------------------------------
            if rebuild_fft_rtl:
                from amaranth.back import verilog
                from gateware.examples.fft.fixedpointfft import FixedPointFFT
                fft = FixedPointFFT(bitwidth=12, pts=fft_pts, verbose=True)
                verilog_code = verilog.convert(fft, name="fft", strip_internal_attrs=False,
                                               ports=[fft.in_i, fft.in_q, fft.out_real, fft.out_imag, fft.strobe_in,
                                                      fft.strobe_out, fft.start, fft.done, fft.wf_start, fft.wf_strobe,
                                                      fft.wf_real, fft.wf_imag, fft.wr_state, fft.wr_state_valid])
                # write verilog to file
                with open("./gateware/examples/fft/fft.v", "w") as f:
                    f.write(verilog_code)


            if with_fft:
                # Define Reset signal
                fft_reset_n = Signal()
                # Connect newly defined reset signal to main rx path reset trough MultiReg
                self.specials += MultiReg(self.fpgacfg.rx_en, fft_reset_n, odomain=self.lms7002_top.source.clock_domain)

                # Instantiate FFT module
                self.fft_example = LimeFFT(platform=platform,
                                           sink_clk_domain=self.lms7002_top.source.clock_domain,
                                           source_clk_domain=self.lms7002_top.source.clock_domain,
                                           fft_pts = fft_pts)

                # Connect reset signal to FFT module
                self.comb += self.fft_example.reset.eq(~fft_reset_n)
            # TODO: maybe distribute pipeline.add commands to be near appropriate module instantiations?
            # ----- RX PIPELINE
            self.rx_pipeline = stream.Pipeline()
            # Add lms7002 to the pipeline.
            if with_lms7002:
                self.rx_pipeline.add(self.lms7002_top)
            else:
                self.rx_pipeline.add(self.phy_rx_sink)
            # Add fft example to the pipeline
            if with_fft:
                self.rx_pipeline.add(self.fft_example)
            # Add rxtx top to the pipeline.
            if with_rx_tx_top:
                self.rx_pipeline.add(self.rxtx_top.rx_path)
            else:
                warnings.warn("LimeTop: RXTX Top not used, RX pipeline will likely not work!")
            # Add endpoint to the pipeline.
            self.rx_pipeline.add(self.source)

            # ----- TX PIPELINE
            self.tx_pipeline = stream.Pipeline()
            self.tx_pipeline.add(self.sink)
            if with_rx_tx_top:
                self.tx_pipeline.add(self.rxtx_top.tx_path)
            else:
                warnings.warn("LimeTop: RXTX Top not used, TX pipeline will likely not work!")
            if with_lms7002:
                self.tx_pipeline.add(self.lms7002_top)
            else:
                self.tx_pipeline.add(self.phy_tx_source)


        # Interrupt --------------------------------------------------------------------------------
        if with_lms7002:
            if with_clk_cfg_irq:
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
