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

# Stream Start Controller --------------------------------------------------------------------------


class StreamStartController(LiteXModule):
    # Stream start modes.
    START_IMMEDIATE       = 0  # en_req=1 starts stream immediately.
    START_ON_PPS         = 1  # en_req=1 arms stream; PPS rising edge starts it.
    START_ON_PPS_VALID   = 2  # en_req=1 arms stream; PPS rising edge starts it only when pps_valid=1.
    START_ON_EXT_TRIGGER = 3  # en_req=1 arms stream; external trigger rising edge starts it.

    def __init__(self, clock_domain="sys", synchronize_inputs=False):

        # CSR start-mode configuration.
        self.rx_delay_mode = CSRStorage(size=2, description="RX stream start mode", fields=[
            CSRField("rx_del_sel", size=2, offset=0, description="RX stream start mode", reset=0, values=[
                ("``0b00``", "Start immediately when rx_en_req is asserted."),
                ("``0b01``", "Start on PPS rising edge after rx_en_req is asserted."),
                ("``0b10``", "Start on PPS rising edge with pps_valid=1 after rx_en_req is asserted."),
                ("``0b11``", "Start on external trigger rising edge after rx_en_req is asserted."),
            ])
        ])

        self.tx_delay_mode = CSRStorage(size=2, description="TX stream start mode", fields=[
            CSRField("tx_del_sel", size=2, offset=0, description="TX stream start mode", reset=0, values=[
                ("``0b00``", "Start immediately when tx_en_req is asserted."),
                ("``0b01``", "Start on PPS rising edge after tx_en_req is asserted."),
                ("``0b10``", "Start on PPS rising edge with pps_valid=1 after tx_en_req is asserted."),
                ("``0b11``", "Start on external trigger rising edge after tx_en_req is asserted."),
            ])
        ])

        self.tx_sync_mode = CSRStorage(size=1, description="TX/RX stream start synchronization mode", fields=[
            CSRField("tx_sync_with_rx", size=1, offset=0, reset=1, values=[
                ("``0b0``", "TX starts independently using tx_en_req and tx_delay_mode."),
                ("``0b1``", "TX follows RX. tx_en_req and tx_delay_mode are ignored."),
            ])
        ])

        # Inputs from CSR / external logic.
        self.rx_en_req = Signal()
        self.tx_en_req = Signal()

        self.pps         = Signal()
        self.pps_valid   = Signal()
        self.ext_trigger = Signal()

        # Effective stream enables.
        self.rx_en = Signal()
        self.tx_en = Signal()

        # # #

        sync_domain = getattr(self.sync, clock_domain)

        rx_en_req       = Signal()
        tx_en_req       = Signal()
        rx_start_mode   = Signal(2)
        tx_start_mode   = Signal(2)
        tx_sync_with_rx = Signal()
        pps             = Signal()
        pps_valid       = Signal()
        ext_trigger     = Signal()

        if synchronize_inputs:
            self.specials += [
                MultiReg(self.rx_en_req, rx_en_req, odomain=clock_domain),
                MultiReg(self.tx_en_req, tx_en_req, odomain=clock_domain),

                MultiReg(self.rx_delay_mode.fields.rx_del_sel, rx_start_mode, odomain=clock_domain),
                MultiReg(self.tx_delay_mode.fields.tx_del_sel, tx_start_mode, odomain=clock_domain),
                MultiReg(self.tx_sync_mode.fields.tx_sync_with_rx, tx_sync_with_rx, odomain=clock_domain),

                MultiReg(self.pps,         pps,         odomain=clock_domain),
                MultiReg(self.pps_valid,   pps_valid,   odomain=clock_domain),
                MultiReg(self.ext_trigger, ext_trigger, odomain=clock_domain),
            ]
        else:
            self.comb += [
                rx_en_req.eq(self.rx_en_req),
                tx_en_req.eq(self.tx_en_req),

                rx_start_mode.eq(self.rx_delay_mode.fields.rx_del_sel),
                tx_start_mode.eq(self.tx_delay_mode.fields.tx_del_sel),
                tx_sync_with_rx.eq(self.tx_sync_mode.fields.tx_sync_with_rx),

                pps.eq(self.pps),
                pps_valid.eq(self.pps_valid),
                ext_trigger.eq(self.ext_trigger),
            ]

        # PPS rising-edge detector.
        pps_d      = Signal()
        pps_rising = Signal()

        sync_domain += [
            pps_d.eq(pps),
        ]

        self.comb += [
            pps_rising.eq(pps & ~pps_d),
        ]

        # External trigger rising-edge detector.
        ext_trigger_d      = Signal()
        ext_trigger_rising = Signal()

        sync_domain += [
            ext_trigger_d.eq(ext_trigger),
        ]

        self.comb += [
            ext_trigger_rising.eq(ext_trigger & ~ext_trigger_d),
        ]

        # Start conditions.
        rx_start_now = Signal()
        tx_start_now = Signal()

        self.comb += [
            rx_start_now.eq(
                (rx_start_mode == self.START_IMMEDIATE) |
                ((rx_start_mode == self.START_ON_PPS) & pps_rising) |
                ((rx_start_mode == self.START_ON_PPS_VALID) & pps_rising & pps_valid) |
                ((rx_start_mode == self.START_ON_EXT_TRIGGER) & ext_trigger_rising)
            ),

            tx_start_now.eq(
                (tx_start_mode == self.START_IMMEDIATE) |
                ((tx_start_mode == self.START_ON_PPS) & pps_rising) |
                ((tx_start_mode == self.START_ON_PPS_VALID) & pps_rising & pps_valid) |
                ((tx_start_mode == self.START_ON_EXT_TRIGGER) & ext_trigger_rising)
            ),
        ]

        # RX enable latch.
        #
        # - Cleared immediately when rx_en_req is low.
        # - Set when rx_en_req is high and selected RX start condition occurs.
        # - Holds while rx_en_req remains high.
        sync_domain += [
            If(rx_en_req == 0,
                self.rx_en.eq(0),
            ).Elif(rx_start_now,
                self.rx_en.eq(1),
            )
        ]

        # TX enable latch.
        #
        # tx_sync_with_rx = 0:
        #   TX starts independently:
        #       tx_en_req=1 arms TX.
        #       tx_delay_mode selects the TX start condition.
        #
        # tx_sync_with_rx = 1:
        #   TX follows RX:
        #       tx_en_req is ignored.
        #       tx_delay_mode is ignored.
        #       TX clears when rx_en_req=0.
        #       TX sets on rx_start_now, so TX and RX go high in the same clock cycle.
        sync_domain += [
            If(tx_sync_with_rx == 1,
                If(rx_en_req == 0,
                    self.tx_en.eq(0),
                ).Elif(self.rx_en | rx_start_now,
                    self.tx_en.eq(1),
                )

            ).Else(
                If(tx_en_req == 0,
                    self.tx_en.eq(0),
                ).Elif(tx_start_now,
                    self.tx_en.eq(1),
                )
            )
        ]


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
        self.pps_valid = Signal()
        self.ext_stream_trigger = Signal()



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
        )

        # Stream Start Controller
        self.stream_start_controller = StreamStartController(clock_domain="sys", synchronize_inputs=True)
        self.comb += [self.stream_start_controller.rx_en_req.eq(self.fpgacfg.rx_en),
                      self.stream_start_controller.tx_en_req.eq(self.fpgacfg.tx_en),
                      self.stream_start_controller.pps.eq(self.pps),
                      self.stream_start_controller.pps_valid.eq(self.pps_valid),
                      self.stream_start_controller.ext_trigger.eq(self.ext_stream_trigger),
                      ]

        # LMS7002 Top ------------------------------------------------------------------------------
        if with_lms7002:
            soc.add_constant("WITH_LMS7002")
            self.lms7002_top = lms7002_top = LMS7002Top(
                platform        = platform,
                vendor          = vendor,
                family          = family,
                pads            = platform.request("LMS"),
                fpgacfg_manager = self.fpgacfg,
                rx_stream_en=self.stream_start_controller.rx_en,
                tx_stream_en    = self.stream_start_controller.tx_en,
                pllcfg_manager  = None,
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

            self.rxtx_top = RXTXTop(platform, self.fpgacfg, self.stream_start_controller.rx_en, self.stream_start_controller.tx_en,
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
                self.specials += MultiReg(self.stream_start_controller.rx_en, fft_reset_n, odomain=self.lms7002_top.source.clock_domain)

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
                rx_en_reg.eq(self.stream_start_controller.rx_en),
                tx_en_reg.eq(self.stream_start_controller.tx_en),
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
                If((self.stream_start_controller.rx_en == 1) & (rx_en_reg == 0),[
                    self.rx_time_min_sec.fields.sec.eq    (self.time_seconds),
                    self.rx_time_min_sec.fields.min.eq    (self.time_minutes),
                    self.rx_time_mon_day_hrs.fields.hrs.eq(self.time_hours  ),
                    self.rx_time_mon_day_hrs.fields.day.eq(self.time_day    ),
                    self.rx_time_mon_day_hrs.fields.mon.eq(self.time_month  ),
                    self.rx_time_yrs.fields.yrs.eq        (self.time_year   ),
                ]).Elif(self.stream_start_controller.rx_en == 0,[
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
                If((self.stream_start_controller.tx_en == 1) & (tx_en_reg == 0),[
                    self.tx_time_min_sec.fields.sec.eq    (self.time_seconds),
                    self.tx_time_min_sec.fields.min.eq    (self.time_minutes),
                    self.tx_time_mon_day_hrs.fields.hrs.eq(self.time_hours  ),
                    self.tx_time_mon_day_hrs.fields.day.eq(self.time_day    ),
                    self.tx_time_mon_day_hrs.fields.mon.eq(self.time_month  ),
                    self.tx_time_yrs.fields.yrs.eq        (self.time_year   ),
                ]).Elif(self.stream_start_controller.tx_en == 0,[
                    self.tx_time_min_sec.fields.sec.eq    (0),
                    self.tx_time_min_sec.fields.min.eq    (0),
                    self.tx_time_mon_day_hrs.fields.hrs.eq(0),
                    self.tx_time_mon_day_hrs.fields.day.eq(0),
                    self.tx_time_mon_day_hrs.fields.mon.eq(0),
                    self.tx_time_yrs.fields.yrs.eq        (0),
                ]),
            ]
            ####
