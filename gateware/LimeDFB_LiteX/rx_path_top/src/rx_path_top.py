#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *


from litescope import LiteScopeAnalyzer


class rx_path_top(LiteXModule):
    def __init__(self, platform, s_axis_iqsmpls_buffer_words=16, m_axis_iqpacket_buffer_words=512, int_clk_domain="sys",
                 m_clk_domain="sys", s_clk_domain="sys", with_debug=False):
        # Add CSRs
        self.ch_en = CSRStorage(2, reset=3,
            description="01 - Channel A enabled, 10 - Channel B enabled, 11 - Channels A and B enabled"
        )
        self.smpl_width = CSRStorage(2, reset=2,
            description="10 - 12bit, 01 - Reserved, 00 - 16bit"
        )
        self.pkt_size = CSRStorage(16, reset=253,
            description="Packet Size in bytes, "
        )

        # Add sources
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/rx_path_top.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/pack_56_to_64.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/pack_48_to_64.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/iq_stream_combiner.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/data2packets_fsm.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/bit_pack.vhd")
        platform.add_source("./gateware/LimeDFB_LiteX/rx_path_top/src/axis_nto1_converter.vhd")

        platform.add_source("./gateware/LimeDFB_LiteX/fifo_axis/src/fifo_axis_wrap.vhd")
        platform.add_ip("./gateware/LimeDFB_LiteX/axis/src/axis_dwidth_converter_128_to_64/axis_dwidth_converter_128_to_64.xci")
        platform.add_source("./gateware/LimeDFB_LiteX/axis/src/axis_pkg.vhd")


        # create misc signals
        self.RESET_N        = Signal()
        self.SMPL_NR_EN     = Signal()
        self.SMPL_NR_CLR    = Signal()
        self.SMPL_NR_LD     = Signal()
        self.SMPL_NR_IN     = Signal(64)
        self.SMPL_NR_OUT    = Signal(64)
        self.TXFLAGS_PCT_LOSS       = Signal()
        self.TXFLAGS_PCT_LOSS_CLR   = Signal()


        # Create streams
        s_axis_datawidth = 64
        s_axis_layout = [("data", max(1, s_axis_datawidth))]
        s_axis_layout += [("keep", max(1, s_axis_datawidth//8))]
        # adding reset along with data, assuming resets are not global
        s_axis_layout += [("areset_n", 1)]

        self.s_axis_iqsmpls = AXIStreamInterface(s_axis_datawidth, layout=s_axis_layout, clock_domain=s_clk_domain)

        m_axis_datawidth = 64
        m_axis_layout = [("data", max(1, m_axis_datawidth))]
        m_axis_layout += [("keep", max(1, m_axis_datawidth//8))]
        # adding reset along with data, assuming resets are not global
        m_axis_layout += [("areset_n", 1)]
        self.m_axis_iqpacket = AXIStreamInterface(m_axis_datawidth, layout=m_axis_layout, clock_domain=m_clk_domain)

        # Create params
        self.params_ios = dict()

        # Assign generics
        self.params_ios.update(
            p_G_S_AXIS_IQSMPLS_BUFFER_WORDS=s_axis_iqsmpls_buffer_words,
            p_G_M_AXIS_IQPACKET_BUFFER_WORDS=m_axis_iqpacket_buffer_words
        )

        # Assign ports
        self.params_ios.update(
            # DIQ1
            i_CLK       = ClockSignal(int_clk_domain),
            i_RESET_N   = self.RESET_N,
            # axis_s
            i_S_AXIS_IQSMPLS_ARESETN    = self.s_axis_iqsmpls.areset_n,
            i_S_AXIS_IQSMPLS_ACLK       = ClockSignal(s_clk_domain),
            i_S_AXIS_IQSMPLS_TVALID     = self.s_axis_iqsmpls.valid,
            o_S_AXIS_IQSMPLS_TREADY     = self.s_axis_iqsmpls.ready,
            i_S_AXIS_IQSMPLS_TDATA      = self.s_axis_iqsmpls.data,
            i_S_AXIS_IQSMPLS_TKEEP      = self.s_axis_iqsmpls.keep,
            i_S_AXIS_IQSMPLS_TLAST      = self.s_axis_iqsmpls.last,
            # axis_m
            i_M_AXIS_IQPACKET_ARESETN   = self.m_axis_iqpacket.areset_n,
            i_M_AXIS_IQPACKET_ACLK      = ClockSignal(m_clk_domain),
            o_M_AXIS_IQPACKET_TVALID    = self.m_axis_iqpacket.valid,
            i_M_AXIS_IQPACKET_TREADY    = self.m_axis_iqpacket.ready,
            o_M_AXIS_IQPACKET_TDATA     = self.m_axis_iqpacket.data,
            o_M_AXIS_IQPACKET_TKEEP     = self.m_axis_iqpacket.keep,
            o_M_AXIS_IQPACKET_TLAST     = self.m_axis_iqpacket.last,
            # CFG
            i_CFG_CH_EN                 = self.ch_en.storage,
            i_CFG_SMPL_WIDTH            = self.smpl_width.storage,
            i_CFG_PKT_SIZE              = self.pkt_size.storage,
            # SMPL_NR
            i_SMPL_NR_EN            = self.SMPL_NR_EN,
            i_SMPL_NR_CLR           = self.SMPL_NR_CLR,
            i_SMPL_NR_LD            = self.SMPL_NR_LD,
            i_SMPL_NR_IN            = self.SMPL_NR_IN,
            o_SMPL_NR_OUT           = self.SMPL_NR_OUT,
            # interface cfg
            i_TXFLAGS_PCT_LOSS      = self.TXFLAGS_PCT_LOSS,
            i_TXFLAGS_PCT_LOSS_CLR  = self.TXFLAGS_PCT_LOSS_CLR
        )

        # Create instance and assign params
        self.specials += Instance("rx_path_top", **self.params_ios)

        # LiteScope example.
        # ------------------
        # Setup LiteScope Analyzer to capture some of the AXI-Lite MMAP signals.
        if with_debug:
            analyzer_signals = [
                self.s_axis_iqsmpls.areset_n,
                self.s_axis_iqsmpls.valid,
                self.s_axis_iqsmpls.ready,
                self.s_axis_iqsmpls.data,
                self.s_axis_iqsmpls.keep,
                self.s_axis_iqsmpls.last,
                self.m_axis_iqpacket.areset_n,
                self.m_axis_iqpacket.valid,
                self.m_axis_iqpacket.ready,
                self.m_axis_iqpacket.data,
                self.m_axis_iqpacket.keep,
                self.m_axis_iqpacket.last,

            ]

            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 512,
                clock_domain = m_clk_domain,
                register     = True,
                csr_csv      = "lime_top_rx_path_analyzer.csv"
            )
