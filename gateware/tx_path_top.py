#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *


from litescope import LiteScopeAnalyzer


class tx_path_top(LiteXModule):
    def __init__(self, platform, buff_count=4, rx_clk_domain="sys", m_clk_domain="sys", s_clk_domain="sys"):
        # Add CSRs
        self.ch_en = CSRStorage(2, reset=3,
            description="01 - Channel A enabled, 10 - Channel B enabled, 11 - Channels A and B enabled"
        )
        self.smpl_width = CSRStorage(2, reset=2,
            description="10 - 12bit, 01 - Reserved, 00 - 16bit"
        )
        self.sync_dis = CSRStorage(1, reset=1,
            description="1 - TX timestamp synchronization disabled, 0 - TX timestamp synchronization enabled"
        )

        # Add sources
        platform.add_source("./gateware/LimeDFB/tx_path_top/src/pct2data_buf_rd.vhd")
        platform.add_source("./gateware/LimeDFB/tx_path_top/src/sample_unpack.vhd")
        platform.add_source("./gateware/LimeDFB/tx_path_top/src/tx_top_pkg.vhd")
        platform.add_source("./gateware/LimeDFB/tx_path_top/src/pct2data_buf_wr.vhd")
        platform.add_source("./gateware/LimeDFB/tx_path_top/src/tx_path_top.vhd")
        platform.add_source("./gateware/LimeDFB/tx_path_top/src/sample_padder.vhd")

        platform.add_ip("./gateware/LimeDFB/axis/src/axis_dwidth_converter_64_to_128/axis_dwidth_converter_64_to_128.xci")
        platform.add_source("./gateware/LimeDFB/axis/src/axis_pkg.vhd")


        # create misc signals
        self.RESET_N            = Signal()
        self.RX_SAMPLE_NR       = Signal(64)
        self.PCT_LOSS_FLG       = Signal()
        self.PCT_LOSS_FLG_CLR   = Signal()



        # Create streams
        s_axis_datawidth = 64
        s_axis_layout = [("data", max(1, s_axis_datawidth))]
        # adding reset along with data, assuming resets are not global
        s_axis_layout += [("areset_n", 1)]

        self.s_axis_iqpacket = AXIStreamInterface(s_axis_datawidth, layout=s_axis_layout, clock_domain=s_clk_domain)

        m_axis_datawidth = 64
        m_axis_layout = [("data", max(1, m_axis_datawidth))]
        # adding reset along with data, assuming resets are not global
        m_axis_layout += [("areset_n", 1)]
        self.m_axis_iqsample = AXIStreamInterface(m_axis_datawidth, layout=m_axis_layout, clock_domain=m_clk_domain)

        # Create params
        self.params_ios = dict()

        # Assign generics
        self.params_ios.update(
            p_G_BUFF_COUNT=buff_count,
        )

        # Assign ports
        self.params_ios.update(
            i_RESET_N   = self.RESET_N,
            # axis_s
            i_S_AXIS_IQPACKET_ARESET_N   = self.s_axis_iqpacket.areset_n,
            i_S_AXIS_IQPACKET_ACLK       = ClockSignal(s_clk_domain),
            i_S_AXIS_IQPACKET_TVALID     = self.s_axis_iqpacket.valid,
            o_S_AXIS_IQPACKET_TREADY     = self.s_axis_iqpacket.ready,
            i_S_AXIS_IQPACKET_TDATA      = self.s_axis_iqpacket.data,
            i_S_AXIS_IQPACKET_TLAST      = self.s_axis_iqpacket.last,
            # axis_m
            i_M_AXIS_IQSAMPLE_ARESET_N   = self.m_axis_iqsample.areset_n,
            i_M_AXIS_IQSAMPLE_ACLK       = ClockSignal(m_clk_domain),
            o_M_AXIS_IQSAMPLE_TVALID     = self.m_axis_iqsample.valid,
            i_M_AXIS_IQSAMPLE_TREADY     = self.m_axis_iqsample.ready,
            o_M_AXIS_IQSAMPLE_TDATA      = self.m_axis_iqsample.data,
            o_M_AXIS_IQSAMPLE_TLAST      = self.m_axis_iqsample.last,
            # RX sample NR
            i_RX_SAMPLE_NR               = self.RX_SAMPLE_NR,
            i_RX_CLK                     = ClockSignal(rx_clk_domain),
            # PCT
            i_PCT_SYNC_DIS               = self.sync_dis.storage,
            o_PCT_LOSS_FLG               = self.PCT_LOSS_FLG,
            i_PCT_LOSS_FLG_CLR           = self.PCT_LOSS_FLG_CLR,
            # CFG
            i_CFG_CH_EN                  = self.ch_en.storage,
            i_CFG_SAMPLE_WIDTH           = self.smpl_width.storage,
        )

        # Create instance and assign params
        self.specials += Instance("tx_path_top", **self.params_ios)
