#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import math

from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect                import stream

from gateware.common import *

# RX Path Top --------------------------------------------------------------------------------------

class TXPathTop(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
        # TX parameters
        IQ_WIDTH          = 12,
        PCT_MAX_SIZE      = 4096,
        PCT_HDR_SIZE      = 16,
        BUFF_COUNT        = 4,
        FIFO_DATA_W       = 128,
        rx_clk_domain     = "lms_rx",
        m_clk_domain      = "lms_tx",
        s_clk_domain      = "lms_tx"
        ):

        assert fpgacfg_manager is not None

        self.platform          = platform

        self.source            = AXIStreamInterface(64,          clock_domain=m_clk_domain)
        self.sink              = AXIStreamInterface(FIFO_DATA_W, clock_domain=s_clk_domain)

        self.rx_sample_nr      = Signal(64)
        self.pct_loss_flg      = Signal()
        self.pct_loss_flg_clr  = Signal()

        self.tx_txant_en       = Signal()

        self.ext_reset_n       = Signal(reset=1)

        # # #

        # Signals.
        s_reset_n        = Signal()
        m_reset_n        = Signal()

        # Synchro
        rx_sample_nr     = Signal(64)
        ch_en            = Signal(2)
        smpl_width       = Signal()
        synch_dis        = Signal()

        pct_loss_flg_clr = Signal()

        p2d_wr_tvalid    = Signal(BUFF_COUNT)
        p2d_wr_tdata     = Signal(128)
        p2d_wr_tready    = Signal(BUFF_COUNT)
        p2d_wr_tlast     = Signal(BUFF_COUNT)

        p2d_rd_tvalid    = Signal(BUFF_COUNT)
        p2d_rd_tdata     = Signal(128)
        p2d_rd_tready    = Signal(BUFF_COUNT)
        p2d_rd_tlast     = Signal(BUFF_COUNT)
        p2d_rd_resetn    = Signal(BUFF_COUNT)

        curr_buf_index   = Signal(math.ceil(math.log2(BUFF_COUNT)))

        p2d_wr_buf_empty = Signal(BUFF_COUNT)

        data_pad_tvalid  = Signal()
        data_pad_tdata   = Signal(128)
        data_pad_tready  = Signal()
        data_pad_tlast   = Signal()

        # AXI Slave 64 -> 128 (must uses s_axis_domain)
        conv_64_to_128      = ResetInserter()(ClockDomainsRenamer(s_clk_domain)(stream.Converter(64, 128)))
        self.conv_64_to_128 = conv_64_to_128

        # FIFO before unpacker
        fifo_smpl_buff      = ResetInserter()(ClockDomainsRenamer(m_clk_domain)(stream.SyncFIFO([("data", 128)], 16)))
        self.fifo_smpl_buff = fifo_smpl_buff

        unpack_bypass       = Signal()

        # Clocks ----------------------------------------------------------------------------------
        # Sample NR FIFO (must be async with sink in RX_CLK, source iqsample, areset_n with iqpacket_areset_n)
        if platform.name in ["limesdr_mini_v1"]:
            smpl_nr_fifo      = ResetInserter()(ClockDomainsRenamer("lms_tx")(stream.SyncFIFO([("data", 64)], 128)))
            self.smpl_nr_fifo = smpl_nr_fifo
            self.comb += smpl_nr_fifo.reset.eq(s_reset_n),
        else:
            self.cd_smpl_nr_fifo  = ClockDomain()
            smpl_nr_fifo          = stream.ClockDomainCrossing([("data", 64)],
                cd_from = "smpl_nr_fifo",
                cd_to   = s_clk_domain,
                depth   = 128,
            )
            self.smpl_nr_fifo     = smpl_nr_fifo
            self.comb += [
                self.cd_smpl_nr_fifo.clk.eq(ClockSignal(rx_clk_domain)),
                self.cd_smpl_nr_fifo.rst.eq(ResetSignal(rx_clk_domain) | (~(s_reset_n & self.ext_reset_n))),
            ]

        self.comb += [
            conv_64_to_128.reset.eq(     ~(s_reset_n & self.ext_reset_n)),
            conv_64_to_128.sink.last.eq( 0), # FIXME: something else?

            conv_64_to_128.sink.data.eq( self.sink.data),
            conv_64_to_128.sink.valid.eq(self.sink.valid),
            self.sink.ready.eq(          conv_64_to_128.sink.ready),

            # smpl_nr_fifo
            smpl_nr_fifo.sink.data.eq(   self.rx_sample_nr),
            smpl_nr_fifo.sink.valid.eq(  smpl_nr_fifo.sink.ready),
            rx_sample_nr.eq(             smpl_nr_fifo.source.data),
            smpl_nr_fifo.source.ready.eq(smpl_nr_fifo.source.valid),
        ]

        self.pct2data_buf_wr = Instance("PCT2DATA_BUF_WR",
            # Parameters.
            p_G_BUFF_COUNT    = BUFF_COUNT,

            # Clk/Reset.
            i_AXIS_ACLK       = ClockSignal(s_clk_domain),    # s_axis_domain
            i_S_AXIS_ARESET_N = s_reset_n,                    # s_axis_domain.a_reset_n

            # AXI Stream Slave
            i_S_AXIS_TVALID   = conv_64_to_128.source.valid,
            i_S_AXIS_TDATA    = conv_64_to_128.source.data,
            o_S_AXIS_TREADY   = conv_64_to_128.source.ready,
            i_S_AXIS_TLAST    = conv_64_to_128.source.last,

            # AXI Stream Master
            i_M_AXIS_ARESET_N = s_reset_n,                    # s_axis_domain.a_reset_n
            o_M_AXIS_TVALID   = p2d_wr_tvalid,
            o_M_AXIS_TDATA    = p2d_wr_tdata,
            i_M_AXIS_TREADY   = p2d_wr_tready,
            o_M_AXIS_TLAST    = p2d_wr_tlast,

            i_BUF_EMPTY       = p2d_wr_buf_empty,
            i_RESET_N         = {True: s_reset_n, False: self.ext_reset_n}[platform.name.startswith("limesdr_mini")],
        )

        cases = {}
        for i in range(BUFF_COUNT):
            if platform.name in ["limesdr_mini_v1"]:
                # FIXME: write: s_axis_domain, read: m_axis_domain. Must check PACKET_FIFO mean
                smpl_fifo = ResetInserter()(ClockDomainsRenamer(m_clk_domain)(stream.SyncFIFO([("data", 128)], 256)))
                self.comb += [
                    smpl_fifo.reset.eq(       s_reset_n & p2d_rd_resetn[i]),
                ]
            else:
                # Sample FIFO ClockDomain
                self.cd_smpl_fifo = ClockDomain()
                self.comb += [
                    self.cd_smpl_fifo.clk.eq(ClockSignal(s_clk_domain)),
                    self.cd_smpl_fifo.rst.eq(ResetSignal(s_clk_domain) | (~(s_reset_n & p2d_rd_resetn[i]))),
                ]

                self.smpl_fifo = smpl_fifo = stream.ClockDomainCrossing([("data", 128)],
                    cd_from = "smpl_fifo",
                    cd_to   = m_clk_domain,
                    depth   = 256,
                )

            self.comb += [
                # pct2data_buf_wr -> FIFO
                smpl_fifo.sink.valid.eq(  p2d_wr_tvalid[i]),
                p2d_wr_tready[i].eq(      smpl_fifo.sink.ready),
                smpl_fifo.sink.last.eq(   p2d_wr_tlast[i]),
                smpl_fifo.sink.data.eq(   p2d_wr_tdata),

                # FIFO -> pct2data_buf_rd
                p2d_rd_tvalid[i].eq(      smpl_fifo.source.valid),
                p2d_rd_tlast[i].eq(       smpl_fifo.source.last),
                smpl_fifo.source.ready.eq(p2d_rd_tready[i]),
            ]

            cases[i] = p2d_rd_tdata.eq(smpl_fifo.source.data)

        self.comb += Case(curr_buf_index, cases)

        self.pct2data_buf_rd = Instance("PCT2DATA_BUF_RD",
            # Parameters.
            p_G_BUFF_COUNT       = BUFF_COUNT,

            # Clk/Reset.
            i_AXIS_ACLK          = ClockSignal(m_clk_domain), #m_axis_domain

            # AXI Stream Slave.
            i_S_AXIS_ARESET_N    = m_reset_n,                 # m_axis_domain.a_reset_n (iqsample)
            o_S_AXIS_BUF_RESET_N = p2d_rd_resetn,
            i_S_AXIS_TVALID      = p2d_rd_tvalid,
            i_S_AXIS_TDATA       = p2d_rd_tdata,
            o_S_AXIS_TREADY      = p2d_rd_tready,
            i_S_AXIS_TLAST       = p2d_rd_tlast,

            # AXI Stream Master.
            i_M_AXIS_ARESET_N    = m_reset_n,               # m_axis_domain.a_reset_n (iqsample)
            o_M_AXIS_TVALID      = data_pad_tvalid,
            o_M_AXIS_TDATA       = data_pad_tdata,
            i_M_AXIS_TREADY      = data_pad_tready,
            o_M_AXIS_TLAST       = data_pad_tlast,

            o_CURR_BUF_INDEX     = curr_buf_index,

            i_RESET_N            = m_reset_n,                 # Unconnected for XTRX
            i_SYNCH_DIS          = synch_dis,                 # Disable timestamp sync
            i_SAMPLE_NR          = rx_sample_nr,
            o_PCT_LOSS_FLG       = self.pct_loss_flg,         # Goes high when a packet is dropped due to outdated timestamp, stays high until PCT_LOSS_FLG_CLR is set
            i_PCT_LOSS_FLG_CLR   = pct_loss_flg_clr,          # Clears PCT_LOSS_FLG
        )

        # Pad 12 bit samples to 16 bit samples, bypass logic if no padding is needed
        self.sample_padder = Instance("sample_padder",
            # Clk/Reset.
            i_CLK           = ClockSignal(m_clk_domain), # m_axis_domain
            i_RESET_N       = m_reset_n,                 # Unconnected for XTRX

            # AXI Stream Slave.
            i_S_AXIS_TVALID = data_pad_tvalid,
            i_S_AXIS_TDATA  = data_pad_tdata,
            o_S_AXIS_TREADY = data_pad_tready,
            i_S_AXIS_TLAST  = data_pad_tlast,

            # AXI Stream Master.
            o_M_AXIS_TDATA  = fifo_smpl_buff.sink.data,
            o_M_AXIS_TVALID = fifo_smpl_buff.sink.valid,
            o_M_AXIS_TREADY = fifo_smpl_buff.sink.ready,
            o_M_AXIS_TLAST  = fifo_smpl_buff.sink.last,

            # Control.
            i_BYPASS        = unpack_bypass,
        )

        self.sample_unpack = Instance("SAMPLE_UNPACK",
            # Clk/Reset.
            i_RESET_N       = m_reset_n,                 # Unconnected for XTRX
            i_AXIS_ACLK     = ClockSignal(m_clk_domain), # m_axis_domain
            i_AXIS_ARESET_N = m_reset_n,                 # m_axis_domain.a_reset_n

            # AXI Stream Master
            i_S_AXIS_TDATA  = fifo_smpl_buff.source.data,
            o_S_AXIS_TREADY = fifo_smpl_buff.source.ready,
            i_S_AXIS_TVALID = fifo_smpl_buff.source.valid,
            i_S_AXIS_TLAST  = fifo_smpl_buff.source.last,

            # AXI Stream Master
            o_M_AXIS_TDATA  = self.source.data,
            i_M_AXIS_TREADY = self.source.ready,
            o_M_AXIS_TVALID = self.source.valid,

            # Mode Settings.
            i_CH_EN         = ch_en,
        )

        self.specials += [
            MultiReg(fpgacfg_manager.tx_en,      s_reset_n,        odomain=s_clk_domain),
            MultiReg(fpgacfg_manager.tx_en,      m_reset_n,        odomain=m_clk_domain),
            MultiReg(fpgacfg_manager.ch_en,      ch_en,            odomain=m_clk_domain),
            MultiReg(fpgacfg_manager.smpl_width, smpl_width,       odomain=m_clk_domain),
            MultiReg(fpgacfg_manager.synch_dis,  synch_dis,        odomain=m_clk_domain),
            MultiReg(self.pct_loss_flg_clr,      pct_loss_flg_clr, odomain=m_clk_domain),
        ]

        self.comb += [
            self.source.last.eq(0),
            If(smpl_width == 0b00,
                unpack_bypass.eq(1),
            ).Else(
                unpack_bypass.eq(0),
            ),
        ]

        self.pct2data_buf_wr_conv = add_vhd2v_converter(self.platform,
            instance = self.pct2data_buf_wr,
            files    = ["gateware/LimeDFB_LiteX/tx_path_top/src/pct2data_buf_wr.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.pct2data_buf_wr)

        self.pct2data_buf_rd_conv = add_vhd2v_converter(self.platform,
            instance = self.pct2data_buf_rd,
            files    = ["gateware/LimeDFB_LiteX/tx_path_top/src/pct2data_buf_rd.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.pct2data_buf_rd)

        self.sample_padder_conv = add_vhd2v_converter(self.platform,
            instance = self.sample_padder,
            files    = ["gateware/LimeDFB/tx_path_top/src/sample_padder.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.sample_padder)

        self.sample_unpack_conv = add_vhd2v_converter(self.platform,
            instance = self.sample_unpack,
            files    = ["gateware/LimeDFB_LiteX/tx_path_top/src/sample_unpack.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.sample_unpack)
