#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

"""
TX Path Top Module Structure:

    AXI Stream Input (FIFO_DATA_W)
    |
    v
    Stream Converter (FIFO_DATA_W -> 128)
    |
    v
    Input Buffer (CDC s_clk -> m_clk)
    |
    v
    PCT2DATA Buffer Writer
    |
    v
    Multiple Buffer FIFOs (BUFF_COUNT)
    |
    v
    PCT2DATA Buffer Reader <---- Sample Number FIFO (CDC rx_clk -> m_clk)
    |
    v
    Sample Padder (12->16 bit)
    |
    v
    Sample Unpacker
    |
    v
    AXI Stream Output (64)

"""

import math

from litex.soc.interconnect.stream import ClockDomainCrossing
from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect                import stream

from gateware.common import *

# TX Path Top --------------------------------------------------------------------------------------



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
        s_clk_domain      = "lms_tx",
        input_buff_size   = 512
        ):
        #Input buffer acts as CDC, so a minimum of 512 (4 cycles of 128bit) is required to instantiate the async FIFO
        assert input_buff_size >= 512, "TXPathTop input_buff_size must be greater than or equal to 512 (4 cycles of 128bit)"
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
        smpl_width       = Signal(2)
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

        self.p2d_wr_buf_empty = p2d_wr_buf_empty = Signal(BUFF_COUNT)

        data_pad_tvalid  = Signal()
        data_pad_tdata   = Signal(128)
        data_pad_tready  = Signal()
        data_pad_tlast   = Signal()

        # AXI Slave FIFO_DATA_W -> 128 (must uses s_axis_domain)
        conv_64_to_128      = ResetInserter()(ClockDomainsRenamer(s_clk_domain)(stream.Converter(FIFO_DATA_W, 128)))
        self.conv_64_to_128 = conv_64_to_128

        # Input data buffer (128 bit)
        input_buff = ClockDomainCrossing(
            layout=[("data", 128)],
            cd_from  = s_clk_domain,
            cd_to    = m_clk_domain,
            depth    = int(input_buff_size/128),
            buffered = True)
        self.input_buff = input_buff

        # FIFO before unpacker
        fifo_smpl_buff      = ResetInserter()(ClockDomainsRenamer(m_clk_domain)(stream.SyncFIFO([("data", 128)], 16)))
        self.fifo_smpl_buff = fifo_smpl_buff

        unpack_bypass       = Signal()

        # LiteScope probes
        self.smpl_width      = smpl_width
        self.unpack_bypass   = unpack_bypass
        self.p2d_rd_tready   = p2d_rd_tready
        self.p2d_rd_tlast    = p2d_rd_tlast
        self.p2d_rd_tvalid   = p2d_rd_tvalid
        self.p2d_wr_tvalid   = p2d_wr_tvalid
        self.p2d_wr_tready   = p2d_wr_tready
        self.p2d_wr_tlast    = p2d_wr_tlast
        self.conn_buf        = Signal()
        self.data_pad_tready = data_pad_tready
        self.data_pad_tlast  = data_pad_tlast
        self.data_pad_tvalid = data_pad_tvalid
        self.data_pad_tdata  = data_pad_tdata
        self.curr_buf_index  = curr_buf_index

        self.s_reset_n = s_reset_n
        self.m_reset_n = m_reset_n

        # Clocks ----------------------------------------------------------------------------------
        # Sample NR FIFO (must be async with sink in RX_CLK, source iqsample, areset_n with iqpacket_areset_n)
        if platform.name in ["something"]:
            smpl_nr_fifo      = ResetInserter()(ClockDomainsRenamer("lms_tx")(stream.SyncFIFO([("data", 64)], 128)))
            self.smpl_nr_fifo = smpl_nr_fifo
            self.comb += smpl_nr_fifo.reset.eq(~s_reset_n),
        else:
            #TODO: check if reset is needed here
            self.cd_smpl_nr_fifo  = ClockDomain()
            smpl_nr_fifo          = stream.ClockDomainCrossing([("data", 64)],
                cd_from = "smpl_nr_fifo",
                cd_to   = m_clk_domain,
                depth   = 8,
            )
            self.smpl_nr_fifo     = smpl_nr_fifo
            self.comb += [
                self.cd_smpl_nr_fifo.clk.eq(ClockSignal(rx_clk_domain)),
                self.cd_smpl_nr_fifo.rst.eq( (~(s_reset_n & self.ext_reset_n))),
            ]

        p2d_wr_sink_ready = Signal()

        self.comb += [
            conv_64_to_128.reset.eq(     ~s_reset_n),
            conv_64_to_128.sink.last.eq( 0),

            conv_64_to_128.sink.data.eq( self.sink.data),
            conv_64_to_128.sink.valid.eq(self.sink.valid),
            self.sink.ready.eq(          conv_64_to_128.sink.ready),

            # smpl_nr_fifo
            smpl_nr_fifo.sink.data.eq(   self.rx_sample_nr),
            smpl_nr_fifo.sink.valid.eq(  smpl_nr_fifo.sink.ready),
            rx_sample_nr.eq(             smpl_nr_fifo.source.data),
            smpl_nr_fifo.source.ready.eq(smpl_nr_fifo.source.valid),

            # input_buff
            input_buff.sink.data.eq(     conv_64_to_128.source.data),
            input_buff.sink.last.eq(     conv_64_to_128.source.last),
            input_buff.sink.valid.eq(    conv_64_to_128.source.valid),
            # Async fifo used by ClockDomainCrossing does not have a reset
            # Passing reset as a ready signal to clear out the fifo is a workaround
            conv_64_to_128.source.ready.eq(input_buff.sink.ready | ~s_reset_n),
            input_buff.source.ready.eq(p2d_wr_sink_ready | ~s_reset_n),
        ]

        self.pct2data_buf_wr = Instance("PCT2DATA_BUF_WR",
            # Parameters.
            p_G_BUFF_COUNT    = BUFF_COUNT,

            # Clk/Reset.
            i_AXIS_ACLK       = ClockSignal(m_clk_domain),    # m_axis_domain
            i_S_AXIS_ARESET_N = m_reset_n,                    # m_axis_domain.a_reset_n

            # AXI Stream Slave
            i_S_AXIS_TVALID   = input_buff.source.valid,
            i_S_AXIS_TDATA    = input_buff.source.data,
            o_S_AXIS_TREADY   = p2d_wr_sink_ready,
            i_S_AXIS_TLAST    = input_buff.source.last,

            # AXI Stream Master
            i_M_AXIS_ARESET_N = m_reset_n,                    # m_axis_domain.a_reset_n
            o_M_AXIS_TVALID   = p2d_wr_tvalid,
            o_M_AXIS_TDATA    = p2d_wr_tdata,
            i_M_AXIS_TREADY   = p2d_wr_tready,
            o_M_AXIS_TLAST    = p2d_wr_tlast,

            i_BUF_EMPTY       = p2d_wr_buf_empty,
            i_RESET_N         = self.ext_reset_n,
        )

        cases = {}

        # local variables to avoid mismatch between
        # data_width and tkeep_width
        packet_mode = True
        data_width  = 128
        tkeep_width = int(data_width/8)
        fifo_depth  = 256

        force_convert = platform.vhd2v_force
        # May be problematic if we need to use fifo somewhere else
        self.fifo_src_conv =  VHD2VConverter(platform,
                              work_package   = "work",
                              force_convert  = True,#force_convert,
                              flatten_source = False,
                              add_instance   = False,
                              files    = ["gateware/LimeDFB/axis_fifo/src/axis_fifo.vhd",
                                          "gateware/LimeDFB/axis_fifo/src/wptr_handler.vhd",
                                          "gateware/LimeDFB/axis_fifo/src/rptr_handler.vhd",
                                          "gateware/LimeDFB/axis_fifo/src/ram_mem_wrapper.vhd",
                                          # VHD2VConverter is not able to convert this file.
                                          # OK because it is not used in "Generic" implementation
                                          #"gateware/LimeDFB/axis_fifo/src/xilinx_simple_dual_port_2_clock_ram.vhd",
                                          "gateware/LimeDFB/cdc/src/cdc_sync_bit.vhd",
                                          "gateware/LimeDFB/cdc/src/cdc_sync_bus.vhd",
                                          ],
                              params= {
                                  'p_G_VENDOR'      : "GENERIC", # Only "GENERIC" supported for now
                                  'p_G_PACKET_MODE' : "true" if packet_mode else "false",
                                  'p_g_DATA_WIDTH'  : data_width,
                                  'p_g_FIFO_DEPTH'  : fifo_depth,
                              },
                              top_entity='axis_fifo'
                              )

        # -1 to fix off by one error
        for i in range(BUFF_COUNT):
            usedw_width = math.ceil(math.log2(fifo_depth))
            self.wr_usedw = Signal(usedw_width+1)
            self.rd_usedw = Signal(usedw_width + 1)
            self.p2d_rd_tkeep = Signal(tkeep_width)
            sample_data_out = Signal(data_width)

            self.packet_buf = Instance("axis_fifo",
            # Parameters
            #   vhd2v converter seems to have trouble with boolean and string parameters
            #   but default values are good here
            # p_G_PACKET_MODE         = True,
            # p_G_VENDOR              = "GENERIC",
            #### These are provided to vhd2vconverter
            #p_G_FIFO_DEPTH          = fifo_depth,
            #p_G_DATA_WIDTH          = data_width        ,
            # s_axis
            i_s_axis_aresetn        = (m_reset_n & self.ext_reset_n & p2d_rd_resetn[i]),
            i_s_axis_aclk           = ClockSignal(m_clk_domain),
            i_s_axis_tvalid         = p2d_wr_tvalid[i],
            o_s_axis_tready         = p2d_wr_tready[i],
            i_s_axis_tdata          = p2d_wr_tdata,
            i_s_axis_tkeep          = Replicate(1,tkeep_width),
            i_s_axis_tlast          = p2d_wr_tlast[i],
            # m_axis
            i_m_axis_aresetn  = (m_reset_n & self.ext_reset_n & p2d_rd_resetn[i]),
            i_m_axis_aclk     = ClockSignal(m_clk_domain),
            o_m_axis_tvalid   = p2d_rd_tvalid[i],
            i_m_axis_tready   = p2d_rd_tready[i],
            o_m_axis_tdata    = sample_data_out,
            o_m_axis_tkeep    = self.p2d_rd_tkeep, # open, unused
            o_m_axis_tlast    = p2d_rd_tlast[i],
            # usedw
            o_rdusedw         = self.rd_usedw, # open, unused
            o_wrusedw         = self.wr_usedw
            )

            self.comb +=[
                p2d_wr_buf_empty[i].eq(self.wr_usedw == 0)
            ]

            cases[i] = p2d_rd_tdata.eq(sample_data_out)

        # Mux data input for p2d_rd
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

            i_RESET_N            = self.ext_reset_n,          # Unconnected for XTRX
            i_SYNCH_DIS          = synch_dis,                 # Disable timestamp sync
            i_SAMPLE_NR          = rx_sample_nr,
            o_PCT_LOSS_FLG       = self.pct_loss_flg,         # Goes high when a packet is dropped due to outdated timestamp, stays high until PCT_LOSS_FLG_CLR is set
            i_PCT_LOSS_FLG_CLR   = pct_loss_flg_clr,          # Clears PCT_LOSS_FLG
            o_conn_buf_o         = self.conn_buf,
        )

        # Pad 12 bit samples to 16 bit samples, bypass logic if no padding is needed
        self.sample_padder = Instance("sample_padder",
            # Clk/Reset.
            i_CLK           = ClockSignal(m_clk_domain), # m_axis_domain
            i_RESET_N       = self.ext_reset_n,          # Unconnected for XTRX

            # AXI Stream Slave.
            i_S_AXIS_TVALID = data_pad_tvalid,
            i_S_AXIS_TDATA  = data_pad_tdata,
            o_S_AXIS_TREADY = data_pad_tready,
            i_S_AXIS_TLAST  = data_pad_tlast,

            # AXI Stream Master.
            o_M_AXIS_TDATA  = fifo_smpl_buff.sink.data,
            o_M_AXIS_TVALID = fifo_smpl_buff.sink.valid,
            i_M_AXIS_TREADY = fifo_smpl_buff.sink.ready,
            o_M_AXIS_TLAST  = fifo_smpl_buff.sink.last,

            # Control.
            i_BYPASS        = unpack_bypass,
        )

        self.sample_unpack = Instance("SAMPLE_UNPACK",
            # Clk/Reset.
            i_RESET_N       = self.ext_reset_n,          # Unconnected for XTRX
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

        if platform.name.startswith("limesdr_mini"):
            self.specials += [
                MultiReg(fpgacfg_manager.rx_en, s_reset_n, odomain=s_clk_domain),
                MultiReg(fpgacfg_manager.rx_en, m_reset_n, odomain=m_clk_domain),
            ]
        else:
            self.specials += [
                MultiReg(fpgacfg_manager.rx_en, s_reset_n, odomain=s_clk_domain),
                MultiReg(fpgacfg_manager.rx_en, m_reset_n, odomain=m_clk_domain),
            ]

        self.specials += [
            MultiReg(fpgacfg_manager.ch_en,      ch_en,            odomain=m_clk_domain),
            MultiReg(fpgacfg_manager.smpl_width, smpl_width,       odomain=m_clk_domain),
            MultiReg(fpgacfg_manager.synch_dis,  synch_dis,        odomain=m_clk_domain),
            MultiReg(self.pct_loss_flg_clr,      pct_loss_flg_clr, odomain=m_clk_domain),
        ]

        self.comb += [
            fifo_smpl_buff.reset.eq(~self.ext_reset_n),
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
