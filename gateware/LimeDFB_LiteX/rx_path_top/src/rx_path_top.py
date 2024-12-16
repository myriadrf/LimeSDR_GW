#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect.csr            import CSRStatus, CSRStorage, CSRField
from litex.soc.interconnect                import stream

from gateware.common              import *

# RX Path Top --------------------------------------------------------------------------------------

class RXPathTop(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
        # RX parameters
        RX_IQ_WIDTH                  = 12,
        S_AXIS_IQSMPLS_BUFFER_WORDS  = 16,
        M_AXIS_IQPACKET_BUFFER_WORDS = 512,
        int_clk_domain               = "lms_rx",
        m_clk_domain                 = "lms_rx",
        s_clk_domain                 = "lms_rx"
        ):

        assert fpgacfg_manager is not None

        self.platform              = platform

        self.sink                  = AXIStreamInterface(64, clock_domain=s_clk_domain)
        self.source                = AXIStreamInterface(64, clock_domain=m_clk_domain)

        self.rx_pct_fifo_aclrn_req = Signal()

        self.pct_hdr_cap           = Signal()

        # Sample NR.
        self.smpl_nr_cnt           = Signal(64)

        # Flag Control.
        self.tx_pct_loss_flg       = Signal()

        # Sample Compare.
        self.smpl_cnt_en           = Signal()

        self.pkt_size = CSRStorage(16, reset=253,
            description="Packet Size in bytes, "
        )

        # # #

        # Signals.
        # --------

        # sync.
        s_clk_rst_n            = Signal()
        m_clk_rst_n            = Signal()
        int_clk_rst_n          = Signal()
        int_clk_smpl_nr_clr    = Signal()
        int_clk_ch_en          = Signal(2)
        mimo_en                = Signal()
        ddr_en                 = Signal()
        s_smpl_width           = Signal(2)

        # IQ Stream Combiner To Rx Path Top.
        iq_to_bit_pack_tdata    = Signal(64)
        iq_to_bit_pack_tvalid   = Signal()
        iq_to_bit_pack_tready   = Signal()
        iq_to_bit_pack_tkeep    = Signal(8)
        bit_pack_to_nto1_tdata  = Signal(64)
        bit_pack_to_nto1_tvalid = Signal()
        bit_pack_to_nto1_tlast  = Signal()

        pct_hdr_0               = Signal(64)

        iqpacket_axis           = stream.Endpoint([("data", 128)])

        iqpacket_wr_data_count  = Signal(9)
        bp_sample_nr_counter    = Signal(64)
        pkt_size                = Signal(15)

        iqsmpls_fifo_sink_ready   = Signal()
        iqsmpls_fifo_sink_valid   = Signal()
        iqsmpls_fifo_source_valid = Signal()
        iqsmpls_fifo_source_ready = Signal()


        self.fifo_conv    = fifo_conv = ResetInserter()(ClockDomainsRenamer(m_clk_domain)(stream.Converter(128, 64)))
        iqsmpls_fifo      = stream.AsyncFIFO([("data", 128)], 16)
        iqsmpls_fifo      = ClockDomainsRenamer({"write": s_clk_domain, "read": int_clk_domain})(iqsmpls_fifo)
        self.iqsmpls_fifo = iqsmpls_fifo

        self.comb += fifo_conv.reset.eq(~m_clk_rst_n)

        if platform.name.startswith("limesdr_mini"):
            self.comb += [
                # Packet Header 0
                pct_hdr_0.eq(Cat(Constant(0, 16), 0x060504030201)), # FIXME: 0:15: isn't 0 and 16:63 differs for XTRX
                pkt_size.eq(Constant(4096 // 16, 16)),              # 256 * 128b = 4096Bytes
            ]
        else:
            self.comb += [
                # Packet Header 0
                pct_hdr_0.eq(0x7766554433221100),
                pkt_size.eq(Cat(Constant(0, 3), self.pkt_size.storage)[7:]),
            ]

        # AXI Stream packager (removes null bytes from axi stream)
        # Combine IQ samples into full 64bit bus
        # In mimo Mode: AI AQ BI BQ
        # In siso Mode: AI AQ AI AQ
        self.iq_stream_combiner_params = dict()
        self.iq_stream_combiner_params.update(
            # Clk/Reset.
            i_CLK               = ClockSignal(s_clk_domain), # S_AXIS_IQSMPLS_ACLK
            i_RESET_N           = s_clk_rst_n,               # S_AXIS_IQSMPLS_ARESETN
            # Mode Settings.
            i_ddr_en            = ddr_en,
            i_mimo_en           = mimo_en,
            # AXI Stream Slave
            i_S_AXIS_TVALID     = self.sink.valid,
            o_S_AXIS_TREADY     = self.sink.ready,
            i_S_AXIS_TDATA      = self.sink.data,
            i_S_AXIS_TKEEP      = self.sink.keep,
            # AXI Stream Master
            o_M_AXIS_TVALID     = iq_to_bit_pack_tvalid,
            i_M_AXIS_TREADY     = iqsmpls_fifo_sink_ready, #iq_to_bit_pack_tready, # axis_iq128.tready
            o_M_AXIS_TDATA      = iq_to_bit_pack_tdata,
            o_M_AXIS_TKEEP      = iq_to_bit_pack_tkeep,  # Unused full 1
        )

        # Bit packer
        self.bit_pack_params = dict()
        self.bit_pack_params.update(
            # Parameters.
            p_G_PORT_WIDTH  = 64,
            #p_G_DISABLE_14BIT = false
            # Clk/Reset.
            i_clk            = ClockSignal(s_clk_domain), # S_AXIS_IQSMPLS_ACLK,
            i_reset_n        = s_clk_rst_n,               # S_AXIS_IQSMPLS_ARESETN,
            # AXI Stream Slave interface.
            i_data_in        = iq_to_bit_pack_tdata,
            i_data_in_valid  = iq_to_bit_pack_tvalid,
            i_sample_width   = s_smpl_width,              # "10"-12bit, "01"-14bit, "00"-16bit;
            # AXI Stream Master interface.
            o_data_out       = bit_pack_to_nto1_tdata,
            o_data_out_valid = bit_pack_to_nto1_tvalid,
            o_data_out_tlast = bit_pack_to_nto1_tlast,      # always 1 when smpl_width == 0b00
        )

        self.axis_nto1_converter_params = dict()
        self.axis_nto1_converter_params.update(
            # Parameters.
            p_G_N_RATIO    = 2,
            p_G_DATA_WIDTH = 64,

            # Clk/Reset.
            i_ACLK           = ClockSignal(s_clk_domain), # S_AXIS_IQSMPLS_ACLK,
            i_ARESET_N       = s_clk_rst_n,               # S_AXIS_IQSMPLS_ARESETN,
            # AXIS Slave
            i_S_AXIS_TVALID = bit_pack_to_nto1_tvalid,
            o_S_AXIS_TREADY = Open(),
            i_S_AXIS_TDATA  = bit_pack_to_nto1_tdata,
            i_S_AXIS_TLAST  = bit_pack_to_nto1_tlast,
            # AXIS Master
            o_M_AXIS_TVALID = iqsmpls_fifo_sink_valid,
            o_M_AXIS_TDATA  = iqsmpls_fifo.sink.data,
            o_M_AXIS_TLAST  = iqsmpls_fifo.sink.last,
        )

        self.rx_path_top_params = dict()
        self.rx_path_top_params.update(
            # Parameters.
            p_G_S_AXIS_IQSMPLS_BUFFER_WORDS  = S_AXIS_IQSMPLS_BUFFER_WORDS,
            p_G_M_AXIS_IQPACKET_BUFFER_WORDS = M_AXIS_IQPACKET_BUFFER_WORDS,

            # Clk/Reset.
            i_CLK                     = ClockSignal(int_clk_domain),
            i_RESET_N                 = int_clk_rst_n,

            # AXI Stream Slave bus for IQ samples
            i_S_AXIS_IQSMPLS_ACLK     = ClockSignal(s_clk_domain),
            i_S_AXIS_IQSMPLS_ARESETN  = s_clk_rst_n,
            i_S_AXIS_IQSMPLS_TVALID   = iqsmpls_fifo_source_valid,
            i_S_AXIS_IQSMPLS_TREADY   = iqsmpls_fifo_source_ready,
            i_S_AXIS_IQSMPLS_TLAST    = iqsmpls_fifo.source.last,

            # Mode settings.
            i_CFG_SMPL_WIDTH          = s_smpl_width,                # "10"-12bit, "01"-14bit, "00"-16bit;
            i_CFG_CH_EN               = int_clk_ch_en,               # "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.

            # sample nr
            i_SMPL_NR_INCR            = (self.sink.valid & iqsmpls_fifo_sink_ready),
            i_SMPL_NR_CLR             = int_clk_smpl_nr_clr,
            i_SMPL_NR_LD              = Constant(0, 1),
            i_SMPL_NR_IN              = Constant(0, 64),
            o_SMPL_NR_OUT             = self.smpl_nr_cnt,
            o_bp_sample_nr_counter    = bp_sample_nr_counter,
        )

        self.data2packets_fsm_params = dict()
        self.data2packets_fsm_params.update(
            # Clk/Reset.
            i_ACLK               = ClockSignal(int_clk_domain),
            i_ARESET_N           = int_clk_rst_n,
            i_PCT_SIZE           = pkt_size,
            # PCT_HDR_0          = x"7766554433221100",
            i_PCT_HDR_0          = pct_hdr_0,
            i_PCT_HDR_1          = bp_sample_nr_counter,
            # AXIS Slave.
            i_S_AXIS_TVALID      = iqsmpls_fifo_source_valid,
            o_S_AXIS_TREADY      = iqsmpls_fifo_source_ready,
            i_S_AXIS_TDATA       = iqsmpls_fifo.source.data,
            i_S_AXIS_TLAST       = iqsmpls_fifo.source.last,
            # AXIS Master.
            o_M_AXIS_TVALID      = iqpacket_axis.valid,
            i_M_AXIS_TREADY      = iqpacket_axis.ready,
            o_M_AXIS_TDATA       = iqpacket_axis.data,
            o_M_AXIS_TLAST       = iqpacket_axis.last,
            o_WR_DATA_COUNT_AXIS = iqpacket_wr_data_count
        )

        self.comb += [
            If(int_clk_rst_n,
               iqsmpls_fifo_sink_ready.eq(iqsmpls_fifo.sink.ready),
               iqsmpls_fifo.sink.valid.eq(iqsmpls_fifo_sink_valid),
               iqsmpls_fifo_source_valid.eq(iqsmpls_fifo.source.valid),
               iqsmpls_fifo.source.ready.eq(iqsmpls_fifo_source_ready),
            ).Else(
               iqsmpls_fifo_sink_ready.eq(0),
               iqsmpls_fifo.sink.valid.eq(0),
               iqsmpls_fifo_source_valid.eq(0),
               iqsmpls_fifo.source.ready.eq(1),
            )
        ]

        if M_AXIS_IQPACKET_BUFFER_WORDS > 0:
            # FIFO before Converter
            fifo_iqpacket = ResetInserter()(ClockDomainsRenamer(int_clk_domain)(stream.SyncFIFO([("data", 128)],
                depth    = M_AXIS_IQPACKET_BUFFER_WORDS,
                buffered = True)))
            self.fifo_iqpacket = fifo_iqpacket

            self.iqpacket_cdc = iqpacket_cdc = stream.ClockDomainCrossing([("data", 128)],
                cd_from = int_clk_domain,
                cd_to   = m_clk_domain,
            )

            self.comb += [
                fifo_iqpacket.reset.eq(~int_clk_rst_n), # axis_iqmpls_areset_n
                iqpacket_wr_data_count.eq(fifo_iqpacket.level),
            ]
            self.iqpacket_pipeline = stream.Pipeline(
                iqpacket_axis,
                fifo_iqpacket.sink,
            )
            self.comb += [
                fifo_iqpacket.source.connect(iqpacket_cdc.sink),
                iqpacket_cdc.source.connect(fifo_conv.sink),
                If(~int_clk_rst_n,
                    iqpacket_cdc.sink.valid.eq(0),
                    fifo_iqpacket.source.ready.eq(0),
                    fifo_conv.sink.valid.eq(0),
                    iqpacket_cdc.source.ready.eq(1)
                )
            ]
            self.iqpacket_pipeline2 = stream.Pipeline(
                fifo_conv.source,
                self.source,
            )
        else:
            self.comb += iqpacket_wr_data_count.eq(0) # FIXME: correct value ?
            self.iqpacket_pipeline = stream.Pipeline(
                iqpacket_axis,
                fifo_conv,
                self.source,
            )

        # CDC. -------------------------------------------------------------------------------------
        if s_clk_domain == "sys":
            self.comb += [
                s_clk_rst_n.eq(               fpgacfg_manager.rx_en),
                self.rx_pct_fifo_aclrn_req.eq(s_clk_rst_n),
                mimo_en.eq(                   fpgacfg_manager.mimo_int_en),
                ddr_en.eq(                    fpgacfg_manager.ddr_en),
                s_smpl_width.eq(              fpgacfg_manager.smpl_width),
            ]
        else:
            self.specials += [
                MultiReg(fpgacfg_manager.rx_en,       s_clk_rst_n,                s_clk_domain, reset=1),
                MultiReg(s_clk_rst_n,                 self.rx_pct_fifo_aclrn_req, s_clk_domain, reset=1),
                MultiReg(fpgacfg_manager.mimo_int_en, mimo_en,                    s_clk_domain),
                MultiReg(fpgacfg_manager.ddr_en,      ddr_en,                     s_clk_domain),
                MultiReg(fpgacfg_manager.smpl_width,  s_smpl_width,               s_clk_domain),
            ]

        if int_clk_domain == "sys":
            self.comb += [
                int_clk_rst_n.eq(      fpgacfg_manager.rx_en),
                int_clk_ch_en.eq(      fpgacfg_manager.ch_en),
                int_clk_smpl_nr_clr.eq(fpgacfg_manager.smpl_nr_clr),
            ]
        else:
            self.specials += [
                MultiReg(fpgacfg_manager.rx_en,       int_clk_rst_n,       int_clk_domain, reset=1),
                MultiReg(fpgacfg_manager.ch_en,       int_clk_ch_en,       int_clk_domain, reset=0),
                MultiReg(fpgacfg_manager.smpl_nr_clr, int_clk_smpl_nr_clr, int_clk_domain, reset=0),
            ]
        if m_clk_domain == "sys":
            self.comb += m_clk_rst_n.eq(fpgacfg_manager.rx_en)
        else:
            self.specials += MultiReg(fpgacfg_manager.rx_en, m_clk_rst_n, m_clk_domain, reset=1)


    def do_finalize(self):
        self.iq_stream_combiner = add_vhd2v_converter(self.platform,
            top    = "IQ_STREAM_COMBINER",
            params = self.iq_stream_combiner_params,
            files  = ["gateware/LimeDFB_LiteX/rx_path_top/src/iq_stream_combiner.vhd"],
        )

        bit_pack_files = [
            "gateware/LimeDFB/rx_path_top/src/bit_pack.vhd",
            "gateware/LimeDFB/rx_path_top/src/pack_48_to_64.vhd",
            "gateware/LimeDFB/rx_path_top/src/pack_56_to_64.vhd",
        ]

        self.bit_pack = add_vhd2v_converter(self.platform,
            top    = "bit_pack",
            params = self.bit_pack_params,
            files  = bit_pack_files,
        )

        self.axis_nto1_converter = add_vhd2v_converter(self.platform,
            top    = "AXIS_NTO1_CONVERTER",
            params = self.axis_nto1_converter_params,
            files  = ["gateware/LimeDFB/rx_path_top/src/axis_nto1_converter.vhd"],
        )

        self.rx_path_top = add_vhd2v_converter(self.platform,
            top    = "RX_PATH_TOP",
            params = self.rx_path_top_params,
            files  = ["gateware/LimeDFB_LiteX/rx_path_top/src/rx_path_top.vhd"],
        )

        self.data2packets_fsm = add_vhd2v_converter(self.platform,
            top    = "DATA2PACKETS_FSM",
            params = self.data2packets_fsm_params,
            files  = ["gateware/LimeDFB/rx_path_top/src/data2packets_fsm.vhd"],
        )
