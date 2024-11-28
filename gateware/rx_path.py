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

# RX Path ------------------------------------------------------------------------------------------

class RXPath(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
        # RX parameters
        RX_IQ_WIDTH            = 12,
        RX_INVERT_INPUT_CLOCKS = "OFF",
        RX_SMPL_BUFF_RDUSEDW_W = 11,
        RX_PCT_BUFF_WRUSEDW_W  = 12,
        ):

        assert fpgacfg_manager is not None

        self.platform              = platform

        self.sink                  = AXIStreamInterface(64, clock_domain="lms_rx")
        self.source                = AXIStreamInterface(64, clock_domain="lms_rx")

        self.rx_pct_fifo_aclrn_req = Signal()

        self.pct_hdr_cap           = Signal()

        # Sample NR.
        self.smpl_nr_cnt           = Signal(64)

        # Flag Control.
        self.tx_pct_loss_flg       = Signal()

        # Sample Compare.
        self.smpl_cnt_en           = Signal()

        # # #

        # Signals.
        # --------

        # sync.
        inst5_reset_n          = Signal()
        mimo_en                = Signal()
        ddr_en                 = Signal()

        # IQ Stream Combiner To Rx Path Top.
        iq_to_bit_pack_tdata    = Signal(64)
        iq_to_bit_pack_tvalid   = Signal()
        iq_to_bit_pack_tready   = Signal()
        iq_to_bit_pack_tkeep    = Signal(8)
        bit_pack_to_nto1_tdata  = Signal(64)
        bit_pack_to_nto1_tvalid = Signal()
        bit_pack_to_nto1_tlast  = Signal()

        pct_hdr_0               = Signal(64)

        self.fifo_conv    = fifo_conv    = ResetInserter()(ClockDomainsRenamer("lms_rx")(stream.Converter(128, 64)))
        self.iqsmpls_fifo = iqsmpls_fifo = ResetInserter()(ClockDomainsRenamer("lms_rx")(stream.SyncFIFO([("data", 128)], 16)))

        self.comb += [
            fifo_conv.reset.eq(           ~inst5_reset_n),
            iqsmpls_fifo.reset.eq(        ~inst5_reset_n),
            self.fifo_conv.source.connect(self.source),

            # Packet Header 0
            pct_hdr_0.eq(Cat(Constant(0, 16), 0x060504030201)) # FIXME: 0:15: isn't 0 and 16:63 differs for XTRX
        ]

        self.specials += [
            MultiReg(fpgacfg_manager.rx_en,       inst5_reset_n,              "lms_rx", reset=1),
            MultiReg(inst5_reset_n,               self.rx_pct_fifo_aclrn_req, "lms_rx", reset=1),
            MultiReg(fpgacfg_manager.mimo_int_en, mimo_en,                    "lms_rx"),
            MultiReg(fpgacfg_manager.ddr_en,      ddr_en,                     "lms_rx"),
            # AXI Stream packager (removes null bytes from axi stream)
            # Combine IQ samples into full 64bit bus
            # In mimo Mode: AI AQ BI BQ
            # In siso Mode: AI AQ AI AQ
            Instance("IQ_STREAM_COMBINER",
                # Clk/Reset.
                i_CLK               = ClockSignal("lms_rx"), # S_AXIS_IQSMPLS_ACLK
                i_RESET_N           = inst5_reset_n,         # S_AXIS_IQSMPLS_ARESETN
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
                i_M_AXIS_TREADY     = iqsmpls_fifo.sink.ready, #iq_to_bit_pack_tready, # axis_iq128.tready
                o_M_AXIS_TDATA      = iq_to_bit_pack_tdata,
                o_M_AXIS_TKEEP      = iq_to_bit_pack_tkeep,  # Unused full 1
            ),
            # Bit packer
            Instance("bit_pack",
                # Parameters.
                p_G_PORT_WIDTH  = 64,
                #p_G_DISABLE_14BIT = false
                # Clk/Reset.
                i_CLK            = ClockSignal("lms_rx"), # S_AXIS_IQSMPLS_ACLK,
                i_RESET_N        = inst5_reset_n,         # S_AXIS_IQSMPLS_ARESETN,
                # AXI Stream Slave interface.
                i_DATA_IN        = iq_to_bit_pack_tdata,
                i_DATA_IN_VALID  = iq_to_bit_pack_tvalid,
                i_SAMPLE_WIDTH   = fpgacfg_manager.smpl_width,  # "10"-12bit, "01"-14bit, "00"-16bit;
                # AXI Stream Master interface.
                o_DATA_OUT       = bit_pack_to_nto1_tdata,
                o_DATA_OUT_VALID = bit_pack_to_nto1_tvalid,
                o_DATA_OUT_TLAST = bit_pack_to_nto1_tlast,      # always 1 when smpl_width == 0b00
            ),
            Instance("axis_nto1_converter",
                # Parameters.
                p_G_N_RATIO    = 2,
                p_G_DATA_WIDTH = 64,

                # Clk/Reset.
                i_ACLK           = ClockSignal("lms_rx"), # S_AXIS_IQSMPLS_ACLK,
                i_ARESET_N       = inst5_reset_n,         # S_AXIS_IQSMPLS_ARESETN,
                # AXIS Slave
                i_S_AXIS_TVALID = bit_pack_to_nto1_tvalid,
                o_S_AXIS_TREADY = Open(),
                i_S_AXIS_TDATA  = bit_pack_to_nto1_tdata,
                i_S_AXIS_TLAST  = bit_pack_to_nto1_tlast,
                # AXIS Master
                o_M_AXIS_TVALID = iqsmpls_fifo.sink.valid,
                o_M_AXIS_TDATA  = iqsmpls_fifo.sink.data,
                o_M_AXIS_TLAST  = iqsmpls_fifo.sink.last,
            ),
            Instance("rx_path_top",
                # Parameters.
                p_G_S_AXIS_IQSMPLS_BUFFER_WORDS  = 16,
                p_G_M_AXIS_IQPACKET_BUFFER_WORDS = 512,

                # Clk/Reset.
                i_clk                     = ClockSignal("lms_rx"),
                i_reset_n                 = inst5_reset_n,

                # AXI Stream Slave bus for IQ samples
                i_S_AXIS_IQSMPLS_ACLK     = ClockSignal("lms_rx"),
                i_S_AXIS_IQSMPLS_ARESETN  = inst5_reset_n,
                i_S_AXIS_IQSMPLS_TVALID   = iqsmpls_fifo.source.valid,
                o_S_AXIS_IQSMPLS_TREADY   = iqsmpls_fifo.source.ready,
                i_S_AXIS_IQSMPLS_TDATA    = iqsmpls_fifo.source.data,
                i_S_AXIS_IQSMPLS_TKEEP    = Open(8),
                i_S_AXIS_IQSMPLS_TLAST    = iqsmpls_fifo.source.last,

                # AXI Stream Master bus for Stream packets
                i_M_AXIS_IQPACKET_ACLK    = ClockSignal("lms_rx"),
                i_M_AXIS_IQPACKET_ARESETN = inst5_reset_n,
                o_M_AXIS_IQPACKET_TVALID  = fifo_conv.sink.valid,
                i_M_AXIS_IQPACKET_TREADY  = fifo_conv.sink.ready,
                o_M_AXIS_IQPACKET_TDATA   = fifo_conv.sink.data,
                o_M_AXIS_IQPACKET_TKEEP   = Open(8),
                o_M_AXIS_IQPACKET_TLAST   = fifo_conv.sink.last,

                # Mode settings.
                i_CFG_SMPL_WIDTH          = fpgacfg_manager.smpl_width,  # "10"-12bit, "01"-14bit, "00"-16bit;
                i_CFG_PKT_SIZE            = Constant(4096, 16),          # 256 x 128b = 4096Bytes
                i_ddr_en                  = ddr_en,                      # DDR: 1; SDR: 0
                i_mimo_en                 = mimo_en,                     # Enabled: 1; Disabled: 0
                i_CFG_CH_EN               = fpgacfg_manager.ch_en,       # "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.
                i_pct_hdr_0               = pct_hdr_0,

                # sample nr
                i_SMPL_NR_EN              = Constant(1, 1), # Unused
                i_SMPL_NR_INCR            = (self.sink.valid & iqsmpls_fifo.sink.ready),
                i_SMPL_NR_CLR             = fpgacfg_manager.smpl_nr_clr,
                i_SMPL_NR_LD              = Constant(0, 1),
                i_SMPL_NR_IN              = Constant(0, 64),
                o_SMPL_NR_OUT             = self.smpl_nr_cnt,

                # flag control
                i_TXFLAGS_PCT_LOSS         = self.tx_pct_loss_flg,
                i_TXFLAGS_PCT_LOSS_CLR     = fpgacfg_manager.txpct_loss_clr,
            )
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        general_periph_files = [
            "gateware/hdl/packages/synth/FIFO_PACK.vhd",
            "gateware/LimeDFB/rx_path_top/src/iq_stream_combiner.vhd",
            "gateware/LimeDFB/rx_path_top/src/axis_nto1_converter.vhd",
            "gateware/LimeDFB/rx_path_top/src/bit_pack.vhd",
            "gateware/LimeDFB/rx_path_top/src/data2packets_fsm.vhd",
            "gateware/LimeDFB/rx_path_top/src/pack_48_to_64.vhd",
            "gateware/LimeDFB/rx_path_top/src/pack_56_to_64.vhd",
            "gateware/LimeDFB/rx_path_top/src/rx_path_top.py",
            "gateware/LimeDFB/rx_path_top/src/rx_path_top.vhd",

            "gateware/LimeDFB/axis/src/axis_pkg.vhd",
        ]

        for file in general_periph_files:
            platform.add_source(file)
