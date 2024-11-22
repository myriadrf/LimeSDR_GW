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

from gateware.common import *

# RX Path ------------------------------------------------------------------------------------------

class TXPath(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
        # TX parameters
        IQ_WIDTH          = 12,
        PCT_MAX_SIZE      = 4096,
        PCT_HDR_SIZE      = 16,
        BUFF_COUNT        = 4,
        FIFO_DATA_W       = 128,
        ):

        assert fpgacfg_manager is not None

        self.platform          = platform

        self.source            = AXIStreamInterface(128, clock_domain="lms_rx")

        # FIFO
        self.stream_fifo_rd    = Signal()
        self.stream_fifo_data  = Signal(FIFO_DATA_W)
        self.stream_fifo_empty = Signal()

        self.rx_sample_nr      = Signal(64)
        self.pct_loss_flg      = Signal()
        self.pct_loss_flg_clr  = Signal()
        self.pct_buff_rdy      = Signal()

        self.pct_sync_pulse    = Signal()

        # # #

        reset_n = Signal()

        self.specials += [
            MultiReg(fpgacfg_manager.rx_en, reset_n, "lms_tx"),
            Instance("tx_path_top",
                p_g_IQ_WIDTH       = IQ_WIDTH,
                p_g_PCT_MAX_SIZE   = PCT_MAX_SIZE,
                p_g_PCT_HDR_SIZE   = PCT_HDR_SIZE,
                p_g_BUFF_COUNT     = BUFF_COUNT,
                p_g_FIFO_DATA_W    = FIFO_DATA_W,

                # Clk/Reset.
                i_pct_wrclk        = ClockSignal("lms_tx"),
                i_iq_rdclk         = ClockSignal("lms_tx"),
                i_reset_n          = reset_n,
                i_en               = reset_n,

                i_rx_sample_clk    = ClockSignal("lms_rx"),
                i_rx_sample_nr     = self.rx_sample_nr,

                i_pct_sync_dis     = fpgacfg_manager.synch_dis,
                i_pct_sync_size    = fpgacfg_manager.sync_size, # valid in external pulse mode only

                o_pct_loss_flg     = self.pct_loss_flg,
                i_pct_loss_flg_clr = self.pct_loss_flg_clr,

                o_pct_buff_rdy     = self.pct_buff_rdy,

                # Mode settings.
                i_mode             = fpgacfg_manager.mode,        # JESD207: 1; TRXIQ: 0
                i_trxiqpulse       = fpgacfg_manager.trxiq_pulse, # trxiqpulse on: 1; trxiqpulse off: 0
                i_ddr_en           = fpgacfg_manager.ddr_en,      # DDR: 1; SDR: 0
                i_mimo_en          = fpgacfg_manager.mimo_int_en, # SISO: 1; MIMO: 0
                i_ch_en            = fpgacfg_manager.ch_en,       # "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.
                i_fidm             = Constant(0, 1),
                i_sample_width     = fpgacfg_manager.smpl_width,  # "10"-12bit, "01"-14bit, "00"-16bit;

                # AXI Stream Master Interface.
                o_axis_m_tdata     = self.source.data,
                o_axis_m_tvalid    = self.source.valid,
                i_axis_m_tready    = self.source.ready,
                o_axis_m_tlast     = self.source.last,

                # Packet ports
                o_fifo_rdreq       = self.stream_fifo_rd,
                i_fifo_data        = self.stream_fifo_data,
                i_fifo_rdempty     = self.stream_fifo_empty,
            ),
            # pulse_gen instance instance.
            Instance("pulse_gen",
                i_clk         = ClockSignal("lms_tx"),
                i_reset_n     = reset_n,
                i_wait_cycles = fpgacfg_manager.sync_pulse_period,
                o_pulse       = self.pct_sync_pulse
            )
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        general_periph_files = [
            "gateware/hdl/tx_path_top/tx_path/synth/sync_fifo_rw.vhd",
            "gateware/hdl/tx_path_top/tx_path/synth/tx_path_top.vhd",
            "gateware/hdl/tx_path_top/pulse_gen/synth/pulse_gen.vhd",
            "gateware/hdl/tx_path_top/pct_separate/synth/one_pct_fifo.vhd",
            "gateware/hdl/tx_path_top/pct_separate/synth/pct_separate_fsm.vhd",
            "gateware/hdl/tx_path_top/packets2data/synth/p2d_rd.vhd",
            "gateware/hdl/tx_path_top/packets2data/synth/p2d_rd_fsm.vhd",
            "gateware/hdl/tx_path_top/packets2data/synth/p2d_sync_fsm.vhd",
            "gateware/hdl/tx_path_top/packets2data/synth/p2d_wr_fsm.vhd",
            "gateware/hdl/tx_path_top/packets2data/synth/packets2data.vhd",
            "gateware/hdl/tx_path_top/packets2data/synth/packets2data_top.vhd",
            "gateware/hdl/tx_path_top/handshake_sync/synth/handshake_sync.vhd",

            # Lattice FIFOs.
            # --------------
            "gateware/ip/fifodc_w128x256_r128.vhd", # one_pct_fifo.vhd.
            "gateware/ip/fifodc_w128x256_r64.vhd",  # packets2data.vhd.
        ]

        for file in general_periph_files:
            platform.add_source(file)
