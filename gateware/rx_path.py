#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.csr import CSRStatus, CSRStorage, CSRField

from gateware.common              import *
from gateware.lms7002.lms7002_top import SampleCompare

# RX Path ------------------------------------------------------------------------------------------

class RXPath(LiteXModule):
    def __init__(self, platform,
        # RX parameters
        RX_IQ_WIDTH            = 12,
        RX_INVERT_INPUT_CLOCKS = "OFF",
        RX_SMPL_BUFF_RDUSEDW_W = 11,
        RX_PCT_BUFF_WRUSEDW_W  = 12,
        ):

        self.platform              = platform

        self.rx_diq2_h             = Signal(RX_IQ_WIDTH + 1)
        self.rx_diq2_l             = Signal(RX_IQ_WIDTH + 1)
        self.rx_pct_fifo_aclrn_req = Signal()

        self.rx_pct_fifo_wusedw    = Signal(RX_PCT_BUFF_WRUSEDW_W)
        self.rx_pct_fifo_wrreq     = Signal()
        self.rx_pct_fifo_wdata     = Signal(64)

        self.rx_en                 = Signal()
        self.pct_hdr_cap           = Signal()

        # from fpgacfg
        self.rx_ptrn_en            = Signal()
        self.smpl_width            = Signal(2)
        self.mode                  = Signal()
        self.trxiqpulse            = Signal()
        self.ddr_en                = Signal()
        self.mimo_en               = Signal()
        self.ch_en                 = Signal(2)

        # Sample NR.
        self.smpl_nr_clr           = Signal()
        self.smpl_nr_cnt           = Signal(64)

        # Flag Control.
        self.tx_pct_loss_flg       = Signal()
        self.tx_pct_loss_clr       = Signal()

        # Sample Compare.
        self.rx_smpl_cmp_start     = Signal()
        self.rx_smpl_cmp_length    = Signal(16)
        self.rx_smpl_cmp_done      = Signal()
        self.rx_smpl_cmp_err       = Signal()

        # # #

        inst5_reset_n   = Signal()

        self.specials += [
            Instance("sync_reg",
                i_clk         = ClockSignal("lms_rx"),
                i_reset_n     = self.rx_en,
                i_async_in    = Constant(1),
                o_sync_out    = inst5_reset_n,
            ),
            Instance("sync_reg",
                i_clk         = ClockSignal("lms_rx"),
                i_reset_n     = inst5_reset_n,
                i_async_in    = Constant(1),
                o_sync_out    = self.rx_pct_fifo_aclrn_req,
            ),
            Instance("rx_path_top",
                p_iq_width            = RX_IQ_WIDTH,
                p_invert_input_clocks = RX_INVERT_INPUT_CLOCKS,
                p_smpl_buff_rdusedw_w = RX_SMPL_BUFF_RDUSEDW_W,
                p_pct_buff_wrusedw_w  = RX_PCT_BUFF_WRUSEDW_W,

                # Clk/Reset.
                i_clk                 = ClockSignal("lms_rx"),
                i_reset_n             = inst5_reset_n,
                i_test_ptrn_en        = self.rx_ptrn_en,

                # Mode settings.
                i_sample_width        = self.smpl_width, # "10"-12bit, "01"-14bit, "00"-16bit;
                i_mode                = self.mode,       # JESD207: 1; TRXIQ: 0
                i_trxiqpulse          = self.trxiqpulse, # trxiqpulse on: 1; trxiqpulse off: 0
                i_ddr_en              = self.ddr_en,     # DDR: 1; SDR: 0
                i_mimo_en             = self.mimo_en,    # SISO: 1; MIMO: 0
                i_ch_en               = self.ch_en,      # "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.
                i_fidm                = Constant(0, 1),  # Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
                # Rx interface data
                i_rx_diq2_h           = self.rx_diq2_h,
                i_rx_diq2_l           = self.rx_diq2_l,

                # samples
                o_smpl_fifo_wrreq_out = Open(),

                # Packet fifo ports
                i_pct_fifo_wusedw     = self.rx_pct_fifo_wusedw,
                o_pct_fifo_wrreq      = self.rx_pct_fifo_wrreq,
                o_pct_fifo_wdata      = self.rx_pct_fifo_wdata,
                o_pct_hdr_cap         = self.pct_hdr_cap,

                # sample nr
                i_clr_smpl_nr         = self.smpl_nr_clr,
                i_ld_smpl_nr          = Constant(0, 1),
                i_smpl_nr_in          = Constant(0, 64),
                o_smpl_nr_cnt         = self.smpl_nr_cnt,

                # flag control
                i_tx_pct_loss         = self.tx_pct_loss_flg,
                i_tx_pct_loss_clr     = self.tx_pct_loss_clr,

                # sample compare
                i_smpl_cmp_start      = self.rx_smpl_cmp_start,
                i_smpl_cmp_length     = self.rx_smpl_cmp_length,
                o_smpl_cmp_done       = self.rx_smpl_cmp_done,
                o_smpl_cmp_err        = self.rx_smpl_cmp_err,
            )
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        general_periph_files = [
            "gateware/hdl/rx_path_top/bit_pack/synth/bit_pack.vhd",
            "gateware/hdl/rx_path_top/bit_pack/synth/pack_48_to_64.vhd",
            "gateware/hdl/rx_path_top/bit_pack/synth/pack_56_to_64.vhd",
            "gateware/hdl/rx_path_top/smpl_cmp/synth/smpl_cmp.vhd",
            "gateware/hdl/rx_path_top/rx_path/synth/rx_path_top.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/diq2fifo.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq_mimo.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq_mimo_ddr.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq_pulse_ddr.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq_siso.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq_siso_ddr.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/rxiq_siso_sdr.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/test_data_dd.vhd",
            "gateware/hdl/rx_path_top/data2packets/synth/data2packets.vhd",
            "gateware/hdl/rx_path_top/data2packets/synth/data2packets_fsm.vhd",
            "gateware/hdl/rx_path_top/data2packets/synth/data2packets_top.vhd",
            "gateware/hdl/tx_path_top/fifo2diq/synth/edge_delay.vhd",
            "gateware/hdl/rx_path_top/smpl_cnt/synth/iq_smpl_cnt.vhd",
            "gateware/hdl/rx_path_top/smpl_cnt/synth/smpl_cnt.vhd",

            # Lattice FIFOs.
            # --------------
            "gateware/ip/fifodc_w48x1024_r48.vhd",  # rx_path_top.vhd.
        ]

        for file in general_periph_files:
            platform.add_source(file)
