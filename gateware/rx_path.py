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

        self.axis_s                = AXIStreamInterface(RX_IQ_WIDTH * 4, 8, clock_domain="lms_rx")

        self.rx_pct_fifo_aclrn_req = Signal()

        self.rx_pct_fifo_wusedw    = Signal(RX_PCT_BUFF_WRUSEDW_W)
        self.rx_pct_fifo_wrreq     = Signal()
        self.rx_pct_fifo_wdata     = Signal(64)

        self.pct_hdr_cap           = Signal()

        # Sample NR.
        self.smpl_nr_cnt           = Signal(64)

        # Flag Control.
        self.tx_pct_loss_flg       = Signal()

        # Sample Compare.
        self.smpl_cnt_en           = Signal()

        # # #

        inst5_reset_n   = Signal()

        self.specials += [
            MultiReg(fpgacfg_manager.rx_en, inst5_reset_n,              "lms_rx", reset=1),
            MultiReg(inst5_reset_n,         self.rx_pct_fifo_aclrn_req, "lms_rx", reset=1),
            Instance("rx_path_top",
                p_iq_width            = RX_IQ_WIDTH,
                p_invert_input_clocks = RX_INVERT_INPUT_CLOCKS,
                p_smpl_buff_rdusedw_w = RX_SMPL_BUFF_RDUSEDW_W,
                p_pct_buff_wrusedw_w  = RX_PCT_BUFF_WRUSEDW_W,

                # Clk/Reset.
                i_clk                 = ClockSignal("lms_rx"),
                i_reset_n             = inst5_reset_n,

                # Mode settings.
                i_sample_width        = fpgacfg_manager.smpl_width,  # "10"-12bit, "01"-14bit, "00"-16bit;
                i_mode                = fpgacfg_manager.mode,        # JESD207: 1; TRXIQ: 0
                i_trxiqpulse          = fpgacfg_manager.trxiq_pulse, # trxiqpulse on: 1; trxiqpulse off: 0
                i_ddr_en              = fpgacfg_manager.ddr_en,      # DDR: 1; SDR: 0
                i_mimo_en             = fpgacfg_manager.mimo_int_en, # SISO: 1; MIMO: 0
                i_ch_en               = fpgacfg_manager.ch_en,       # "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.

                # AXI Stream Slave Interface.
                i_s_axis_tdata   = self.axis_s.data,
                i_s_axis_tkeep   = self.axis_s.keep,
                i_s_axis_tvalid  = self.axis_s.valid,
                i_s_axis_tlast   = self.axis_s.last,
                o_s_axis_tready  = self.axis_s.ready,

                # samples
                o_smpl_fifo_wrreq_out = Open(),

                # Packet fifo ports
                i_pct_fifo_wusedw     = self.rx_pct_fifo_wusedw,
                o_pct_fifo_wrreq      = self.rx_pct_fifo_wrreq,
                o_pct_fifo_wdata      = self.rx_pct_fifo_wdata,
                o_pct_hdr_cap         = self.pct_hdr_cap,

                # sample nr
                i_clr_smpl_nr         = fpgacfg_manager.smpl_nr_clr,
                i_ld_smpl_nr          = Constant(0, 1),
                i_smpl_nr_in          = Constant(0, 64),
                o_smpl_nr_cnt         = self.smpl_nr_cnt,

                # flag control
                i_tx_pct_loss         = self.tx_pct_loss_flg,
                i_tx_pct_loss_clr     = fpgacfg_manager.txpct_loss_clr,

                # sample compare
                o_smpl_cnt_en          = self.smpl_cnt_en,
            )
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        general_periph_files = [
            "gateware/hdl/rx_path_top/bit_pack/synth/bit_pack.vhd",
            "gateware/hdl/rx_path_top/bit_pack/synth/pack_48_to_64.vhd",
            "gateware/hdl/rx_path_top/bit_pack/synth/pack_56_to_64.vhd",
            "gateware/hdl/rx_path_top/rx_path/synth/rx_path_top.vhd",
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
