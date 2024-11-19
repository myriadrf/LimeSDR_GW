#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import CSRStatus, CSRStorage, CSRField

from gateware.common              import *
from gateware.lms7002.lms7002_top import SampleCompare
from gateware.rx_path             import RXPath

# RXTX Top -----------------------------------------------------------------------------------------

class RXTXTop(LiteXModule):
    def __init__(self, platform,
        # TX parameters
        TX_IQ_WIDTH            = 12,
        TX_N_BUFF              = 4,
        TX_IN_PCT_SIZE         = 4096,
        TX_IN_PCT_HDR_SIZE     = 16,
        TX_IN_PCT_DATA_W       = 128,
        TX_IN_PCT_RDUSEDW_W    = 11,
        TX_OUT_PCT_DATA_W      = 64,

        # RX parameters
        RX_IQ_WIDTH            = 12,
        RX_INVERT_INPUT_CLOCKS = "OFF",
        RX_SMPL_BUFF_RDUSEDW_W = 11,
        RX_PCT_BUFF_WRUSEDW_W  = 12,
        ):

        self.platform              = platform

        self.tx_txant_en           = Signal()
        self.tx_diq1_h             = Signal(TX_IQ_WIDTH + 1)
        self.tx_diq1_l             = Signal(TX_IQ_WIDTH + 1)

        self.rx_diq2_h             = Signal(RX_IQ_WIDTH + 1)
        self.rx_diq2_l             = Signal(RX_IQ_WIDTH + 1)
        self.rx_pct_fifo_aclrn_req = Signal()
        self.rx_smpl_cmp           = SampleCompare()
        self.rxtx_smpl_cmp_length  = Signal(16)

        self.rx_en                    = Signal()
        self.from_tstcfg_test_en      = Signal(6)
        self.from_tstcfg_test_frc_err = Signal(6)
        self.from_tstcfg_tx_tst_i     = Signal(16)
        self.from_tstcfg_tx_tst_q     = Signal(16)

        self.stream_fifo           = FIFOInterface(TX_IN_PCT_DATA_W, 64, TX_IN_PCT_RDUSEDW_W, RX_PCT_BUFF_WRUSEDW_W)

        # From FPGA Cfg.
        # --------------
        # Peripheral config.
        self.txant_pre         = CSRStorage(16, reset=1)
        self.txant_post        = CSRStorage(16, reset=1)

        # Interface config
        self.channel_cntrl     = CSRStorage(fields=[
            CSRField("ch_en", size=2, offset=0, reset=0b11, values=[
                ("``2b01", "Channel A"),
                ("``2b10", "Channel B"),
                ("``2b11", "Channels A and B")
            ])
        ])
        self.reg08             = CSRStorage(fields=[
            CSRField("smpl_width",  size=2, offset=0,  reset=0b10),
            CSRField("mode",        size=1, offset=5,  reset=0),
            CSRField("ddr_en",      size=1, offset=6,  reset=0),
            CSRField("trxiq_pulse", size=1, offset=7,  reset=0),
            CSRField("mimo_int_en", size=1, offset=8,  reset=1),
            CSRField("synch_dis",   size=1, offset=9,  reset=0),
            CSRField("synch_mode",  size=1, offset=10, reset=0),
        ])
        self.reg09             = CSRStorage(fields=[
            CSRField("smpl_nr_clr",    size=1, offset=0, reset=1),
            CSRField("txpct_loss_clr", size=1, offset=1, reset=1),
        ])
        self.reg10             = CSRStorage(fields=[
            CSRField("rx_en",       size=1, offset=0),
            CSRField("tx_en",       size=1, offset=1),
            CSRField("rx_ptrn_en" , size=1, offset=8),
            CSRField("tx_ptrn_en",  size=1, offset=9),
            CSRField("tx_cnt_en",   size=1, offset=10),
        ])

        self.reg13             = CSRStorage(fields=[
            CSRField("wfm_play", size=1, offset=1),
            CSRField("wfm_load", size=1, offset=2),
        ])

        self.sync_size         = CSRStorage(16, reset=0x03FC)

        # Peripheral config.

        self.sync_pulse_period = CSRStorage(32, reset=0x3D090)            # 30

        # Test Cfg From RXTX.
        # -------------------
        # testcfg_from_rxtx @22
        self._ddr2_1_status        = CSRStatus(3)
        # testcfg_from_rxtx @23
        self._ddr2_1_pnf_per_bit_l = CSRStatus(16)
        # testcfg_from_rxtx @24
        self._ddr2_1_pnf_per_bit_h = CSRStatus(16)

        # # #

        # Signals.
        # --------
        self._ddr2_1_pnf_per_bit = Signal(32)
        pct_hdr_cap              = Signal()
        smpl_nr_cnt              = Signal(64)
        tx_pct_loss_flg          = Signal()

        # rxtx_top wrapper (required due to record).
        # ------------------------------------------
        self.specials += Instance("rxtx_top_wrapper",
            # TX parameters
            p_TX_IQ_WIDTH            = TX_IQ_WIDTH,
            p_TX_N_BUFF              = TX_N_BUFF,
            p_TX_IN_PCT_SIZE         = TX_IN_PCT_SIZE,
            p_TX_IN_PCT_HDR_SIZE     = TX_IN_PCT_HDR_SIZE,
            p_TX_IN_PCT_DATA_W       = TX_IN_PCT_DATA_W,
            p_TX_IN_PCT_RDUSEDW_W    = TX_IN_PCT_RDUSEDW_W,
            p_TX_OUT_PCT_DATA_W      = TX_OUT_PCT_DATA_W,

            # Configuration memory ports
            #from_fpgacfg            : in     t_FROM_FPGACFG;
            i_ch_en                  = Cat(self.channel_cntrl.fields.ch_en, Constant(0, 14)),
            i_smpl_width             = self.reg08.fields.smpl_width,
            i_mode                   = self.reg08.fields.mode,
            i_ddr_en                 = self.reg08.fields.ddr_en,
            i_trxiq_pulse            = self.reg08.fields.trxiq_pulse,
            i_mimo_int_en            = self.reg08.fields.mimo_int_en,
            i_synch_dis              = self.reg08.fields.synch_dis,
            i_synch_mode             = self.reg08.fields.synch_mode,
            i_txpct_loss_clr         = self.reg09.fields.txpct_loss_clr,
            i_rx_en                  = self.reg10.fields.rx_en,
            i_tx_ptrn_en             = self.reg10.fields.tx_ptrn_en,
            i_tx_cnt_en              = self.reg10.fields.tx_cnt_en,
            i_wfm_play               = self.reg13.fields.wfm_play,
            i_sync_pulse_period      = self.sync_pulse_period.storage,
            i_sync_size              = self.sync_size.storage,
            i_txant_pre              = self.txant_pre.storage,
            i_txant_post             = self.txant_post.storage,

            ##to_tstcfg_from_rxtx     : out    t_TO_TSTCFG_FROM_RXTX;
            o_DDR2_1_STATUS          = self._ddr2_1_status.status,
            o_DDR2_1_pnf_per_bit     = self._ddr2_1_pnf_per_bit,
            #from_tstcfg             : in     t_FROM_TSTCFG;
            i_TEST_EN                = self.from_tstcfg_test_en,
            i_TEST_FRC_ERR           = self.from_tstcfg_test_frc_err,
            i_TX_TST_I               = self.from_tstcfg_tx_tst_i,
            i_TX_TST_Q               = self.from_tstcfg_tx_tst_q,

            ## TX path
            i_tx_clk                 = ClockSignal("lms_tx"),
            o_tx_clkout              = Open(),
            i_tx_clk_reset_n         = ~ResetSignal("sys"),
            o_tx_pct_loss_flg        = tx_pct_loss_flg,
            o_tx_txant_en            = self.tx_txant_en,
            #  Tx interface data
            o_tx_diq1_h              = self.tx_diq1_h,
            o_tx_diq1_l              = self.tx_diq1_l,
            #  TX FIFO read ports
            o_tx_in_pct_rdreq        = self.stream_fifo.rd,
            i_tx_in_pct_data         = self.stream_fifo.rdata,
            i_tx_in_pct_rdempty      = self.stream_fifo.empty,
            i_tx_in_pct_rdusedw      = self.stream_fifo.rdusedw,

            i_smpl_nr_cnt            = smpl_nr_cnt,
            i_pct_hdr_cap            = pct_hdr_cap,


            ## RX path
            i_rx_clk                 = ClockSignal("lms_rx"),
        )

        self.rx_path = rx_path = RXPath(platform,
            ## RX parameters
            RX_IQ_WIDTH            = RX_IQ_WIDTH,
            RX_SMPL_BUFF_RDUSEDW_W = RX_SMPL_BUFF_RDUSEDW_W,
            RX_PCT_BUFF_WRUSEDW_W  = RX_PCT_BUFF_WRUSEDW_W,
        )

        self.comb += [
            rx_path.rx_en.eq(             self.reg10.fields.rx_en),
            rx_path.rx_ptrn_en.eq(        self.reg10.fields.rx_ptrn_en),

            # Mode settings.
            rx_path.smpl_width.eq(        self.reg08.fields.smpl_width),
            rx_path.mode.eq(              self.reg08.fields.mode),
            rx_path.trxiqpulse.eq(        self.reg08.fields.trxiq_pulse),
            rx_path.ddr_en.eq(            self.reg08.fields.ddr_en),
            rx_path.mimo_en.eq(           self.reg08.fields.mimo_int_en),
            rx_path.ch_en.eq(             self.channel_cntrl.fields.ch_en),

            # Rx interface data
            rx_path.rx_diq2_h.eq(         self.rx_diq2_h),
            rx_path.rx_diq2_l.eq(         self.rx_diq2_l),

            # Packet fifo ports
            self.rx_pct_fifo_aclrn_req.eq(rx_path.rx_pct_fifo_aclrn_req),
            rx_path.rx_pct_fifo_wusedw.eq(self.stream_fifo.wrusedw),
            self.stream_fifo.wr.eq(       rx_path.rx_pct_fifo_wrreq),
            self.stream_fifo.wdata.eq(    rx_path.rx_pct_fifo_wdata),

            # sample nr
            rx_path.smpl_nr_clr.eq(       self.reg09.fields.smpl_nr_clr),
            smpl_nr_cnt.eq(               rx_path.smpl_nr_cnt),

            # Flag Control.
            rx_path.tx_pct_loss_flg.eq(   tx_pct_loss_flg),
            rx_path.tx_pct_loss_clr.eq(   self.reg09.fields.txpct_loss_clr),

            # Sample Compare.
            rx_path.rx_smpl_cmp_start.eq( self.rx_smpl_cmp.en),
            rx_path.rx_smpl_cmp_length.eq(self.rxtx_smpl_cmp_length),
            self.rx_smpl_cmp.done.eq(     rx_path.rx_smpl_cmp_done),
            self.rx_smpl_cmp.error.eq(    rx_path.rx_smpl_cmp_err),
        ]

        # Logic.
        # ------

        self.comb += [
            self.rx_en.eq(                       self.reg10.fields.rx_en),
            self._ddr2_1_pnf_per_bit_l.status.eq(self._ddr2_1_pnf_per_bit[:16]),
            self._ddr2_1_pnf_per_bit_h.status.eq(self._ddr2_1_pnf_per_bit[15:]),
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        general_periph_files = [
            "gateware/hdl/rxtx_top/synth/rxtx_top.vhd",
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
            "gateware/hdl/tx_path_top/fifo2diq/synth/fifo2diq.vhd",
            "gateware/hdl/tx_path_top/fifo2diq/synth/txiq.vhd",
            "gateware/hdl/tx_path_top/fifo2diq/synth/txiq_ctrl.vhd",
            "gateware/hdl/tx_path_top/bit_unpack/synth/bit_unpack_64.vhd",
            "gateware/hdl/tx_path_top/bit_unpack/synth/unpack_64_to_48.vhd",
            "gateware/hdl/tx_path_top/bit_unpack/synth/unpack_64_to_56.vhd",
            "gateware/hdl/tx_path_top/bit_unpack/synth/unpack_64_to_64.vhd",

            # Lattice FIFOs.
            # --------------
            "gateware/ip/fifodc_w128x256_r128.vhd", # one_pct_fifo.vhd.
            "gateware/ip/fifodc_w128x256_r64.vhd",  # packets2data.vhd.

            "gateware/rxtx_top_wrapper.vhd",
        ]

        for file in general_periph_files:
            platform.add_source(file)

    def do_finalize(self):
        import os
        import subprocess
        def gen_fifo(input_width, output_width, depth, with_buffer=False, with_cdc=False):
            output_dir = self.platform.output_dir
            script_name = os.path.join(os.path.abspath(os.path.dirname(__file__)), "fifo_gen.py")

            filename = f"fifo_w{input_width}x{depth}_r{output_width}"

            cmd = [script_name, "--input-width", str(input_width), "--output-width", str(output_width), "--depth", str(depth), "--build"]
            cmd += ["--output-dir", output_dir]
            if with_cdc:
                cmd.append("--with-cdc")
                filename += "_cdc"
            if with_buffer:
                cmd.append("--with-buffer")
                filename += "_buffer"
            filename += ".v"

            s = subprocess.run(cmd)
            if s.returncode:
                raise OSError(f"Unable to generate FIFO with: " + " ".join(cmd))

            self.platform.add_source(os.path.join(output_dir, filename))


        # LiteX FIFOs.
        # ------------

        # fifo_w64x256_r64_cdc
        # ./fifo_gen --input-width 64 --output-width 64 --depth 256 --with-cdc --build
        gen_fifo(64, 64, 256, with_cdc=True)
        # ./fifo_gen.py --input-width 48 --output-width 48 --depth 1024 --build --with-buffer
        gen_fifo(48, 48, 1024, True)
        # ./fifo_gen.py --input-width 128 --output-width 64 --depth 256 --with-buffer --build
        gen_fifo(128, 64, 256, True)
