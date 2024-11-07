#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import CSRStatus

from gateware.common      import *
from gateware.lms7002_top import SampleCompare

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

        self.tx_txant_en           = Signal()
        self.tx_diq1_h             = Signal(TX_IQ_WIDTH + 1)
        self.tx_diq1_l             = Signal(TX_IQ_WIDTH + 1)

        self.rx_diq2_h             = Signal(RX_IQ_WIDTH + 1)
        self.rx_diq2_l             = Signal(RX_IQ_WIDTH + 1)
        self.rx_pct_fifo_aclrn_req = Signal()
        self.rx_smpl_cmp           = SampleCompare()
        self.rxtx_smpl_cmp_length  = Signal(16)

        self.from_fpgacfg             = FromFPGACfg()
        self.from_tstcfg_TEST_EN      = Signal(6)
        self.from_tstcfg_TEST_FRC_ERR = Signal(6)
        self.from_tstcfg_TX_TST_I     = Signal(16)
        self.from_tstcfg_TX_TST_Q     = Signal(16)

        self.stream_fifo           = FIFOInterface(TX_IN_PCT_DATA_W, 64, TX_IN_PCT_RDUSEDW_W, RX_PCT_BUFF_WRUSEDW_W)

        # # #

        # Signals.
        # --------
        self._DDR2_1_STATUS      = Signal(3)
        self._DDR2_1_pnf_per_bit = Signal(32)

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

            ## RX parameters
            p_RX_IQ_WIDTH            = RX_IQ_WIDTH,
            #p_RX_INVERT_INPUT_CLOCKS = RX_INVERT_INPUT_CLOCKS,
            p_RX_SMPL_BUFF_RDUSEDW_W = RX_SMPL_BUFF_RDUSEDW_W,
            p_RX_PCT_BUFF_WRUSEDW_W  = RX_PCT_BUFF_WRUSEDW_W,

            # Configuration memory ports
            #from_fpgacfg            : in     t_FROM_FPGACFG;
            i_phase_reg_sel          = self.from_fpgacfg.phase_reg_sel,
            i_clk_ind                = self.from_fpgacfg.clk_ind,
            i_cnt_ind                = self.from_fpgacfg.cnt_ind,
            i_load_phase_reg         = self.from_fpgacfg.load_phase_reg,
            i_drct_clk_en            = self.from_fpgacfg.drct_clk_en,
            i_ch_en                  = self.from_fpgacfg.ch_en,
            i_smpl_width             = self.from_fpgacfg.smpl_width,
            i_mode                   = self.from_fpgacfg.mode,
            i_ddr_en                 = self.from_fpgacfg.ddr_en,
            i_trxiq_pulse            = self.from_fpgacfg.trxiq_pulse,
            i_mimo_int_en            = self.from_fpgacfg.mimo_int_en,
            i_synch_dis              = self.from_fpgacfg.synch_dis,
            i_synch_mode             = self.from_fpgacfg.synch_mode,
            i_smpl_nr_clr            = self.from_fpgacfg.smpl_nr_clr,
            i_txpct_loss_clr         = self.from_fpgacfg.txpct_loss_clr,
            i_rx_en                  = self.from_fpgacfg.rx_en,
            i_tx_en                  = self.from_fpgacfg.tx_en,
            i_rx_ptrn_en             = self.from_fpgacfg.rx_ptrn_en,
            i_tx_ptrn_en             = self.from_fpgacfg.tx_ptrn_en,
            i_tx_cnt_en              = self.from_fpgacfg.tx_cnt_en,
            i_wfm_ch_en              = self.from_fpgacfg.wfm_ch_en,
            i_wfm_play               = self.from_fpgacfg.wfm_play,
            i_wfm_load               = self.from_fpgacfg.wfm_load,
            i_wfm_smpl_width         = self.from_fpgacfg.wfm_smpl_width,
            i_SPI_SS                 = self.from_fpgacfg.SPI_SS,
            i_LMS1_SS                = self.from_fpgacfg.LMS1_SS,
            i_LMS1_RESET             = self.from_fpgacfg.LMS1_RESET,
            i_LMS1_CORE_LDO_EN       = self.from_fpgacfg.LMS1_CORE_LDO_EN,
            i_LMS1_TXNRX1            = self.from_fpgacfg.LMS1_TXNRX1,
            i_LMS1_TXNRX2            = self.from_fpgacfg.LMS1_TXNRX2,
            i_LMS1_TXEN              = self.from_fpgacfg.LMS1_TXEN,
            i_LMS1_RXEN              = self.from_fpgacfg.LMS1_RXEN,
            i_GPIO                   = self.from_fpgacfg.GPIO,
            i_FPGA_LED1_CTRL         = self.from_fpgacfg.FPGA_LED1_CTRL,
            i_FPGA_LED2_CTRL         = self.from_fpgacfg.FPGA_LED2_CTRL,
            i_FX3_LED_CTRL           = self.from_fpgacfg.FX3_LED_CTRL,
            i_CLK_ENA                = self.from_fpgacfg.CLK_ENA,
            i_sync_pulse_period      = self.from_fpgacfg.sync_pulse_period,
            i_sync_size              = self.from_fpgacfg.sync_size,
            i_txant_pre              = self.from_fpgacfg.txant_pre,
            i_txant_post             = self.from_fpgacfg.txant_post,

            ##to_tstcfg_from_rxtx     : out    t_TO_TSTCFG_FROM_RXTX;
            o_DDR2_1_STATUS          = self._DDR2_1_STATUS,
            o_DDR2_1_pnf_per_bit     = self._DDR2_1_pnf_per_bit,
            #from_tstcfg             : in     t_FROM_TSTCFG;
            i_TEST_EN                = self.from_tstcfg_TEST_EN,
            i_TEST_FRC_ERR           = self.from_tstcfg_TEST_FRC_ERR,
            i_TX_TST_I               = self.from_tstcfg_TX_TST_I,
            i_TX_TST_Q               = self.from_tstcfg_TX_TST_Q,

            ## TX path
            i_tx_clk                 = ClockSignal("lms_tx"),
            o_tx_clkout              = Open(),
            i_tx_clk_reset_n         = ~ResetSignal("sys"),
            o_tx_pct_loss_flg        = Open(),
            o_tx_txant_en            = self.tx_txant_en,
            #  Tx interface data
            o_tx_diq1_h              = self.tx_diq1_h,
            o_tx_diq1_l              = self.tx_diq1_l,
            #  TX FIFO read ports
            o_tx_in_pct_rdreq        = self.stream_fifo.rd,
            i_tx_in_pct_data         = self.stream_fifo.rdata,
            i_tx_in_pct_rdempty      = self.stream_fifo.empty,
            i_tx_in_pct_rdusedw      = self.stream_fifo.rdusedw,

            ## RX path
            i_rx_clk                 = ClockSignal("lms_rx"),
            i_rx_clk_reset_n         = ~ResetSignal("sys"),
            ##  Rx interface data
            i_rx_diq2_h              = self.rx_diq2_h,
            i_rx_diq2_l              = self.rx_diq2_l,
            #  Packet fifo ports
            o_rx_pct_fifo_aclrn_req  = self.rx_pct_fifo_aclrn_req,
            i_rx_pct_fifo_wusedw     = self.stream_fifo.wrusedw,
            o_rx_pct_fifo_wrreq      = self.stream_fifo.wr,
            o_rx_pct_fifo_wdata      = self.stream_fifo.wdata,
            #  sample compare
            i_rx_smpl_cmp_start      = self.rx_smpl_cmp.en,
            i_rx_smpl_cmp_length     = self.rxtx_smpl_cmp_length,
            o_rx_smpl_cmp_done       = self.rx_smpl_cmp.done,
            o_rx_smpl_cmp_err        = self.rx_smpl_cmp.error,
        )

        self.add_csr()

        self.add_sources(platform)

    def add_csr(self):

        # testcfg_from_rxtx @22
        self._ddr2_1_status        = CSRStatus(3)
        # testcfg_from_rxtx @23
        self._ddr2_1_pnf_per_bit_l = CSRStatus(16)
        # testcfg_from_rxtx @24
        self._ddr2_1_pnf_per_bit_h = CSRStatus(16)

        self.comb += [
            self._ddr2_1_status.status.eq(       self._DDR2_1_STATUS),
            self._ddr2_1_pnf_per_bit_l.status.eq(self._DDR2_1_pnf_per_bit[:16]),
            self._ddr2_1_pnf_per_bit_h.status.eq(self._DDR2_1_pnf_per_bit[15:]),
        ]

    def add_sources(self, platform):
        general_periph_files = [
            "LimeSDR-Mini_lms7_trx/src/rxtx_top/synth/rxtx_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/tx_path/synth/sync_fifo_rw.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/tx_path/synth/tx_path_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/pulse_gen/synth/pulse_gen.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/pct_separate/synth/one_pct_fifo.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/pct_separate/synth/pct_separate_fsm.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/packets2data/synth/p2d_rd.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/packets2data/synth/p2d_rd_fsm.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/packets2data/synth/p2d_sync_fsm.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/packets2data/synth/p2d_wr_fsm.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/packets2data/synth/packets2data.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/packets2data/synth/packets2data_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/handshake_sync/synth/handshake_sync.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/fifo2diq/synth/fifo2diq.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/fifo2diq/synth/txiq.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/fifo2diq/synth/txiq_ctrl.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/bit_unpack/synth/bit_unpack_64.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/bit_unpack/synth/unpack_64_to_48.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/bit_unpack/synth/unpack_64_to_56.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/bit_unpack/synth/unpack_64_to_64.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/bit_pack/synth/bit_pack.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/bit_pack/synth/pack_48_to_64.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/bit_pack/synth/pack_56_to_64.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/smpl_cmp/synth/smpl_cmp.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/rx_path/synth/rx_path_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/diq2fifo.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq_mimo.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq_mimo_ddr.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq_pulse_ddr.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq_siso.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq_siso_ddr.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/rxiq_siso_sdr.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/diq2fifo/synth/test_data_dd.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/data2packets/synth/data2packets.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/data2packets/synth/data2packets_fsm.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/data2packets/synth/data2packets_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/tx_path_top/fifo2diq/synth/edge_delay.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/smpl_cnt/synth/iq_smpl_cnt.vhd",
            "LimeSDR-Mini_lms7_trx/src/rx_path_top/smpl_cnt/synth/smpl_cnt.vhd",

            # Lattice FIFOs.
            # --------------
            "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w48x1024_r48/fifodc_w48x1024_r48.vhd",
            "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w128x256_r128/fifodc_w128x256_r128.vhd", # one_pct_fifo.vhd.
            "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w128x256_r64/fifodc_w128x256_r64.vhd",   # packets2data.vhd.

            # ./fifo_gen --input-width 64 --output-width 64 --depth 256 --build
            "gateware/fifo_w64x256_r64.v",
            # ./fifo_gen --input-width 48 --output-width 48 --depth 1024 --build
            "gateware/fifo_w48x1024_r48.v",
            # ./fifo_gen.py --input-width 128 --output-width 64 --depth 256 --build
            "gateware/fifo_w128x256_r64.v",

            "gateware/rxtx_top_wrapper.vhd",
        ]

        for file in general_periph_files:
            platform.add_source(file)
