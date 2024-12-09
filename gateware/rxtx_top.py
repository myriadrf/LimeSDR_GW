#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess

from migen import *

from litex.gen import *

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect.csr            import CSRStatus, CSRStorage, CSRField

from gateware.common                                    import *
from gateware.LimeDFB_LiteX.rx_path_top.src.rx_path_top import RXPathTop
from gateware.LimeDFB_LiteX.tx_path_top.src.tx_path_top import TXPathTop

# RXTX Top -----------------------------------------------------------------------------------------

class RXTXTop(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
        # TX parameters
        TX_IQ_WIDTH            = 12,
        TX_N_BUFF              = 4,
        TX_IN_PCT_SIZE         = 4096,
        TX_IN_PCT_HDR_SIZE     = 16,
        TX_IN_PCT_DATA_W       = 128,
        TX_OUT_PCT_DATA_W      = 64,
        tx_s_clk_domain        = "lms_tx",
        tx_m_clk_domain        = "lms_tx",

        # RX parameters
        RX_IQ_WIDTH            = 12,
        rx_s_clk_domain        = "lms_rx",
        rx_int_clk_domain      = "lms_rx",
        rx_m_clk_domain        = "lms_rx",
        ):

        assert fpgacfg_manager is not None

        self.platform              = platform

        self.rx_pct_fifo_aclrn_req = Signal()
        self.rx_en                 = Signal()

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

        # TX Path. ---------------------------------------------------------------------------------
        self.tx_path = tx_path = TXPathTop(platform, fpgacfg_manager,
            # TX parameters
            IQ_WIDTH      = TX_IQ_WIDTH,
            PCT_MAX_SIZE  = TX_IN_PCT_SIZE,
            PCT_HDR_SIZE  = TX_IN_PCT_HDR_SIZE,
            BUFF_COUNT    = TX_N_BUFF,
            FIFO_DATA_W   = TX_IN_PCT_DATA_W,
            s_clk_domain  = tx_s_clk_domain,
            m_clk_domain  = tx_m_clk_domain,
            rx_clk_domain = rx_s_clk_domain, # FIXME: unsure
        )

        # RX Path. ---------------------------------------------------------------------------------
        self.rx_path = rx_path = RXPathTop(platform, fpgacfg_manager,
            ## RX parameters
            RX_IQ_WIDTH            = RX_IQ_WIDTH,
            m_clk_domain           = rx_m_clk_domain,
            int_clk_domain         = rx_int_clk_domain,
            s_clk_domain           = rx_s_clk_domain,
        )

        # Logic.
        # ------

        self.comb += [
            self.rx_en.eq(                       fpgacfg_manager.rx_en),

            # CSR
            self._ddr2_1_pnf_per_bit_l.status.eq(self._ddr2_1_pnf_per_bit[:16]),
            self._ddr2_1_pnf_per_bit_h.status.eq(self._ddr2_1_pnf_per_bit[15:]),

            # RX <-> TX
            tx_path.pct_loss_flg_clr.eq( rx_path.pct_hdr_cap),
            tx_path.rx_sample_nr.eq(     rx_path.smpl_nr_cnt),
            rx_path.tx_pct_loss_flg.eq(  tx_path.pct_loss_flg),

            # RX -> FIFO
            self.rx_pct_fifo_aclrn_req.eq(rx_path.rx_pct_fifo_aclrn_req),
        ]
