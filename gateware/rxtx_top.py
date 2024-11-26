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

from gateware.common              import *
from gateware.rx_path             import RXPath
from gateware.tx_path             import TXPath

# RXTX Top -----------------------------------------------------------------------------------------

class RXTXTop(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
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

        assert fpgacfg_manager is not None

        self.platform              = platform

        self.stream_fifo           = FIFOInterface(TX_IN_PCT_DATA_W, 64, TX_IN_PCT_RDUSEDW_W, RX_PCT_BUFF_WRUSEDW_W)

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
        self.tx_path = tx_path = TXPath(platform, fpgacfg_manager,
            # TX parameters
            IQ_WIDTH     = TX_IQ_WIDTH,
            PCT_MAX_SIZE = TX_IN_PCT_SIZE,
            PCT_HDR_SIZE = TX_IN_PCT_HDR_SIZE,
            BUFF_COUNT   = TX_N_BUFF,
            FIFO_DATA_W  = TX_IN_PCT_DATA_W,
        )

        # RX Path. ---------------------------------------------------------------------------------
        self.rx_path = rx_path = RXPath(platform, fpgacfg_manager,
            ## RX parameters
            RX_IQ_WIDTH            = RX_IQ_WIDTH,
            RX_SMPL_BUFF_RDUSEDW_W = RX_SMPL_BUFF_RDUSEDW_W,
            RX_PCT_BUFF_WRUSEDW_W  = RX_PCT_BUFF_WRUSEDW_W,
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

            # FIFO -> TX
            self.stream_fifo.rd.eq(      tx_path.stream_fifo_rd),
            tx_path.stream_fifo_data.eq( self.stream_fifo.rdata),
            tx_path.stream_fifo_empty.eq(self.stream_fifo.empty),

            # RX -> FIFO
            self.rx_pct_fifo_aclrn_req.eq(rx_path.rx_pct_fifo_aclrn_req),
            rx_path.rx_pct_fifo_wusedw.eq(self.stream_fifo.wrusedw),
            rx_path.rx_pct_fifo_ready.eq( ~self.stream_fifo.full),
            self.stream_fifo.wr.eq(       rx_path.rx_pct_fifo_wrreq),
            self.stream_fifo.wdata.eq(    rx_path.rx_pct_fifo_wdata),
        ]

    def do_finalize(self):
        def gen_fifo(vendor, input_width, output_width, depth, with_buffer=False, with_cdc=False, disable_rd_delay=False):
            output_dir  = os.path.join(self.platform.output_dir, "fifo")
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
            if disable_rd_delay:
                cmd.append("--disable-rd-delay")
            cmd.append(f"--vendor={vendor}")
            filename += ".v"

            s = subprocess.run(cmd)
            if s.returncode:
                raise OSError(f"Unable to generate FIFO with: " + " ".join(cmd))

            self.platform.add_source(os.path.join(output_dir, filename))


        # LiteX FIFOs.
        # ------------
        vendor = {
            "limesdr_mini_v1" : "altera",
            "limesdr_mini_v2" : "lattice",
        }[self.platform.name]


        # Generate fifo_w64x256_r64_cdc.v
        gen_fifo(vendor=vendor, input_width= 64,  output_width= 64, depth=256,  with_cdc=True)

        # Generate fifo_w64x1024_r64_buffer.v
        gen_fifo(vendor=vendor, input_width= 64,  output_width= 64, depth=1024, with_buffer=True)

        # Generate fifo_w128x128_r256_buffer.v
        gen_fifo(vendor=vendor, input_width=128, output_width=128,  depth=256,  with_buffer=True, disable_rd_delay=True)

        # Generate fifo_w128x64_r256_buffer.v
        gen_fifo(vendor=vendor, input_width=128, output_width= 64,  depth=256,  with_buffer=True, disable_rd_delay=True)

        # Generate fifo_w128x512_r128_buffer.v
        gen_fifo(vendor=vendor, input_width=128, output_width=128,  depth=512,  with_buffer=True, disable_rd_delay=True)
