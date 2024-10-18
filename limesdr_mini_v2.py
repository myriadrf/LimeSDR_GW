#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use:
# ./limesdr_mini_v2.py --csr-csv=csr.csv --build --load
# litex_server --jtag --jtag-config=openocd_limesdr_mini_v2.cfg
# litex_term crossover

# loading a demo
# ./limesdr_mini_v2.py --integrated-main-ram-size 0x8000 --load --build --uart-name=jtag_uart
# litex_bare_metal_demo --build-path build/limesdr_mini_v2
# litex_term jtag --jtag-config=openocd_limesdr_mini_v2.cfg --kernel demo.bin

import math

from migen import *

from litex.gen import *

import limesdr_mini_v2_platform as limesdr_mini_v2

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litex.soc.cores.led import LedChaser
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.usb_fifo import FT245PHYSynchronous

from litescope import LiteScopeAnalyzer

from gateware.lms7_trx_top   import LMS7TRXTopWrapper
from gateware.ft601          import FT601
from gateware.lms7002_top    import LMS7002Top
from gateware.tst_top        import TstTop
from gateware.general_periph import GeneralPeriphTop
from gateware.rxtx_top       import RXTXTop

# Constants ----------------------------------------------------------------------------------------

FTDI_DQ_WIDTH        = 32    # FTDI Data bus size
CTRL0_FPGA_RX_SIZE   = 1024  # Control PC->FPGA, FIFO size in bytes.
CTRL0_FPGA_RX_RWIDTH = 32    # Control PC->FPGA, FIFO rd width.
CTRL0_FPGA_TX_SIZE   = 1024  # Control FPGA->PC, FIFO size in bytes
CTRL0_FPGA_TX_WWIDTH = 32    # Control FPGA->PC, FIFO wr width
STRM0_FPGA_RX_SIZE   = 4096  # Stream PC->FPGA, FIFO size in bytes
STRM0_FPGA_RX_RWIDTH = 128   # Stream PC->FPGA, rd width
STRM0_FPGA_TX_SIZE   = 16384 # Stream FPGA->PC, FIFO size in bytes
STRM0_FPGA_TX_WWIDTH = 64    # Stream FPGA->PC, wr width

LMS_DIQ_WIDTH        = 12
TX_IN_PCT_HDR_SIZE   = 16
TX_PCT_SIZE          = 4096  # TX packet size in bytes
TX_N_BUFF            = 4     # N 4KB buffers in TX interface (2 OR 4)

C_EP02_RDUSEDW_WIDTH = int(math.ceil(math.log2(CTRL0_FPGA_RX_SIZE / (CTRL0_FPGA_RX_RWIDTH // 8)))) + 1
C_EP82_WRUSEDW_WIDTH = int(math.ceil(math.log2(CTRL0_FPGA_TX_SIZE / (CTRL0_FPGA_TX_WWIDTH // 8)))) + 1
C_EP03_RDUSEDW_WIDTH = int(math.ceil(math.log2(STRM0_FPGA_RX_SIZE / (STRM0_FPGA_RX_RWIDTH // 8)))) + 1
C_EP83_WRUSEDW_WIDTH = int(math.ceil(math.log2(STRM0_FPGA_TX_SIZE / (STRM0_FPGA_TX_WWIDTH // 8)))) + 1

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_usb = ClockDomain()

        # # #

        # Clk.
        lmk_clk = platform.request("LMK_CLK")

        self.comb += self.cd_sys.clk.eq(lmk_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=80e6, toolchain="diamond",
        with_usb_fifo   = True, with_usb_fifo_loopback=False,
        with_led_chaser = True,
        **kwargs):
        platform = limesdr_mini_v2.Platform(toolchain=toolchain)

        ft_clk        = platform.request("FT_CLK")
        lms_pads      = platform.request("LMS")
        revision_pads = platform.request("revision")

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on LimeSDR-Mini-V2", **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # TOP --------------------------------------------------------------------------------------
        self.lms7_trx_top = LMS7TRXTopWrapper(self.platform, lms_pads,
            FTDI_DQ_WIDTH        = FTDI_DQ_WIDTH,
            CTRL0_FPGA_RX_SIZE   = CTRL0_FPGA_RX_SIZE,
            CTRL0_FPGA_RX_RWIDTH = CTRL0_FPGA_RX_RWIDTH,
            CTRL0_FPGA_TX_SIZE   = CTRL0_FPGA_TX_SIZE,
            CTRL0_FPGA_TX_WWIDTH = CTRL0_FPGA_TX_WWIDTH,
            STRM0_FPGA_RX_SIZE   = STRM0_FPGA_RX_SIZE,
            STRM0_FPGA_RX_RWIDTH = STRM0_FPGA_RX_RWIDTH,
            STRM0_FPGA_TX_SIZE   = STRM0_FPGA_TX_SIZE,
            STRM0_FPGA_TX_WWIDTH = STRM0_FPGA_TX_WWIDTH,
            C_EP02_RDUSEDW_WIDTH = C_EP02_RDUSEDW_WIDTH,
            C_EP82_WRUSEDW_WIDTH = C_EP82_WRUSEDW_WIDTH,
            C_EP03_RDUSEDW_WIDTH = C_EP03_RDUSEDW_WIDTH,
            C_EP83_WRUSEDW_WIDTH = C_EP83_WRUSEDW_WIDTH,

        )
        self.comb += [
            self.lms7_trx_top.ft_clk.eq(ft_clk),
            self.lms7_trx_top.HW_VER.eq(revision_pads.HW_VER),
            self.lms7_trx_top.BOM_VER.eq(revision_pads.BOM_VER),
        ]

        # FT601 ------------------------------------------------------------------------------------
        self.ft601 = FT601(self.platform, platform.request("FT"), ft_clk,
            FT_data_width      = FTDI_DQ_WIDTH,
            FT_be_width        = FTDI_DQ_WIDTH // 8,
            EP02_rdusedw_width = C_EP02_RDUSEDW_WIDTH,
            EP02_rwidth        = CTRL0_FPGA_RX_RWIDTH,
            EP82_wrusedw_width = C_EP82_WRUSEDW_WIDTH,
            EP82_wwidth        = CTRL0_FPGA_TX_WWIDTH,
            EP82_wsize         = 64,
            EP03_rdusedw_width = C_EP03_RDUSEDW_WIDTH,
            EP03_rwidth        = STRM0_FPGA_RX_RWIDTH,
            EP83_wrusedw_width = C_EP83_WRUSEDW_WIDTH,
            EP83_wwidth        = STRM0_FPGA_TX_WWIDTH,
            EP83_wsize         = 2048,

        )

        self.comb += [
            self.ft601.reset_n.eq(self.lms7_trx_top.reset_n),
            self.ft601.ctrl_fifo_fpga_pc_reset_n.eq(self.lms7_trx_top.ctrl_fifo_fpga_pc_reset_n),
            self.ft601.stream_fifo_pc_fpga_reset_n.eq(self.lms7_trx_top.stream_fifo_pc_fpga_reset_n),

            self.lms7_trx_top.ctrl_fifo.connect(self.ft601.ctrl_fifo),
        ]

        # LMS7002 Top ------------------------------------------------------------------------------
        self.lms7002_top = LMS7002Top(platform, lms_pads)

        self.comb += [
            self.lms7002_top.reset_n.eq(self.lms7_trx_top.reset_n),
            self.lms7_trx_top.delay_control.connect(self.lms7002_top.delay_control),
        ]

        # Tst Top / Clock Test ---------------------------------------------------------------------
        self.tst_top = TstTop(platform, ft_clk, ClockSignal("sys"))
        self.comb += [
            self.tst_top.reset_n.eq(self.lms7_trx_top.reset_n),
            self.tst_top.test_en.eq(self.lms7_trx_top.test_en),
            self.tst_top.test_frc_err.eq(self.lms7_trx_top.test_frc_err),

            self.lms7_trx_top.test_cmplt.eq(self.tst_top.test_cmplt),
            self.lms7_trx_top.test_rez.eq(self.tst_top.test_rez),

            self.tst_top.Si5351C_clk_0.eq(self.lms7_trx_top.Si5351C_clk_0),
            self.tst_top.Si5351C_clk_1.eq(self.lms7_trx_top.Si5351C_clk_1),
            self.tst_top.Si5351C_clk_2.eq(self.lms7_trx_top.Si5351C_clk_2),
            self.tst_top.Si5351C_clk_3.eq(self.lms7_trx_top.Si5351C_clk_3),
            self.tst_top.Si5351C_clk_5.eq(self.lms7_trx_top.Si5351C_clk_5),
            self.tst_top.Si5351C_clk_6.eq(self.lms7_trx_top.Si5351C_clk_6),
            self.tst_top.Si5351C_clk_7.eq(self.lms7_trx_top.Si5351C_clk_7),
            self.tst_top.adf_muxout.eq(self.lms7_trx_top.adf_muxout),

            self.lms7_trx_top.fx3_clk_cnt.eq(self.tst_top.fx3_clk_cnt),
            self.lms7_trx_top.Si5351C_clk_0_cnt.eq(self.tst_top.Si5351C_clk_0_cnt),
            self.lms7_trx_top.Si5351C_clk_1_cnt.eq(self.tst_top.Si5351C_clk_1_cnt),
            self.lms7_trx_top.Si5351C_clk_2_cnt.eq(self.tst_top.Si5351C_clk_2_cnt),
            self.lms7_trx_top.Si5351C_clk_3_cnt.eq(self.tst_top.Si5351C_clk_3_cnt),
            self.lms7_trx_top.Si5351C_clk_5_cnt.eq(self.tst_top.Si5351C_clk_5_cnt),
            self.lms7_trx_top.Si5351C_clk_6_cnt.eq(self.tst_top.Si5351C_clk_6_cnt),
            self.lms7_trx_top.Si5351C_clk_7_cnt.eq(self.tst_top.Si5351C_clk_7_cnt),
            self.lms7_trx_top.lmk_clk_cnt.eq(self.tst_top.lmk_clk_cnt),
            self.lms7_trx_top.adf_muxout_cnt.eq(self.tst_top.adf_muxout_cnt),
        ]

        # General Periph ---------------------------------------------------------------------------

        self.general_periph = GeneralPeriphTop(platform, "MAX 10")

        self.comb += [
            self.general_periph.reset_n.eq(self.lms7_trx_top.reset_n),
            self.general_periph.HW_VER.eq(revision_pads.HW_VER),

            self.lms7_trx_top.to_periphcfg.eq(self.general_periph.to_periphcfg),
            self.general_periph.from_periphcfg.eq(self.lms7_trx_top.from_periphcfg),

            self.general_periph.led1_mico32_busy.eq(self.lms7_trx_top.led1_mico32_busy),
            self.general_periph.led1_ctrl.eq(self.lms7_trx_top.led1_ctrl),
            self.general_periph.led2_ctrl.eq(self.lms7_trx_top.led2_ctrl),
            self.general_periph.fx3_led_ctrl.eq(self.lms7_trx_top.led3_ctrl),
            self.general_periph.ep03_active.eq(self.ft601.stream_fifo.rd_active),
            self.general_periph.ep83_active.eq(self.ft601.stream_fifo.wr_active),
        ]

        # RXTX Top ---------------------------------------------------------------------------------
        self.rxtx_top = RXTXTop(platform, lms_pads,
            # TX parameters
            TX_IQ_WIDTH            = LMS_DIQ_WIDTH,
            TX_N_BUFF              = TX_N_BUFF,
            TX_IN_PCT_SIZE         = TX_PCT_SIZE,
            TX_IN_PCT_HDR_SIZE     = TX_IN_PCT_HDR_SIZE,
            TX_IN_PCT_DATA_W       = STRM0_FPGA_RX_RWIDTH,
            TX_IN_PCT_RDUSEDW_W    = C_EP03_RDUSEDW_WIDTH,

            # RX parameters
            RX_IQ_WIDTH            = LMS_DIQ_WIDTH,
            RX_INVERT_INPUT_CLOCKS = "ON",
            RX_PCT_BUFF_WRUSEDW_W  = C_EP83_WRUSEDW_WIDTH,
        )

        self.comb += [
            # CPU <-> RXTX Top.
            self.rxtx_top.tx_clk_reset_n.eq(self.lms7_trx_top.reset_n),
            self.rxtx_top.rx_clk_reset_n.eq(self.lms7_trx_top.reset_n),

            self.rxtx_top.from_fpgacfg.eq(self.lms7_trx_top.from_fpgacfg),
            self.lms7_trx_top.to_tstcfg_from_rxtx.eq(self.rxtx_top.to_tstcfg_from_rxtx),
            self.rxtx_top.from_tstcfg.eq(self.lms7_trx_top.from_tstcfg),

            self.rxtx_top.rxtx_smpl_cmp_length.eq(self.lms7_trx_top.rxtx_smpl_cmp_length),

            # LMS7002 <-> RXTX Top.
            self.lms7002_top.tx_diq1_h.eq(self.rxtx_top.tx_diq1_h),
            self.lms7002_top.tx_diq1_l.eq(self.rxtx_top.tx_diq1_l),
            self.rxtx_top.rx_diq2_h.eq(self.lms7002_top.rx_diq2_h),
            self.rxtx_top.rx_diq2_l.eq(self.lms7002_top.rx_diq2_l),
            self.rxtx_top.rx_smpl_cmp.connect(self.lms7002_top.smpl_cmp),

            # FT601 <-> RXTX Top.
            self.ft601.stream_fifo_fpga_pc_reset_n.eq(self.rxtx_top.rx_pct_fifo_aclrn_req),
            self.rxtx_top.stream_fifo.connect(self.ft601.stream_fifo),

            # General Periph <-> RXTX Top.
            self.general_periph.tx_txant_en.eq(self.rxtx_top.tx_txant_en),
        ]


        #eco_config memebr -instance {lms7_trx_top/inst0_cpu/inst_cpu/lm32_inst/ebr/genblk1.ram} -init_all no -mem {/home/gwe/enjoydigital/lime/LimeSDR-Mini-v2_GW/LimeSDR-Mini_lms7_trx/mico32_sw/lms7_trx/lms7_trx.mem} -format hex -init_data static -module {pmi_ram_dpEhnonessen3213819232138192p13822039} -mode {RAM_DP} -depth {8192} -widtha {32} -widthb {32}

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=limesdr_mini_v2.Platform, description="LiteX SoC on LimeSDR-Mini-V2.")
    parser.add_target_argument("--sys-clk-freq", default=40e6, type=float, help="System clock frequency.")
    args = parser.parse_args()
    args.no_uart   = True
    args.cpu_type  = "None"
    args.toolchain = "diamond"

    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        toolchain    = args.toolchain,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".svf")) # FIXME

if __name__ == "__main__":
    main()
