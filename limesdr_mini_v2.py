#!/usr/bin/env python3

#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

# Build/Use:
# ./limesdr_mini_v2.py --csr-csv=csr.csv --build --load
# litex_server --jtag --jtag-config=openocd_limesdr_mini_v2.cfg

import math

from migen import *

from litex.gen import *

import limesdr_mini_v2_platform as limesdr_mini_v2

from litex.build.generic_platform import Subsignal, IOStandard, Pins

from litex.soc.interconnect         import stream
from litex.soc.interconnect.csr     import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.bitbang        import I2CMaster
from litex.soc.cores.spi.spi_master import SPIMaster

from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP

from litespi.phy.generic import LiteSPIPHY

from litescope import LiteScopeAnalyzer

from gateware.busy_delay       import BusyDelay
from gateware.lms7_trx_top     import LMS7TRXTopWrapper
from gateware.fpgacfg          import FPGACfg
from gateware.ft601            import FT601
from gateware.lms7002_top      import LMS7002Top
from gateware.tst_top          import TstTop
from gateware.general_periph   import GeneralPeriphTop
from gateware.pllcfg           import PLLCfg
from gateware.rxtx_top         import RXTXTop
from gateware.fifo_ctrl_to_csr import FIFOCtrlToCSR

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
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_por   = ClockDomain()
        self.cd_ft601 = ClockDomain()

        # # #

        # Clk.
        self.ft_clk = platform.request("FT_CLK")

        # Power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # Internal Oscilator
        # DIV values: 2(~155MHz) - 128(~2.4MHz)
        self.specials += Instance("OSCG",
            p_DIV = 4,
            o_OSC = self.cd_sys.clk,
        )

        self.comb += self.cd_sys.rst.eq(~por_done)

        platform.add_platform_command("GSR_NET NET main_por_done;")

        # FT601 Clk/Rst
        self.comb += [
            self.cd_ft601.clk.eq(self.ft_clk),
            self.cd_ft601.rst.eq(~por_done),
        ]

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=80e6, toolchain="diamond",
        with_litescope  = False,
        cpu_firmware    = None,
        **kwargs):

        # Platform ---------------------------------------------------------------------------------
        platform      = limesdr_mini_v2.Platform(toolchain=toolchain)
        platform.name = "limesdr_mini_v2"

        lms_pads      = platform.request("LMS")
        revision_pads = platform.request("revision")
        rfsw_pads     = platform.request("RFSW")
        tx_lb_pads    = platform.request("TX_LB")
        gpio_pads     = platform.request("FPGA_GPIO")
        #egpio_pads    = platform.request("FPGA_EGPIO")
        platform.add_extension([
            ("serial", 0,
                Subsignal("tx", Pins("A10")),
                Subsignal("rx", Pins("A8")),
                IOStandard("LVCMOS33")
            )
        ])

        # SoCCore ----------------------------------------------------------------------------------
        uart_name     = {True: "crossover", False:"serial"}[with_litescope]
        with_uartbone = with_litescope
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-Mini-V2",
            ident_version            = True,
            cpu_type                 = "picorv32", # FIXME: Switch to VexRiscv when working with Diamond.
            cpu_variant              = "standard",
            integrated_rom_size      = 0x8000,
            integrated_sram_ram_size = 0x1000,
            integrated_main_ram_size = 0x4000,
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little"),
            with_uartbone            = with_uartbone,
            uart_name                = uart_name,
        )

        # Avoid stalling CPU at startup.
        self.uart.add_auto_tx_flush(sys_clk_freq=sys_clk_freq, timeout=1, interval=128)

        # Automatically jump to pre-initialized firmware.
        self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["main_ram"])

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # I2C Bus0 (LM75 & EEPROM) -----------------------------------------------------------------
        self.i2c0 = I2CMaster(pads=platform.request("FPGA_I2C", 0))

        # SPI (LMS7002 & DAC) ----------------------------------------------------------------------
        self.add_spi_master(name="spimaster", pads=platform.request("FPGA_SPI", 0), data_width=32, spi_clk_freq=10e6)
        # SPI Flash --------------------------------------------------------------------------------
        from litespi.modules import W25Q128JV
        from litespi.opcodes import SpiNorFlashOpCodes as Codes

        self.add_spi_flash("FPGA_CFG_SPI", mode="1x", module=W25Q128JV(Codes.READ_1_1_1), with_master=False)

        # SPI (CFG_TOP) ----------------------------------------------------------------------------
        cfg_top_spi = Record(SPIMaster.pads_layout)
        self.add_spi_master(name="cfg_top", pads=cfg_top_spi, data_width=32, spi_clk_freq=sys_clk_freq)

        # mico32_busy(gpo) & busy_delay ------------------------------------------------------------
        self._gpo = CSRStorage(description="GPO interface", fields=[
            CSRField("mico32_busy", size=1, offset=0, description="CPU state.", values=[
                ("``0b0``", "IDLE."),
                ("``0b1``", "BUSY."),
            ])
        ])

        self.busy_delay  = BusyDelay(platform, "sys", 25, 100)
        self.comb       += self.busy_delay.busy_in.eq(self._gpo.fields.mico32_busy)

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
            self.lms7_trx_top.ft_clk.eq(self.crg.ft_clk),

            # CFG_TOP SPI Interface.
            self.lms7_trx_top.cfg_top_mosi.eq(cfg_top_spi.mosi),
            self.lms7_trx_top.cfg_top_sclk.eq(cfg_top_spi.clk),
            self.lms7_trx_top.cfg_top_ss_n.eq(cfg_top_spi.cs_n),
            cfg_top_spi.miso.eq(self.lms7_trx_top.cfg_top_miso),
        ]

        # FPGA Cfg ---------------------------------------------------------------------------------
        self.fpgacfg = FPGACfg(revision_pads)
        self.comb += self.fpgacfg.pwr_src.eq(0)

        # PLL Cfg ----------------------------------------------------------------------------------
        self.pllcfg = PLLCfg()
        self.comb += self.fpgacfg.pwr_src.eq(0)

        # FIFO Control -----------------------------------------------------------------------------
        self.fifo_ctrl = FIFOCtrlToCSR(CTRL0_FPGA_RX_RWIDTH, CTRL0_FPGA_TX_WWIDTH)

        # FT601 ------------------------------------------------------------------------------------
        self.ft601 = FT601(self.platform, platform.request("FT"),
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
            self.ft601.ctrl_fifo_fpga_pc_reset_n.eq(~self.fifo_ctrl.fifo_reset),
            self.ft601.stream_fifo_pc_fpga_reset_n.eq(self.fpgacfg.from_fpgacfg.rx_en),
            self.fifo_ctrl.ctrl_fifo.connect(self.ft601.ctrl_fifo),
        ]

        # LMS7002 Top ------------------------------------------------------------------------------
        self.lms7002_top = LMS7002Top(platform, lms_pads, revision_pads.HW_VER)

        # Tst Top / Clock Test ---------------------------------------------------------------------
        self.tst_top = TstTop(platform, self.crg.ft_clk, platform.request("LMK_CLK"))
        self.comb += [
            self.tst_top.Si5351C_clk_0.eq(0),
            self.tst_top.Si5351C_clk_1.eq(0),
            self.tst_top.Si5351C_clk_2.eq(0),
            self.tst_top.Si5351C_clk_3.eq(0),
            self.tst_top.Si5351C_clk_5.eq(0),
            self.tst_top.Si5351C_clk_6.eq(0),
            self.tst_top.Si5351C_clk_7.eq(0),
            self.tst_top.adf_muxout.eq(0),
        ]

        # General Periph ---------------------------------------------------------------------------
        self.general_periph = GeneralPeriphTop(platform, "MAX 10",
            revision_pads = revision_pads,
            gpio_pads     = gpio_pads,
            gpio_len      = len(gpio_pads),
            egpio_pads    = None,
            egpio_len     = 2,
        )

        self.comb += [
            self.general_periph.led1_mico32_busy.eq(self.busy_delay.busy_out),
            self.general_periph.ep03_active.eq(self.ft601.stream_fifo.rd_active),
            self.general_periph.ep83_active.eq(self.ft601.stream_fifo.wr_active),
        ]

        # RXTX Top ---------------------------------------------------------------------------------
        self.rxtx_top = RXTXTop(platform,
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
            self.rxtx_top.from_fpgacfg.eq(self.fpgacfg.from_fpgacfg),
            self.rxtx_top.from_tstcfg_TEST_EN.eq(self.tst_top.test_en),
            self.rxtx_top.from_tstcfg_TEST_FRC_ERR.eq(self.tst_top.test_frc_err),
            self.rxtx_top.from_tstcfg_TX_TST_I.eq(self.tst_top.tx_tst_i),
            self.rxtx_top.from_tstcfg_TX_TST_Q.eq(self.tst_top.tx_tst_q),

            self.rxtx_top.rxtx_smpl_cmp_length.eq(self.pllcfg.auto_phcfg_smpls),

            # LMS7002 <-> RXTX Top.
            self.lms7002_top.tx_diq1_h.eq(self.rxtx_top.tx_diq1_h),
            self.lms7002_top.tx_diq1_l.eq(self.rxtx_top.tx_diq1_l),
            self.rxtx_top.rx_diq2_h.eq(self.lms7002_top.rx_diq2_h),
            self.rxtx_top.rx_diq2_l.eq(self.lms7002_top.rx_diq2_l),
            self.lms7002_top.smpl_cmp.connect(self.rxtx_top.rx_smpl_cmp),

            # FT601 <-> RXTX Top.
            self.ft601.stream_fifo_fpga_pc_reset_n.eq(self.rxtx_top.rx_pct_fifo_aclrn_req),
            self.rxtx_top.stream_fifo.connect(self.ft601.stream_fifo),

            # General Periph <-> RXTX Top.
            self.general_periph.tx_txant_en.eq(self.rxtx_top.tx_txant_en),

            # General Periph <-> LMS7002
            self.lms7002_top.PERIPH_OUTPUT_VAL_1.eq(self.general_periph.PERIPH_OUTPUT_VAL_1),
        ]

        # RF Switches ------------------------------------------------------------------------------
        self.comb += [
            # RF Switch.
            rfsw_pads.RX_V1.eq(self.fpgacfg.from_fpgacfg.GPIO[8]),
            rfsw_pads.RX_V2.eq(self.fpgacfg.from_fpgacfg.GPIO[9]),
            rfsw_pads.TX_V1.eq(self.fpgacfg.from_fpgacfg.GPIO[12]),
            rfsw_pads.TX_V2.eq(self.fpgacfg.from_fpgacfg.GPIO[13]),

            # TX
            tx_lb_pads.AT.eq(  self.fpgacfg.from_fpgacfg.GPIO[1]),
            tx_lb_pads.SH.eq(  self.fpgacfg.from_fpgacfg.GPIO[2]),
        ]

        # Timings ----------------------------------------------------------------------------------
        self.platform.add_sdc("LimeSDR-Mini_lms7_trx/proj/FT601_timing.sdc")
        self.platform.add_sdc("LimeSDR-Mini_lms7_trx/proj/LMS7002_timing.sdc")
        self.platform.add_sdc("gateware/timing.sdc")

        # Analyzer ---------------------------------------------------------------------------------
        if with_litescope:
            analyzer_signals = [
                self.fifo_ctrl.ctrl_fifo.rd,
                self.fifo_ctrl.ctrl_fifo.rdata,
                self.fifo_ctrl.ctrl_fifo.empty,
                self.fifo_ctrl.ctrl_fifo.wr,
                self.fifo_ctrl.ctrl_fifo.wdata,
                self.fifo_ctrl.ctrl_fifo.full,
                self.fifo_ctrl.fifo_reset,

                self.ft601.ctrl_fifo.rd,
                self.ft601.ctrl_fifo.rdata,
                self.ft601.ctrl_fifo.empty,
                self.ft601.ctrl_fifo.wr,
                self.ft601.ctrl_fifo.wdata,
                self.ft601.ctrl_fifo.full,
                self.ft601.ctrl_fifo_fpga_pc_reset_n,
            ]
            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 1024,
                clock_domain = "sys",
                register     = True,
                csr_csv      = "analyzer.csv"
            )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=limesdr_mini_v2.Platform, description="LiteX SoC on LimeSDR-Mini-V2.")
    parser.add_target_argument("--sys-clk-freq",   default=77.5e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-litescope", action="store_true",        help="Enable LiteScope.")
    args = parser.parse_args()
    args.toolchain = "diamond"

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        soc = BaseSoC(
            sys_clk_freq   = args.sys_clk_freq,
            toolchain      = args.toolchain,
            with_litescope = args.with_litescope,
            cpu_firmware   = None if prepare else "firmware/firmware.bin",
            **parser.soc_argdict
        )
        builder = Builder(soc, csr_csv="csr.csv")
        builder.build(run=build)
        if prepare:
            os.system(f"cd firmware && make BUILD_DIR={builder.output_dir} clean all")

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".svf")) # FIXME

if __name__ == "__main__":
    main()
