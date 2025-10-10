#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import math
from shutil import which

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from boards.platforms import limesdr_mini_v2_platform as limesdr_mini_v2

from litex.soc.interconnect         import stream
from litex.soc.interconnect.csr     import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock          import ECP5PLL
from litex.soc.cores.bitbang        import I2CMaster
from litex.soc.cores.spi.spi_master import SPIMaster

from litespi.phy.generic import LiteSPIPHY

from litescope import LiteScopeAnalyzer

from gateware.LimeTop                       import LimeTop

from gateware.LimeDFB_LiteX.FT601.src.ft601 import FT601
from gateware.Revision import *
from gateware.helpers import write_module_hierarchy_json

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
    def __init__(self, platform, sys_clk_freq, use_pll=False):
        self.rst      = Signal()
        self.cd_por   = ClockDomain()
        self.cd_sys   = ClockDomain()
        self.cd_ft601 = ClockDomain()
        self.cd_lmk  = ClockDomain()

        # # #

        # Clk.
        self.ft_clk  = platform.request("FT_CLK")
        self.lmk_clk = platform.request("LMK_CLK")

        # Power-on-Clk/Rst.
        por_count = Signal(4)
        por_done  = Signal()
        por_count.attr.add("keep")
        por_done.attr.add("keep")
        self.specials += Instance("OSCG",
            # DIV values: 2: ~155MHz to 128: ~2.4MHz.
            p_DIV = 4, # ~77.5MHz.
            o_OSC = self.cd_por.clk,
        )
        self.comb += por_done.eq(Reduce("AND", por_count))
        self.sync.por += por_count.eq(Cat(Constant(1, 1), por_count[0:3]))
        platform.add_platform_command("GSR_NET NET crg_por_done;")

        # Sys Clk/Rst.
        if use_pll:
            self.pll = pll = ECP5PLL()
            self.comb += pll.reset.eq(self.rst | ~por_done)
            pll.register_clkin(self.ft_clk, 100e6)
            pll.create_clkout(self.cd_sys, sys_clk_freq)
        else:
            self.comb += self.cd_sys.clk.eq(self.cd_por.clk)
            self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done)

        # FT601 Clk/Rst.
        self.comb     += self.cd_ft601.clk.eq(self.ft_clk),
        self.specials += AsyncResetSynchronizer(self.cd_ft601, ~por_done)

        # LMK_CLK
        self.comb += self.cd_lmk.clk.eq(self.lmk_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):

    def add_uartbone_mod(self, name="uartbone", clk_freq=None, baudrate=115200, cd="sys", with_dynamic_baudrate=False, uart_pads=None):
        # Imports.
        from litex.soc.cores import uart

        # Core.
        if clk_freq is None:
            clk_freq = self.sys_clk_freq
        self.check_if_exists(name)
        uartbone_phy = uart.UARTPHY(uart_pads, clk_freq, baudrate, with_dynamic_baudrate=with_dynamic_baudrate)
        uartbone     = uart.UARTBone(
            phy           = uartbone_phy,
            clk_freq      = clk_freq,
            cd            = cd,
            address_width = self.bus.address_width)
        self.add_module(name=f"{name}_phy", module=uartbone_phy)
        self.add_module(name=name,          module=uartbone)
        self.bus.add_master(name=name, master=uartbone.wishbone)
    def __init__(self, sys_clk_freq=77.5e6, cpu_type="vexriscv", toolchain="trellis",
        with_bios      = False,
        with_rx_tx_top = True,
        with_uartbone  = False,
        with_spi_flash = False,
        cpu_firmware   = None,
        with_fft       = False,
        **kwargs):

        # Assert Diamond Limitations ---------------------------------------------------------------

        if toolchain == "diamond":
            # CPU.
            if cpu_type == "vexriscv":
                raise ValueError("VexRiscv is not supported with the Diamond toolchain (HDL implementation issues).")
            # SPI Flash.
            if with_spi_flash:
                raise ValueError("SPI Flash is not supported with the Diamond toolchain (HDL implementation issues).")

        # Platform ---------------------------------------------------------------------------------
        platform             = limesdr_mini_v2.Platform(toolchain=toolchain)
        platform.name        = "limesdr_mini_v2"
        platform.vhd2v_force = True
        if toolchain == "diamond":
            platform.toolchain.additional_ldf_commands += ["prj_strgy set_value -strategy Strategy1 syn_vhdl2008=True"] # Enable VHDL-2008 support.

        # SoCCore ----------------------------------------------------------------------------------
        assert cpu_type in ["vexriscv", "picorv32", "fazyrv", "firev"]
        cpu_variant = {
            "vexriscv" : "minimal",
            "picorv32" : "minimal",
            "fazyrv"   : "standard",
            "firev"    : "standard",
        }[cpu_type]

        if with_bios:
            integrated_rom_size      = 0x9800
            integrated_rom_init      = []
            integrated_main_ram_size = 0x4800
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
        else:
            integrated_rom_size      = 0x4200
            integrated_rom_init      = [0] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_main_ram_size = 0
            integrated_main_ram_init = []

        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-Mini-V2",
            ident_version            = True,
            cpu_type                 = cpu_type,
            cpu_variant              = cpu_variant,
            integrated_rom_size      = integrated_rom_size,
            integrated_rom_init      = integrated_rom_init,
            integrated_sram_ram_size = 0x0200,
            integrated_main_ram_size = integrated_main_ram_size,
            integrated_main_ram_init = integrated_main_ram_init,
            with_uart                = False, #needs to be false to be able to add uart manually
            # with_uartbone            = with_uartbone,
            # uart_name                = {True: "crossover", False:"serial"}[with_uartbone],
        )
        serial_signals = Record(layout=[("tx", 1), ("rx", 1)])
        self.add_uart(name="uart", uart_name={True: "crossover", False:"serial"}[with_uartbone], baudrate=115200, fifo_depth=16, with_dynamic_baudrate=False, uart_pads=serial_signals)
        if with_uartbone:
            self.add_uartbone_mod(baudrate=115200, with_dynamic_baudrate=False, uart_pads=serial_signals)

        # Avoid stalling CPU at startup.
        self.uart.add_auto_tx_flush(sys_clk_freq=sys_clk_freq, timeout=1, interval=128)

        # Automatically jump to pre-initialized firmware.
        self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["main_ram"])

        # Define platform name constant.
        self.add_constant(platform.name.upper())

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # I2C Bus0 (LM75 & EEPROM) -----------------------------------------------------------------
        self.i2c0 = I2CMaster(pads=platform.request("FPGA_I2C"))

        # SPI (LMS7002 & DAC) ----------------------------------------------------------------------
        spi_pads = platform.request("FPGA_SPI")
        self.add_spi_master(name="spimaster", pads=spi_pads, data_width=32, spi_clk_freq=10e6)

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes

            self.add_spi_flash(mode="1x", clk_freq=100_000, module=W25Q128JV(Codes.READ_1_1_1), with_master=True)

        # LimeTop ----------------------------------------------------------------------------------

        self.limetop  = LimeTop(self, platform,
            LMS_DIQ_WIDTH      = LMS_DIQ_WIDTH,
            sink_width         = STRM0_FPGA_RX_RWIDTH,
            sink_clk_domain    = "sys",
            source_width       = STRM0_FPGA_TX_WWIDTH,
            source_clk_domain  = "sys",
            TX_N_BUFF          = TX_N_BUFF,
            TX_PCT_SIZE        = TX_PCT_SIZE,
            TX_IN_PCT_HDR_SIZE = TX_IN_PCT_HDR_SIZE,
            with_rx_tx_top     = with_rx_tx_top,

            with_fft           = with_fft,
            # FPGACFG.
            board_id           = 0x0011,
            major_rev          = MajorRevision,
            compile_rev        = CompileRevision,
            revision_pads      = platform.request("revision"),
        )
        # Assign UART signals to general periph
        self.comb += [
            self.limetop.general_periph.gpio_out_val[8].eq(serial_signals.tx),
            serial_signals.rx.eq(self.limetop.general_periph.gpio_in_val[9]),
        ]

        self.comb += self.limetop.rxtx_top.tx_path.ext_reset_n.eq(self.limetop.fpgacfg.rx_en)

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
            s_clk_domain       = "sys",
            m_clk_domain       = "sys",
        )

        self.comb += [
            self.ft601.source.connect(self.limetop.sink),
            self.limetop.source.connect(self.ft601.sink),
            # FT601 <-> RXTX Top.
            self.ft601.stream_fifo_fpga_pc_reset_n.eq(self.limetop.rxtx_top.rx_en),
            self.ft601.stream_fifo_pc_fpga_reset_n.eq(self.limetop.rxtx_top.rx_en),
        ]

        # Connect stream indication 
        self.comb += [
            self.limetop.wr_active.eq(self.ft601.wr_active),
            self.limetop.rd_active.eq(self.ft601.rd_active),
        ]
        # Timing Constraints -----------------------------------------------------------------------

        # FIXME: Improve, minimal for now.

        timings_sdc_filename = "timing.sdc"
        with open(timings_sdc_filename, "w") as f:
            # Write timing constraints.
            f.write("# FT601 / 100MHz.\n")
            f.write("create_clock -name FT_CLK  -period 10.000 [get_ports FT_CLK]\n\n")

            # Sys Clk / 77.5MHz.
            f.write("# Sys Clk / 77.5MHz.\n")
            f.write("create_clock -name SYS_CLK -period 12.903 [get_pins {OSCG.OSC}]\n\n")

            # LMS7002M / 40MHz & 125MHz.
            f.write("# LMS7002M / 40MHz & 125MHz.\n")
            f.write("create_clock -name LMK_CLK   -period 25.000 [get_ports LMK_CLK]\n")
            f.write("create_clock -name LMS_MCLK1 -period 8.000  [get_ports LMS_MCLK1]\n")
            f.write("create_clock -name LMS_MCLK2 -period 8.000  [get_ports LMS_MCLK2]\n")
        self.platform.add_sdc(timings_sdc_filename)

        # PPSDO ------------------------------------------------------------------------------------

        with_ppsdo = True

        if with_ppsdo:
            sys.path.append("../LimePPSDO/src") # FIXME.

            # FIXME: Fake pps, replace.
            pps = Signal()
            self.pps_timer = pps_timer = WaitTimer(sys_clk_freq - 1)
            self.comb += [
                pps.eq(self.pps_timer.done),
                self.pps_timer.wait.eq(~self.pps_timer.done)
            ]

            # PPSDO Instance.
            from ppsdo import PPSDO
            self.ppsdo = ppsdo = PPSDO(cd_sys="sys", cd_rf="lms_rx", with_csr=True)
            self.ppsdo.add_sources()
            self.comb += ppsdo.pps.eq(pps) # PPS.

    # LiteScope Analyzer Probes --------------------------------------------------------------------

    def add_ft601_ctrl_probe(self):
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

    def add_rxdatapath_ctrl_probe(self):
        analyzer_signals = [
            #self.lms7002_top.source.ready,
            #self.lms7002_top.source.valid,
            self.rxtx_top.rx_path.iqsmpls_fifo.sink.ready,
            self.rxtx_top.rx_path.iqsmpls_fifo.sink.valid,
            self.rxtx_top.rx_path.iqsmpls_fifo.sink.last,
            self.rxtx_top.rx_path.iqsmpls_fifo.source.valid,
            self.rxtx_top.rx_path.iqsmpls_fifo.source.ready,
            self.rxtx_top.rx_path.iqsmpls_fifo.source.last,
            self.rxtx_top.rx_path.iqpacket_axis.ready,
            self.rxtx_top.rx_path.iqpacket_axis.valid,
            self.rxtx_top.rx_path.fifo_iqpacket.sink.ready,
            self.rxtx_top.rx_path.fifo_iqpacket.sink.valid,
            self.rxtx_top.rx_path.fifo_iqpacket.level,
            self.rxtx_top.rx_path.iqpacket_cdc.sink.valid,
            self.rxtx_top.rx_path.iqpacket_cdc.sink.ready,
            self.rxtx_top.rx_path.iqpacket_cdc.source.valid,
            self.rxtx_top.rx_path.iqpacket_cdc.source.ready,
            self.rxtx_top.rx_path.source.ready,
            self.rxtx_top.rx_path.source.valid,
            self.rxtx_top.rx_path.drop_samples,
            self.rxtx_top.rx_path.wr_header,

            self.ft601.sink.ready,
            self.ft601.sink.valid,
            self.ft601.EP83_fifo.level,
            self.ft601.EP83_fifo.sink.ready,
            self.ft601.EP83_fifo.sink.valid,
            self.ft601.EP83_fifo.sink.data,
            self.ft601.EP83_conv.sink.valid,
            self.ft601.EP83_conv.sink.ready,
            self.ft601.EP83_conv.source.valid,
            self.ft601.EP83_conv.source.ready,
            #self.ft601.EP83_fifo_status.busy_in,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 1024,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv"
        )
    # SoC hierarchy JSON utilities -----------------------------------------------------------------

    def print_soc_hierarchy_json(self, outfile=None):
        """Generate the SoC submodule hierarchy and write it as JSON to soc_structure.json.
        The filename is constant. No terminal printing.
        """
        write_module_hierarchy_json(self, outfile="soc_structure.json", name="SoC")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR-Mini-V2 LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--toolchain",    default="trellis",   help="Build toolchain.", choices=["diamond", "trellis"])
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--flash",        action="store_true", help="Flash Golden and User bitstreams using the MCS file.")
    parser.add_argument("--flash-user",   action="store_true", help="Flash User bitstream.")
    parser.add_argument("--flash-golden", action="store_true", help="Flash Golden bitstream.")
    parser.add_argument("--cable",        default="ft2232",    help="JTAG cable.")

    # SoC parameters.
    parser.add_argument("--with-bios",         action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-uartbone",     action="store_true", help="Enable UARTBone.")
    parser.add_argument("--without-spi-flash", action="store_true", help="Disable SPI Flash (MMAPed).")
    parser.add_argument("--cpu-type",          default="vexriscv",  help="Select CPU.", choices=[
        "vexriscv", "picorv32", "fazyrv", "firev"]),

    # Examples.
    parser.add_argument("--with-fft",       action="store_true", help="Enable FFT module examples.")

    # Introspection.
    parser.add_argument("--no-soc-json",    action="store_true", help="Disable automatic SoC hierarchy JSON generation.")

    # Litescope Analyzer Probes.
    probeopts = parser.add_mutually_exclusive_group()
    probeopts.add_argument("--with-ft601-ctrl-probe",      action="store_true", help="Enable FT601 Ctrl Probe.")
    probeopts.add_argument("--with-rxdatapath-probe",      action="store_true", help="Enable RXDatapath Ctrl Probe.")

    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        # SoC.
        soc = BaseSoC(
            cpu_type       = args.cpu_type,
            toolchain      = args.toolchain,
            with_bios      = args.with_bios,
            with_uartbone  = args.with_uartbone,
            with_spi_flash = not args.without_spi_flash,
            cpu_firmware   = None if prepare else "firmware/firmware.bin",
            with_fft       = args.with_fft,
        )
        # LiteScope Analyzer Probes.
        if args.with_ft601_ctrl_probe or args.with_rxdatapath_probe:
            assert args.with_uartbone
            if args.with_ft601_ctrl_probe:
                soc.add_ft601_ctrl_probe()
            if args.with_rxdatapath_probe:
                soc.add_rxdatapath_ctrl_probe()
        # Builder.
        builder = Builder(soc, csr_csv="csr.csv", bios_console="lite")
        builder.build(run=build)
        # Firmware build.
        if prepare:
            linker = {
                True  : "linker_main_ram.ld",
                False : "linker_rom.ld",
            }[args.with_bios]
            # Create a makefile fragment with board specific variables
            # delete old one if it exists
            env_mak = os.path.join("firmware", "env.mak")
            if os.path.exists(env_mak):
                os.remove(env_mak)
            with open(env_mak, "w") as f:
                f.write(f"BUILD_DIR={builder.output_dir}\n")
                f.write(f"TARGET={soc.platform.name.upper()}\n")
                f.write(f"LINKER={linker}\n")
            os.system(f"cd firmware && make clean all")
            bistream_output_dir = "bitstream/LimeSDR_Mini_V2"
            if not os.path.exists(bistream_output_dir):
                os.makedirs(bistream_output_dir)

        # Always generate SoC hierarchy JSON during prepare pass unless disabled.
        if prepare and not args.no_soc_json:
            soc.print_soc_hierarchy_json()

    # Prepare User/Golden bitstream.
    if which("ddtcmd") is None:
        msg = "\nUnable to find Diamond ddtcmd tool, please:\n"
        msg += "- Add Diamond toolchain to your $PATH.\n"
        msg += "\nCannot generate the MCS file.\n"
        print(msg)
    else:
        os.system(f"./tools/limesdr_mini_v2_bitstream.py")

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bit"))

    # Flash Bitstreams (User + Golden).
    if args.flash:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0, "bitstream/LimeSDR_Mini_V2/limesdr_mini_v2.mcs")

    # Flash Golden Bitstream.
    if args.flash_golden:
        golden = f"bitstream/LimeSDR_Mini_V2/{soc.platform.name}_golden.bit"
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0x00140000, golden)

    # Flash User Bitstream.
    if args.flash_user:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0, builder.get_bitstream_filename(mode="sram", ext=".bit"))

if __name__ == "__main__":
    main()
