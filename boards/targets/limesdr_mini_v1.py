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

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from boards.platforms import limesdr_mini_v1_platform as limesdr_mini_v1

from litex.soc.interconnect         import stream
from litex.soc.interconnect.csr     import *
from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock          import Max10PLL
from litex.soc.cores.bitbang        import I2CMaster
from litex.soc.cores.spi.spi_master import SPIMaster

from litespi.phy.generic import LiteSPIPHY

from litescope import LiteScopeAnalyzer

from gateware.Revision import *
from gateware.max10_onchipflash.max10_onchipflash import Max10OnChipFlash
from gateware.max10_dual_cfg.max10_dual_cfg       import Max10DualCfg

from gateware.LimeDFB_LiteX.FT601.src.ft601 import FT601

from gateware.LimeTop                       import LimeTop

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
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.comb += por_done.eq(Reduce("AND", por_count))
        self.sync.por += por_count.eq(Cat(Constant(1, 1), por_count[0:3]))

        # Sys Clk/Rst.
        self.comb += self.cd_sys.clk.eq(self.lmk_clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, self.rst | ~por_done)

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
    def __init__(self, sys_clk_freq=40e6, cpu_type="vexriscv", toolchain="quartus",
        with_bios      = False,
        with_rx_tx_top = False,
        with_lms7002   = False,
        flash_boot     = False,
        with_uartbone  = False,
        with_spi_flash = False,
        cpu_firmware   = None,
        gold_img       = False,
        **kwargs):

        # Platform ---------------------------------------------------------------------------------
        platform             = limesdr_mini_v1.Platform(toolchain=toolchain)
        platform.name        = "limesdr_mini_v1"
        platform.vhd2v_force = False
        platform.add_platform_command("set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008") # Enable VHDL-2008 support.
        platform.add_platform_command("set_global_assignment -name INTERNAL_FLASH_UPDATE_MODE \"DUAL IMAGES\"") # Required to access internal flash

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
            integrated_main_ram_size = 0x3800
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_sram_ram_size = 0x400
        elif flash_boot:
            integrated_rom_size      = 0
            integrated_rom_init      = []
            integrated_main_ram_size = 0
            integrated_main_ram_init = []
            integrated_sram_ram_size = 0x800
        else:
            integrated_rom_size      = 0x4000
            integrated_rom_init      = [0] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_main_ram_size = 0
            integrated_main_ram_init = []
            integrated_sram_ram_size = 0x400

        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-Mini-V2",
            ident_version            = True,
            cpu_type                 = cpu_type,
            cpu_variant              = cpu_variant,
            integrated_rom_size      = integrated_rom_size,
            integrated_rom_init      = integrated_rom_init,
            integrated_sram_ram_size = integrated_sram_ram_size,
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
        if not flash_boot:
            self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["main_ram"])

        # Define platform name constant.
        self.add_constant(platform.name.upper())

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # I2C Bus0 (LM75 & EEPROM) -----------------------------------------------------------------
        self.i2c0 = I2CMaster(pads=platform.request("FPGA_I2C"))

        # SPI (LMS7002 & DAC) ----------------------------------------------------------------------
        self.add_spi_master(name="spimaster", pads=platform.request("FPGA_SPI"), data_width=32, spi_clk_freq=10e6)

        # Internal Flash ---------------------------------------------------------------------------
        self.internal_flash = Max10OnChipFlash(platform, {True:cpu_firmware, False: None}[flash_boot])

        internal_flash_region = SoCRegion(
                origin = 0x100000, # keep original addr
                size   = 0x8C000,
                mode   = "rwx")
        self.bus.add_slave("internal_flash", self.internal_flash.bus, internal_flash_region)

        if flash_boot:
            self.bus.add_region("rom", SoCRegion(
                origin = self.bus.regions["internal_flash"].origin,
                size   = 32768 * 2,
                linker = True)
            )

            # Automatically jump to pre-initialized firmware.
            self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["main_ram"])
            self.cpu.set_reset_address(self.bus.regions["rom"].origin)

        # Max10 Dual Cfg ---------------------------------------------------------------------------
        self.dual_cfg = Max10DualCfg(platform)
        dual_cfg_region = SoCRegion(
                origin = 0x900000,
                size   = 0x100,
                mode   = "rwx")
        self.bus.add_slave("dual_cfg", self.dual_cfg.bus, dual_cfg_region)

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes

            self.add_spi_flash(mode="1x", clk_freq=100_000, module=W25Q128JV(Codes.READ_1_1_1), with_master=True)

        # LimeTop ----------------------------------------------------------------------------------

        revision_pads = platform.request("revision")
        revision_pads.BOM_VER = Cat(revision_pads.BOM_VER0, revision_pads.BOM_VER1, revision_pads.BOM_VER2)

        limetop  = LimeTop(self, platform,
            LMS_DIQ_WIDTH      = LMS_DIQ_WIDTH,
            sink_width         = STRM0_FPGA_RX_RWIDTH,
            sink_clk_domain    = "ft601",
            source_width       = STRM0_FPGA_TX_WWIDTH,
            source_clk_domain  = "ft601",
            TX_N_BUFF          = TX_N_BUFF,
            TX_PCT_SIZE        = TX_PCT_SIZE,
            TX_IN_PCT_HDR_SIZE = TX_IN_PCT_HDR_SIZE,
            with_rx_tx_top     = with_rx_tx_top,
            with_lms7002       = with_lms7002,

            # FPGACFG.
            board_id           = 0x0011,
            major_rev          =  MajorRevision if not gold_img else 0xDEAD,
            compile_rev        =  CompileRevision if not gold_img else 0xDEAD,
            revision_pads      = revision_pads,
        )
        # Make sure all sync statements happen on ft601 clock domain.
        # limetop = ClockDomainsRenamer({"sys": "ft601"})(limetop)
        self.limetop = limetop
        # Assign UART signals to general periph
        self.comb += [
            self.limetop.general_periph.gpio_out_val[8].eq(serial_signals.tx),
            serial_signals.rx.eq(self.limetop.general_periph.gpio_in_val[9]),
            ]


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
            s_clk_domain       = "ft601",
            m_clk_domain       = "ft601",
        )

        self.comb += [
            self.ft601.source.connect(self.limetop.sink),
            self.limetop.source.connect(self.ft601.sink),
        ]

        if with_rx_tx_top:
            self.comb += [
                self.limetop.rxtx_top.tx_path.ext_reset_n.eq(self.limetop.fpgacfg.rx_en),
                # FT601 <-> RXTX Top.
                self.ft601.stream_fifo_fpga_pc_reset_n.eq(self.limetop.rxtx_top.rx_pct_fifo_aclrn_req),
                self.ft601.stream_fifo_pc_fpga_reset_n.eq(self.limetop.rxtx_top.rx_en),
            ]

        # Timing Constraints -----------------------------------------------------------------------

        # FIXME: Add timing constraints.
        # LMS7002M constrains
        # TODO: The SDC file for LMS7002M is currently added directly in the platform. Find a cleaner way to add constraints without cluttering this file.

        # FT601 constraints.
        platform.toolchain.additional_sdc_commands.append("set_input_delay -max 7.0 -clock [get_clocks FT_CLK] [get_ports {FT_RXFn FT_TXEn}]")
        platform.toolchain.additional_sdc_commands.append("set_input_delay -max 7.0 -clock [get_clocks FT_CLK] [get_ports {FT_BE[*] FT_D[*]}]")
        platform.toolchain.additional_sdc_commands.append("set_input_delay -min 4.0 -clock [get_clocks FT_CLK] [get_ports {FT_RXFn FT_TXEn}] -add_delay")
        platform.toolchain.additional_sdc_commands.append("set_input_delay -min 4.0 -clock [get_clocks FT_CLK] [get_ports {FT_BE[*] FT_D[*]}] -add_delay")

        platform.toolchain.additional_sdc_commands.append("set_output_delay -max 0.5 -clock [get_clocks FT_CLK] [get_ports FT_WRn] -add_delay")
        platform.toolchain.additional_sdc_commands.append("set_output_delay -min 0.5 -clock [get_clocks FT_CLK] [get_ports FT_WRn] -add_delay")
        platform.toolchain.additional_sdc_commands.append("set_output_delay -max 0.5 -clock [get_clocks FT_CLK] [get_ports {FT_BE[*] FT_D[*]}] -add_delay")
        platform.toolchain.additional_sdc_commands.append("set_output_delay -min 0.5 -clock [get_clocks FT_CLK] [get_ports {FT_BE[*] FT_D[*]}] -add_delay")

        # Clock groups.
        platform.toolchain.additional_sdc_commands.append(
            "set_clock_groups -asynchronous "
            "-group {LMK_CLK FPGA_SPI_SCLK FPGA_SPI_SCLK_out DUAL_BOOT_CLK ONCHIP_FLASH_CLK FPGA_SPI_SCLK_FPGA} "
            "-group {FT_CLK} "
            "-group {LMS_MCLK2} "
            "-group {TX_C0} "
            "-group {TX_C1 LMS_FCLK1} "
            "-group {RX_C2} "
            "-group {RX_C3 LMS_FCLK2}"
        )

        # False path constraints.
        platform.toolchain.additional_sdc_commands.append("set_false_path -from [get_clocks LMS_MCLK1]")
        platform.toolchain.additional_sdc_commands.append("set_false_path -from [get_clocks LMS_MCLK2] -to [get_clocks LMS_MCLK2]")
        platform.toolchain.additional_sdc_commands.append("set_false_path -from [get_clocks LMS_MCLK2_VIRT] -to [get_clocks LMS_MCLK2]")

        # Timing constrains for Low sample rate path delay on LCELL combinational cells
        platform.toolchain.additional_sdc_commands.append("set_net_delay 	-max 5.1 -from [get_pins -compatibility_mode *lcell*|combout*]")
        platform.toolchain.additional_sdc_commands.append("set_net_delay 	-min 4 -from [get_pins -compatibility_mode *lcell*|combout*]")

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

    def add_i2c0_signals_probe(self):
        analyzer_signals = [
            self.i2c0.pads.scl,
            self.i2c0.pads.sda,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 2048,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR-Mini-V2 LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build",     action="store_true", help="Build bitstream.")
    parser.add_argument("--toolchain", default="quartus",   help="FPGA toolchain.", choices=["quartus"])
    parser.add_argument("--flash",     action="store_true", help="Flash bitstream.")
    parser.add_argument("--cable",     default="ft2232",    help="JTAG cable.")

    # SoC parameters.
    parser.add_argument("--with-bios",      action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-uartbone",  action="store_true", help="Enable UARTBone.")
    parser.add_argument("--with-spi-flash", action="store_true", help="Enable SPI Flash (MMAPed).")
    parser.add_argument("--golden",         action="store_true", help="Build golden image instead of user")

    # Litescope Analyzer Probes.
    probeopts = parser.add_mutually_exclusive_group()
    probeopts.add_argument("--with-ft601-ctrl-probe",   action="store_true", help="Enable FT601 Ctrl Probe.")
    probeopts.add_argument("--with-i2c0-signals-probe", action="store_true", help="Enable I2C0 SDA/SCL Probe.")

    args = parser.parse_args()

    cpu_firmware = "firmware/firmware" + {True: ".bin", False: ".hex"}[args.golden]
    cpu_firmware = os.path.join(os.path.abspath(os.path.dirname(".")), cpu_firmware)

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        # SoC.
        soc = BaseSoC(
            toolchain      = args.toolchain,
            with_bios      = args.with_bios,
            with_rx_tx_top = not args.golden,
            with_lms7002   = not args.golden,
            flash_boot     = not args.golden,
            with_uartbone  = args.with_uartbone,
            with_spi_flash = args.with_spi_flash,
            cpu_firmware   = None if prepare else cpu_firmware,
            gold_img       = args.golden
        )
        # LiteScope Analyzer Probes.
        if args.with_ft601_ctrl_probe:
            assert args.with_uartbone
            soc.add_ft601_ctrl_probe()
        if args.with_i2c0_signals_probe:
            assert args.with_uartbone
            soc.add_i2c0_signals_probe()
        # Builder.
        output_dir = os.path.abspath(os.path.join("build", soc.platform.name))
        if args.golden:
            output_dir = output_dir + "_golden"
        builder = Builder(soc, output_dir=output_dir, csr_csv="csr.csv", bios_console="lite")
        builder.build(run=build)
        # Firmware build.
        if prepare:
            linker = {
                True  : "linker_main_ram.ld",
                False : "linker_rom.ld",
            }[args.with_bios]
            is_golden = {
                True  : 1,
                False : 0,
            }[args.golden]
            # Create a makefile fragment with board specific variables
            # delete old one if it exists
            env_mak = os.path.join("firmware", "env.mak")
            if os.path.exists(env_mak):
                os.remove(env_mak)
            with open(env_mak, "w") as f:
                f.write(f"BUILD_DIR={builder.output_dir}\n")
                f.write(f"TARGET={soc.platform.name.upper()}\n")
                f.write(f"LINKER={linker}\n")
                f.write(f"GOLDEN={is_golden}\n")
            os.system(f"cd firmware && make clean all")
            assert os.path.exists(cpu_firmware), f"Error: {cpu_firmware} not available"

    # Prepare pof/rpd files.
    # TODO: add logic so that image generation could use a prebuilt gold image if possible, or requested
    if os.path.exists(os.path.join(output_dir + "_golden", "gateware/limesdr_mini_v1.sof")):
        os.system("quartus_cpf -c gateware/gen_pof_file.cof")
        os.system("quartus_cpf -c -q 25MHz -g 3.3 -n p bitstream/LimeSDR_Mini_V1/LimeSDR-Mini_lms7_trx_HW_1.2.pof bitstream/LimeSDR_Mini_V1/LimeSDR-Mini_lms7_trx_HW_1.2.svf")

        # Flash Bitstream.
        if args.flash:
            prog = soc.platform.create_programmer(cable=args.cable)
            prog.flash(0, "bitstream/LimeSDR_Mini_V1/LimeSDR-Mini_lms7_trx_HW_1.2.svf")

    else:
        print("RPD/POF: Disabled. Missing Golden bitstream")

        if args.flash:
            print("Can't flash bitstream: Please build golden before.")

if __name__ == "__main__":
    main()
