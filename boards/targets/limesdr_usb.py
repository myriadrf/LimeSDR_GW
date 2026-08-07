#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from tools.spi_cpha_patch import patch_spi_master_cpha
# Add runtime-selectable SPI clock phase (CPHA) to the LiteX SPIMaster. This is required so the
# AD5601 VCTCXO DAC on "fpga_spi1" (which latches SDIN on the SCLK falling edge, i.e. SPI Mode 1)
# can be driven correctly while the ADF4002 PLL sharing the same master stays in SPI Mode 0.
# Default (cpha=0) is bit-for-bit identical to the stock core, so all existing devices are
# unaffected. Must be applied before any SPIMaster / add_spi_master is instantiated.
patch_spi_master_cpha(verbose=False)

import os
import argparse
import shutil
import subprocess

from deps.litex.litex.soc.interconnect.stream import ClockDomainCrossing
from gateware.LimeDFB.FX3.src.FX3 import FX3
from gateware.LimeTop import LimeTop
from gateware.board_specific.limesdr_usb.PSS_LimeSDR_Usb import PSS_LimeSDR_Usb
from gateware.helpers import write_module_hierarchy_json
from gateware.Revision import *
from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.soc.interconnect import stream

from boards.platforms import limesdr_usb_platform as limesdr_usb

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP

# Constants ----------------------------------------------------------------------------------------

FPGA_TO_HOST_DATA_WIDTH = 64 # bus width connecting FX3 and Limetop
HOST_TO_FPGA_DATA_WIDTH = 64 # bus width connecting FX3 and Limetop
WFM_DATA_WIDTH          = 32 # bus width connecting FX3 and wfmplayer
TX_MAX_BUF_PACKETS      = 16      # maximum number of buffered tx packets in Limetop (any size)
TX_PACKET_BUF_SIZE      = 16384   # total size (in bytes) of tx packet buffer in Limetop

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):

        # Safety feature. Currently there is no support for other frequencies.
        assert sys_clk_freq == 100e6

        self.cd_sys   = ClockDomain()
        self.fx3_pclk = platform.request("FX3_PCLK")
        # FX3 PCLK runs at 100MHz
        platform.add_period_constraint(self.fx3_pclk, 1e9/100e6)
        self.specials += Instance("GLOBAL",
            i_in  = self.fx3_pclk,
            o_out = self.cd_sys.clk
        )
        # self.comb += self.cd_sys.clk.eq(self.fx3_pclk)

        self.ext_gnd = Signal()
        self.ext_gnd = platform.request("EXT_GND")
        self.specials += AsyncResetSynchronizer(self.cd_sys,self.ext_gnd)

        self.cd_lmk  = ClockDomain()
        self.clk_lmk = platform.request("LMK_CLK")
        platform.add_period_constraint(self.clk_lmk, 1e9 / 30.72e6)
        self.comb += self.cd_lmk.clk.eq(self.clk_lmk)

        self.cd_si0 = ClockDomain()
        self.clk_si0 = platform.request("SI_CLK", 0)
        platform.add_period_constraint(self.clk_si0, 1e9 / 250e6)
        self.comb += self.cd_si0.clk.eq(self.clk_si0)

        self.cd_si1 = ClockDomain()
        self.clk_si1 = platform.request("SI_CLK", 1)
        platform.add_period_constraint(self.clk_si1, 1e9 / 250e6)
        self.comb += self.cd_si1.clk.eq(self.clk_si1)

        self.cd_si2 = ClockDomain()
        self.clk_si2 = platform.request("SI_CLK", 2)
        platform.add_period_constraint(self.clk_si2, 1e9 / 250e6)
        self.comb += self.cd_si2.clk.eq(self.clk_si2)

        self.cd_si3 = ClockDomain()
        self.clk_si3 = platform.request("SI_CLK", 3)
        platform.add_period_constraint(self.clk_si3, 1e9 / 250e6)
        self.comb += self.cd_si3.clk.eq(self.clk_si3)

        # No SI CLK 4

        self.cd_si5 = ClockDomain()
        self.clk_si5 = platform.request("SI_CLK", 5)
        platform.add_period_constraint(self.clk_si5, 1e9 / 250e6)
        self.comb += self.cd_si5.clk.eq(self.clk_si5)

        self.cd_si6 = ClockDomain()
        self.clk_si6 = platform.request("SI_CLK", 6)
        platform.add_period_constraint(self.clk_si6, 1e9 / 250e6)
        self.comb += self.cd_si6.clk.eq(self.clk_si6)

        self.cd_si7 = ClockDomain()
        self.clk_si7 = platform.request("SI_CLK", 7)
        platform.add_period_constraint(self.clk_si7, 1e9 / 250e6)
        self.comb += self.cd_si7.clk.eq(self.clk_si7)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self,
                 sys_clk_freq      = 100e6,
                 with_bios         = False,
                 no_ddr            = False,
                 cpu_firmware   = None,
                 with_jtagbone     = False,
                 with_cpu_debug    = False,
                 no_ppsdo          = False):
        platform = limesdr_usb.Platform()
        platform.name        = "limesdr_usb"
        platform.vhd2v_force = False
        platform.add_platform_command("set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008")

        # JTAGBone (wishbone-over-JTAG bus master for gateware debugging / register
        # monitoring with litex_server) and the RISC-V CPU debug tunnel both consume the
        # board's single reserved Altera Virtual-JTAG (sld_virtual_jtag) instance, so they
        # cannot be enabled at the same time.
        # FIXME: cycloneIV openocd litex support added in litex commit 00a2084a8596c00f445425b1ccce62772052f93f
        #        (https://github.com/enjoy-digital/litex/commit/00a2084a8596c00f445425b1ccce62772052f93f)
        #        if using OLDER version of litex, jtagbone will not work, unless litex dep manually modified as in the commit
        # TODO: Remove this and the above fixme comments when this repo's litex_config.py points to a newer version of litex
        #       than specified above
        if with_jtagbone and with_cpu_debug:
            raise ValueError(
                "--with-jtagbone and --with-cpu-debug are mutually exclusive: both need the "
                "single Altera Virtual-JTAG instance on this Cyclone IV."
            )

        # The debug build swaps in a heavier VexRiscv-SMP core; drop DDR to free EP4CE40
        # resources (equivalent to passing --no-ddr).
        no_ddr = no_ddr or with_cpu_debug

        if with_bios:
            integrated_rom_size      = 0x4100
            integrated_rom_init      = []
            integrated_main_ram_size = 0x4100
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
        else:
            integrated_rom_size      = 0x4100
            integrated_rom_init      = [0] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_main_ram_size = 0
            integrated_main_ram_init = []

        # SoCCore ----------------------------------------------------------------------------------
        # CPU selection: production uses the small VexRiscv "minimal" core. The opt-in
        # --with-cpu-debug build switches to a single-core VexRiscv-SMP exposing a
        # spec-compliant RISC-V Debug Module over a dedicated Altera Virtual-JTAG tunnel
        # (wired in add_jtag_cpu_debug()), debugged directly with upstream OpenOCD.
        if with_cpu_debug:
            cpu_type    = "vexriscv_smp"
            cpu_variant = "standard"
            VexRiscvSMP.with_rvc             = True
            VexRiscvSMP.privileged_debug     = True
            VexRiscvSMP.hardware_breakpoints = 4
        else:
            cpu_type    = "vexriscv"
            cpu_variant = "minimal"
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-USB",
            ident_version            = True,
            cpu_type                 = cpu_type,
            cpu_variant              = cpu_variant,
            integrated_rom_size      = integrated_rom_size,
            integrated_rom_init      = integrated_rom_init,
            integrated_sram_size     = 0x2000,
            integrated_main_ram_size = integrated_main_ram_size,
            integrated_main_ram_init = integrated_main_ram_init,
            with_uart                = False, #for now
            # with_uartbone            = with_uartbone,
            # uart_name                = {True: "crossover", False:"serial"}[with_uartbone],
        )

        # 1 for CSR
        # 2 for FTDI
        # 3 for FX3
        self.add_constant("LMS64C_METHOD",3)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # JTAGBone ---------------------------------------------------------------------------------
        # Opt-in wishbone-over-JTAG bus master (over the on-board FT2232 cable) for gateware
        # debugging and CSR/register monitoring via `litex_server --jtag` + `litex_cli`. This
        # leaves the production vexriscv "minimal" core untouched and is independent of the
        # RISC-V CPU debug tunnel (which is mutually exclusive with it, see above).
        if with_jtagbone:
            self.add_jtagbone()
            platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
            platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # JTAG CPU Debug ---------------------------------------------------------------------------
        if with_cpu_debug:
            self.add_jtag_cpu_debug()

        # FX3
        self.FX3 = FX3(platform=platform,
                       pads=platform.request("FX3"),
                       vendor="altera",
                       EP01_0_rwidth = HOST_TO_FPGA_DATA_WIDTH,
                       EP01_1_rwidth = WFM_DATA_WIDTH,
                       EP81_wwidth   = FPGA_TO_HOST_DATA_WIDTH
                       )

        # LMS SPI -----------------------------------------------------------------------------------
        # LMS spi declared outside PSS, because the current firmware driver expects that
        self.add_spi_master(name="spimaster", pads=platform.request("FPGA_SPI0"), data_width=32, spi_clk_freq=1e6)

        # Revision pads (shared between LimeTop's FPGACFG board-id readback and the PSS's FX3 LED
        # hardware-version gating).
        revision_pads = platform.request("revision")

        # PSS (Peripheral Support Subsystem)
        self.pss = PSS_LimeSDR_Usb(self, platform, sys_clk_freq,
                                   pll_ref_clk=self.crg.fx3_pclk,
                                   revision_pads=revision_pads,
                                   fx3_busy=self.FX3.busy_out,
                                   add_ddr_modules=not no_ddr,
                                   wfm_infifo_usedw_width=self.FX3.ep01_0_rdusedw_width,
                                   )

        # PPSDO ------------------------------------------------------------------------------------
        if not no_ppsdo:
            from gateware.LimePPSDO.src.ppsdo import PPSDO
            self.ppsdo = PPSDO(
                cd_rf    = "lmk",
                with_csr = True
            )
            self.comb += self.ppsdo.pps.eq(self.pss.gpio_io.in_val[7])
            self.ppsdo.add_sources(dac_bits=8, patch_fazyrv=True)

        # LimeTop -----------------------------------------------------------------------------------
        self.limetop  = LimeTop(self,
                                platform             = platform,
                                vendor               = "altera",
                                family               = "cycloneIV",
                                sink_width           = HOST_TO_FPGA_DATA_WIDTH,
                                source_width         = FPGA_TO_HOST_DATA_WIDTH,
                                rx_fixed_packet_size = True,
                                TX_N_BUFF            = TX_MAX_BUF_PACKETS,
                                TX_MAX_PCT_SIZE      = TX_PACKET_BUF_SIZE,
                                TX_WITHTXIQ_MUX      = True,
                                # FPGACFG.
                                board_id             = 0x0011,
                                major_rev            = MajorRevision,
                                compile_rev          = CompileRevision,
                                revision_pads        = revision_pads,

                                with_event_manager   = False,
                                with_clk_cfg_irq     = False,
                                )

        self.comb += [
            self.FX3.data_source.connect  (self.limetop.sink),
            self.limetop.source.connect   (self.FX3.data_sink),
            self.FX3.data_sink_clr.eq     (~self.limetop.fpgacfg.rx_en),
            self.FX3.data_source0_clr.eq  (~self.limetop.fpgacfg.rx_en),
            self.limetop.rxtx_top.tx_path.ext_reset_n.eq(self.limetop.fpgacfg.rx_en),

            # PSS LED1/FPGA_GPIO default value <- TX/RX PLL lock status (pll_lock[0]=TX, pll_lock[1]=RX).
            self.pss.tx_pll_lock.eq(self.limetop.lms7002_top.lms7002_clk.CLK_CTRL.PLL_LOCK.status[0]),
            self.pss.rx_pll_lock.eq(self.limetop.lms7002_top.lms7002_clk.CLK_CTRL.PLL_LOCK.status[1]),

            # PSS FPGA_GPIO default value <- TX antenna-enable / TX packet-loss flag.
            self.pss.tx_txant_en.eq(self.limetop.lms7002_top.tx_ant_en),
            self.pss.tx_pct_loss_flg.eq(self.limetop.rxtx_top.tx_path.pct_loss_flg),
        ]
        # WFMPlayer wiring. The WFM player is a DDR-backed feature, so the PSS only
        # instantiates it when DDR modules are present (add_ddr_modules / not no_ddr).
        # The --with-cpu-debug build implies --no-ddr for EP4CE40 headroom and therefore
        # has no WFM player; skip its wiring accordingly.
        if not no_ddr:
            # WFMPlayer <-> lms7002_top
            self.comb += [
                self.limetop.lms7002_top.wfm_sink_l.eq(self.pss.wfm_player.diq_l),
                self.limetop.lms7002_top.wfm_sink_h.eq(self.pss.wfm_player.diq_h),
            ]
            # FX3 <-> WFMPlayer
            # NOTE: both FX3 and WFMPlayer need usedw signals from their fifos to operate properly
            #       LiteX AsyncFifo does not have two level outputs, so two SyncFIFOs have to be used,
            #       one for each clock domain.
            # TODO: See if it's possible to improve asyncFifo to avoid this workaround.
            self.wfm_fifo = ClockDomainsRenamer("lms_tx")(
                ResetInserter()(stream.SyncFIFO([("data", 32)], depth=1024, buffered=True))
            )
            self.wfm_data_cdc = ClockDomainCrossing([("data",32)],"sys","lms_tx", depth=4)
            # No CDC for clear signal, since it is actually in sys clock domain (unmodified fpacfg wfm_load signal passed through wfmplayer)
            self.comb += [
                # --- CDC for wfm data
                self.FX3.data_source_1.connect(self.wfm_data_cdc.sink, omit=["keep", "id", "dest", "user"]),
                self.wfm_data_cdc.source.connect(self.wfm_fifo.sink),
                self.wfm_fifo.source.connect(self.pss.wfm_player.sink, omit=["keep", "id", "dest", "user"]),
                # --- FIFO level for burst management
                self.pss.wfm_player.sink_usedw.eq(self.wfm_fifo.level),
                # --- Controls
                self.wfm_fifo.reset.eq(~self.pss.wfm_player.wfm_infifo_reset_n),
                self.FX3.data_source1_clr.eq  (~self.pss.wfm_player.wfm_infifo_reset_n),
                self.FX3.data_source_sel.eq(self.pss.wfm_player.wfm_load)
            ]

    # JTAG CPU Debug -------------------------------------------------------------------------------
    def add_jtag_cpu_debug(self):
        # Expose the VexRiscv-SMP tunneled RISC-V Debug Module through the single Altera
        # Virtual-JTAG (sld_virtual_jtag) instance available on this Cyclone IV. This is the
        # same BSCAN-tunnel framework used by the XTRX/SSDR/HiperSDR reference boards, so it is
        # driven directly by upstream OpenOCD + riscv_jtag_tunneled.tcl (no litex_server, no
        # jtagbone, no OpenOCD fork). Because the debug bus no longer needs the wishbone bridge,
        # this instance is the sole virtual-JTAG consumer in debug builds.
        from litex.soc.cores.jtag import AlteraJTAG
        self.platform.add_reserved_jtag_decls()
        self.jtag = jtag = AlteraJTAG(
            primitive = AlteraJTAG.get_primitive(self.platform.device),
            pads      = self.platform.get_reserved_jtag_pads(),
        )
        self.comb += [
            self.cpu.jtag_reset.eq(jtag.reset),
            self.cpu.jtag_capture.eq(jtag.capture),
            self.cpu.jtag_shift.eq(jtag.shift),
            self.cpu.jtag_update.eq(jtag.update),
            self.cpu.jtag_clk.eq(jtag.tck),
            self.cpu.jtag_tdi.eq(jtag.tdi),
            self.cpu.jtag_enable.eq(True),
            jtag.tdo.eq(self.cpu.jtag_tdo),
        ]

        self.cd_jtag = ClockDomain()
        self.comb += ClockSignal("jtag").eq(jtag.tck)
        self.platform.add_period_constraint(self.cd_jtag.clk, 1e9/20e6)
        self.platform.add_false_path_constraints(self.cd_jtag.clk, self.crg.cd_sys.clk)

    # LiteScope Analyzer Probes --------------------------------------------------------------------
    def add_debug(self):
        reset_sig = Signal()
        analyzer_signals = []
        analyzer_signals += [self.pss.spi1_phy_pads]
        # Only import LiteScope when it's actually needed
        from litescope import LiteScopeAnalyzer
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 128,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

    # Utils
    def print_soc_hierarchy_json(self, outfile=None):
        """Generate the SoC submodule hierarchy and write it as JSON to soc_structure.json.
        The filename is constant. No terminal printing.
        """
        write_module_hierarchy_json(self, outfile="soc_structure.json", name="SoC")


    def generate_documentation(self, build_name, build_html, **kwargs):
        from litex.soc.doc import generate_docs
        generate_docs(self, "docs/docs/{}/litex_doc".format(build_name),
            project_name = "{}".format(build_name),
            author       = "Lime Microsystems")
        if build_html:
            os.system("sphinx-build -M html docs/docs/{}/litex_doc docs/docs/{}/litex_doc/_build".format(build_name, build_name))

# Flash SVF helpers --------------------------------------------------------------------------------

# Committed, device-specific Serial Flash Loader (SFL) configuration SVF. It loads
# the flash bridge into the FPGA SRAM and is constant (it does not depend on the
# user gateware), so it is stored in the repo and prepended to the .jic-derived
# flash operations at build time. Regenerate it with tools/generate_sfl_svf.py
# (e.g. after a Quartus major-version upgrade).
FLASH_SFL_SVF = "gateware/board_specific/limesdr_usb/sfl_ep4ce40_020f40dd.svf"

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR USB LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")
    parser.add_argument("--cable", default="ft2232", help="JTAG cable.")

    # SoC parameters.
    parser.add_argument("--with-bios",      action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-jtagbone",  action="store_true", help="Enable JTAGBone (wishbone-over-JTAG bus master) for gateware debugging / register monitoring with litex_server (mutually exclusive with --with-cpu-debug).")
    parser.add_argument("--with-cpu-debug", action="store_true", help="Enable spec-compliant RISC-V CPU debug over a dedicated JTAG tunnel (implies --no-ddr, mutually exclusive with --with-jtagbone).")
    parser.add_argument("--no-ddr",         action="store_true", help="Do not include DDR memory related modules. Useful for freeing resources when debugging")
    parser.add_argument("--no-ppsdo",       action="store_true", help="Do not include PPSDO module.")

    # Introspection.
    parser.add_argument("--no-soc-json",    action="store_true", help="Disable automatic SoC hierarchy JSON generation.")
    parser.add_argument("--doc",    action="store_true", help="Generate SOC ducumentation")

    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        
        # SoC.
        soc = BaseSoC(
            with_bios      = args.with_bios,
            with_jtagbone  = args.with_jtagbone,
            with_cpu_debug = args.with_cpu_debug,
            cpu_firmware   = None if prepare else "firmware/firmware.bin",
            no_ddr         = args.no_ddr,
            no_ppsdo       = args.no_ppsdo
        )

        # soc.add_debug()

        # Always generate SoC hierarchy JSON during prepare pass unless disabled.
        if prepare and not args.no_soc_json:
            soc.print_soc_hierarchy_json()
        
        # Builder.
        builder = Builder(soc, csr_csv="csr.csv", bios_console="lite", libc_mode="full")
        builder.build(run=build)
        
        # Firmware build.
        if prepare:
            linker = {
                True  : "linker_main_ram.ld",
                False : "linker_rom.ld",
            }[args.with_bios]
            
            # Create a makefile fragment with board specific variables
            env_mak = os.path.join("firmware", "env.mak")
            if os.path.exists(env_mak):
                os.remove(env_mak)
            with open(env_mak, "w") as f:
                f.write(f"BUILD_DIR={builder.output_dir}\n")
                f.write(f"TARGET=LIMESDR_USB\n")
                f.write(f"LINKER={linker}\n")
                f.write("BSP_PROJECT_DIR=bsp/LimeSDR_USB\n")
            
            os.system(f"cd firmware && make clean all")

    if args.build:
        output_location = "bitstream/LimeSDR_USB"
        prefix = "LimeSDR-USB_lms7_trx"
        sof_file = os.path.join(builder.gateware_dir, "limesdr_usb.sof")

        os.makedirs(output_location, exist_ok=True)
        print(f"Copying limesdr_usb.sof to {output_location}...")
        shutil.copyfile(sof_file, os.path.join(output_location, prefix + ".sof"))

        for fmt in ["rbf", "jic", "pof"]:
            print(f"Generating {fmt.upper()} file...")
            try:
                subprocess.run(["quartus_cpf", "-c", f"gateware/board_specific/limesdr_usb/limesdr_usb_{fmt}.cof"], check=True)
            except Exception as e:
                print(f"Error generating {fmt.upper()}: {e}")

        print("Generating SVF files...")
        try:
            output_sof = os.path.join(output_location, prefix + ".sof")
            output_jic = os.path.join(output_location, prefix + ".jic")
            svf_file   = os.path.join(output_location, prefix + ".svf")
            svf_flash_file = os.path.join(output_location, prefix + "_flash.svf")

            # SVF for SRAM (load)
            print("Generating SVF for SRAM...")
            subprocess.run(["quartus_cpf", "-c", "-q", "12.0MHz", "-g", "3.3", "-n", "p", output_sof, svf_file], check=True)

            # SVF for Flash (flash)
            # The .jic-derived SVF only contains flash operations (erase/program/
            # verify) and expects the Serial Flash Loader (SFL) to be already
            # configured into the FPGA. openFPGALoader cannot do that for the
            # EP4CE40, so we prepend the committed, device-specific SFL
            # configuration SVF to obtain a single self-contained flash SVF.
            print("Generating SVF for Flash...")
            if not os.path.exists(FLASH_SFL_SVF):
                raise FileNotFoundError(
                    f"Missing SFL configuration SVF '{FLASH_SFL_SVF}'. "
                    "Regenerate it with 'python3 tools/generate_sfl_svf.py'."
                )

            svf_flash_ops_file = os.path.join(output_location, prefix + "_flash_ops.svf")

            # Flash operations (erase/program/verify through the SFL).
            subprocess.run(["quartus_cpf", "-c", "-q", "12.0MHz", "-g", "3.3", "-n", "p", output_jic, svf_flash_ops_file], check=True)

            # Concatenate committed SFL configuration + flash ops into a single
            # self-contained flash SVF.
            with open(svf_flash_file, "w") as out_f:
                for part in (FLASH_SFL_SVF, svf_flash_ops_file):
                    with open(part) as in_f:
                        shutil.copyfileobj(in_f, out_f)

            # Remove intermediate SVF.
            if os.path.exists(svf_flash_ops_file):
                os.remove(svf_flash_ops_file)
        except Exception as e:
            print(f"Error generating SVF: {e}")

        print("Bitstream generation completed.")

    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream("bitstream/LimeSDR_USB/LimeSDR-USB_lms7_trx.svf")

    if args.flash:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0, "bitstream/LimeSDR_USB/LimeSDR-USB_lms7_trx_flash.svf")

    # Generate Litex Documentation files and if --doc option is used build also
    build_name = soc.build_name.replace("_", "-")
    soc.generate_documentation(build_name, build_html=args.doc)

if __name__ == "__main__":
    main()
