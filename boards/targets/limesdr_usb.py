#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

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

# Constants ----------------------------------------------------------------------------------------

FPGA_to_Host_data_width = 64 # bus width connecting FX3 and Limetop
Host_to_FPGA_data_width = 64 # bus width connecting FX3 and Limetop
wfm_data_width          = 32 # bus width connecting FX3 and wfmplayer
Tx_max_buf_packets      = 16      # maximum number of buffered tx packets in Limetop (any size)
Tx_packet_buf_size      = 16384   # total size (in bytes) of tx packet buffer in Limetop

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
                 gold_img          = False,
                 cpu_firmware   = None,
                 with_jtagbone     = False):
        platform = limesdr_usb.Platform()
        platform.name        = "limesdr_usb"
        platform.vhd2v_force = False
        platform.add_platform_command("set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008")

        if with_bios:
            integrated_rom_size      = 0x4000
            integrated_rom_init      = []
            integrated_main_ram_size = 0x4000
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
        else:
            integrated_rom_size      = 0x4000
            integrated_rom_init      = [0] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_main_ram_size = 0
            integrated_main_ram_init = []

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-USB",
            ident_version            = True,
            cpu_type                 = "vexriscv",
            cpu_variant              = "minimal",
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
        if with_jtagbone:
            self.add_jtagbone()
            platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
            platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # FX3
        self.FX3 = FX3(platform=platform,
                       pads=platform.request("FX3"),
                       vendor="altera",
                       EP01_0_rwidth = Host_to_FPGA_data_width,
                       EP01_1_rwidth = wfm_data_width,
                       EP81_wwidth   = FPGA_to_Host_data_width
                       )

        # LMS SPI -----------------------------------------------------------------------------------
        # LMS spi declared outside PSS, because the current firmware driver expects that
        self.add_spi_master(name="spimaster", pads=platform.request("FPGA_SPI0"), data_width=32, spi_clk_freq=1e6)

        # PSS (Peripheral Support Subsystem)
        self.pss = PSS_LimeSDR_Usb(self, platform, sys_clk_freq, pll_ref_clk=self.crg.fx3_pclk, add_ddr_modules=True, wfm_infifo_usedw_width=self.FX3.ep01_0_rdusedw_width)

        # LimeTop -----------------------------------------------------------------------------------
        self.limetop  = LimeTop(self,
                                platform             = platform,
                                vendor               = "altera",
                                family               = "cycloneIV",
                                sink_width           = Host_to_FPGA_data_width,
                                source_width         = FPGA_to_Host_data_width,
                                rx_fixed_packet_size = True,
                                TX_N_BUFF            = Tx_max_buf_packets,
                                TX_MAX_PCT_SIZE      = Tx_packet_buf_size,
                                TX_WITHTXIQ_MUX      = True,
                                # FPGACFG.
                                board_id             = 0x0011,
                                major_rev            = MajorRevision,
                                compile_rev          = CompileRevision,
                                revision_pads        = platform.request("revision"),

                                with_event_manager   = False,
                                with_clk_cfg_irq     = False,
                                )

        self.comb += [
            self.FX3.data_source.connect  (self.limetop.sink),
            self.limetop.source.connect   (self.FX3.data_sink),
            self.FX3.data_sink_clr.eq     (~self.limetop.fpgacfg.rx_en),
            self.FX3.data_source0_clr.eq  (~self.limetop.fpgacfg.rx_en),
            self.limetop.rxtx_top.tx_path.ext_reset_n.eq(self.limetop.fpgacfg.rx_en),
        ]
        # WFMPlayer <-> lms7002_top
        self.comb += [
            self.limetop.lms7002_top.wfm_sink_l.eq(self.pss.wfm_player.diq_l),
            self.limetop.lms7002_top.wfm_sink_h.eq(self.pss.wfm_player.diq_h),
        ]
        # FX3 <-> WFMPlayer
        # NOTE: both FX3 and WFMPlayer need usedw signals from their fifos to operate properly
        #       LiteX AsyncFifo does not have two level outputs, so two SyncFIFOs have to be used,
        #       one for each clock domain.
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

            # LiteScope Analyzer Probes --------------------------------------------------------------------
    def add_debug(self):
        reset_sig = Signal()
        self.comb += reset_sig.eq(ResetSignal("lms_tx"))
        analyzer_signals = []
        analyzer_signals += [
            self.pss.wfm_player.wfm_load,
            self.pss.wfm_player.wfm_play,
            self.pss.wfm_player.diq_h,
            self.pss.wfm_player.diq_l,
            self.pss.wfm_player.wfm_ch_en,
            self.pss.wfm_player.sink_usedw,
            # self.pss.wfm_player.sink_usedw_debug,
            # self.pss.wfm_player.sink_debug,
            # self.FX3.data_source_1
            # self.pss.wfm_player.sink
            # reset_sig,
            # self.pss.wfm_player.diq_h,
            # self.pss.wfm_player.diq_l,
            # # self.pss.wfm_player.diq_h_full,
            # self.pss.wfm_player.diq_l_full,
            # self.limetop.lms7002_top.mux0_reg_l,
            # self.limetop.lms7002_top.mux0_reg_h,
            # self.limetop.lms7002_top.mux1_reg_l,
            # self.limetop.lms7002_top.mux1_reg_h,
            # self.limetop.lms7002_top.mux2_reg_l,
            # self.limetop.lms7002_top.mux2_reg_h,
            # self.limetop.lms7002_top.txiq_mux_sel_sync,
            # self.limetop.lms7002_top.txiq_mux_sel.storage,
        ]
        # Only import LiteScope when it's actually needed
        from litescope import LiteScopeAnalyzer
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 128,
            clock_domain = "lms_tx",
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
    parser.add_argument("--with-bios",     action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-jtagbone", action="store_true", help="Enable JTAGBone.")

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
            with_bios     = args.with_bios,
            with_jtagbone = args.with_jtagbone,
            cpu_firmware  = None if prepare else "firmware/firmware.bin"
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
