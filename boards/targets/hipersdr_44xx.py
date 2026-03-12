#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from tools.patches import LiteXMemoryPatcher
# Apply LiteX Memory Patch
LiteXMemoryPatcher(
    min_width   = 0,
    max_depth   = 64,
    limit       = None,
    ram_style   = "distributed"
).patch()

import os
import sys
import argparse

import litepcie.frontend.dma

from litei2c import LiteI2C
from litei2c.phy.generic import LiteI2CPHYCore
from liteiclink.serdes.gth4_ultrascale import GTH4QuadPLL
from liteiclink.serdes.gth4_ultrascale import GTH4
from litejesd204b.common import JESD204BPhysicalSettings, JESD204BTransportSettings, JESD204BSettings
from migen import *

from litex.gen import *
from migen.genlib.io import DifferentialInput
from migen.genlib.resetsync import AsyncResetSynchronizer

from boards.platforms import hipersdr_44xx_platform
from boards.platforms import hipersdr_44xx_v2_platform

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from litex.soc.integration.soc      import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock     import *
from litex.soc.cores.led       import LedChaser
from litex.soc.cores.icap      import ICAP
from litex.soc.cores.xadc      import XADC
from litex.soc.cores.dna       import DNA
from litex.soc.cores.gpio      import GPIOOut
from litex.soc.cores.spi_flash import USSPIFlash
from litex.soc.cores.bitbang   import I2CMaster
from litex.soc.cores.spi       import SPIMaster

from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP

from litepcie.phy.usppciephy import USPPCIEPHY
from litejesd204b.core import LiteJESD204BCoreTX, LiteJESD204BCoreRX, LiteJESD204BCoreControl

from litescope import LiteScopeAnalyzer

from gateware.aux import AUX
from gateware.helpers import write_module_hierarchy_json
from gateware.xtrx_rfsw import xtrx_rfsw
from gateware.LimeTop  import LimeTop


# Constants ----------------------------------------------------------------------------------------

STRM0_FPGA_RX_RWIDTH = 64    # Stream PC->FPGA, rd width
STRM0_FPGA_TX_WWIDTH = 64    # Stream FPGA->PC, wr width
LMS_DIQ_WIDTH        = 12
TX_IN_PCT_HDR_SIZE   = 16
TX_PCT_SIZE          = 16384  # TX packet size in bytes
TX_N_BUFF            = 4     # N 4KB buffers in TX interface (2 OR 4)

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()
        self.cd_afe    = ClockDomain()
        self.cd_jesd_freerun = ClockDomain()

        self.cd_lms_rx = ClockDomain()
        self.cd_lms_tx = ClockDomain()

        self.cd_fpga_sysref = ClockDomain()
        self.cd_fpga_1pps   = ClockDomain()

        # # #

        # Clk / Rst.
        clk125 = ClockSignal("pcie")
        rst125 = ResetSignal("pcie")

        # PLL.
        self.pll = pll = USMMCM(speedgrade=-2)
        self.comb += pll.reset.eq(rst125)
        pll.register_clkin(clk125, 250e6)
        pll.create_clkout(self.cd_idelay, 200e6)
        pll.create_clkout(self.cd_afe, 500e6)
        pll.create_clkout(self.cd_jesd_freerun, 100e6)

        self.pll_sys = pll_sys = USPLL(speedgrade=-2)
        pll_sys.register_clkin(clk125, 250e6)
        pll_sys.create_clkout(self.cd_sys,    sys_clk_freq)

        # TODO: these do nothing for now, currently rely on manual constraints
        #       to make these work, add_period_constraint commands should be used first
        #       to establish clock names used in these commands
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_idelay.clk)
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_afe.clk)


        # IDelayCtrl.
        #self.idelayctrl = USPIDELAYCTRL(self.cd_idelay, self.cd_sys)

        fpga_sysref = platform.request("FPGA_SYSREF")
        self.specials += [Instance("IBUFDS",
            i_I   = fpga_sysref.p,
            i_IB  = fpga_sysref.n,
            o_O   = self.cd_fpga_sysref.clk,
        )]

        fpga_1pps_pads = platform.request("FPGA_1PPS")
        self.specials += [Instance("IBUFDS",
            i_I   = fpga_1pps_pads.p,
            i_IB  = fpga_1pps_pads.n,
            o_O   = self.cd_fpga_1pps.clk,
        )]





# LMS Control CSR----------------------------------------------------------------------------------------
class CNTRL_CSR(LiteXModule):
    def __init__(self, ndmas, nuart):
        self.cntrl          = CSRStorage(512, 0)
        self.enable         = CSRStorage()
        self.test           = CSRStorage(32)
        self.ndma           = CSRStatus(4, reset=ndmas)
        self.enable_both    = CSRStorage()
        self.nuart          = CSRStatus(4, reset=nuart)

        # Create event manager for interrupt
        self.ev = EventManager()
        self.ev.cntrl_isr = EventSourceProcess()
        self.ev.finalize()

        # Trigger interrupt when cntrl register is written
        self.comb += self.ev.cntrl_isr.trigger.eq(self.cntrl.re)

# fpgacfg
class fpgacfg_csr(LiteXModule):
    def __init__(self,gold=False):
        # TODO: implement some sort of version increment mechanism
        #       or redo version storage entirely (maybe move to firmware)
        self.board_id       = CSRStatus(16, reset=27)
        self.reserved_03    = CSRStorage(16, reset=0)
        self.reserved_04    = CSRStorage(16, reset=0)
        self.reserved_05    = CSRStorage(16, reset=0)
        self.reserved_06    = CSRStorage(16, reset=0)
        if gold:
            self.major_rev      = CSRStatus(16, reset=0xDEAD)
            self.compile_rev    = CSRStatus(16, reset=0xDEAD)
        else:
            self.major_rev      = CSRStatus(16, reset=3)
            self.compile_rev    = CSRStatus(16, reset=6)
        self.channel_cntrl  = CSRStorage(fields=[
            CSRField("ch_en", size=2, offset=0, values=[
                ("``2b01", "Channel A"),
                ("``2b10", "Channel B"),
                ("``2b11", "Channels A and B")
            ], reset=0)
        ])
        self.TCXO_EN = CSRStorage(1, reset=1,
            description="TCXO Enable: 0: Disabled, 1: Enabled."
        )
        self.EXT_CLK = CSRStorage(1, reset=0,
            description="CLK source select: 0: Internal, 1: External."
        )

# periphcfg
class periphcfg_csr(LiteXModule):
    def __init__(self):
        self.BOARD_GPIO_OVRD        = CSRStorage(16, reset=0)
        self.BOARD_GPIO_RD          = CSRStorage(16, reset=0)
        self.BOARD_GPIO_DIR         = CSRStorage(16, reset=0)
        self.BOARD_GPIO_VAL         = CSRStorage(16, reset=0)
        self.PERIPH_INPUT_SEL_0     = CSRStorage(16, reset=0)
        self.PERIPH_INPUT_RD_0      = CSRStorage(16, reset=0)
        self.PERIPH_INPUT_RD_1      = CSRStorage(16, reset=0)
        self.PERIPH_OUTPUT_OVRD_0   = CSRStorage(16, reset=0)
        self.PERIPH_OUTPUT_VAL_0    = CSRStorage(16, reset=0)
        self.PERIPH_OUTPUT_OVRD_1   = CSRStorage(16, reset=0)
        self.PERIPH_OUTPUT_VAL_1    = CSRStorage(16, reset=0)
        self.PERIPH_EN              = CSRStorage(16, reset=0)
        self.PERIPH_SEL             = CSRStorage(16, reset=0)


# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    SoCCore.csr_map = {
        # SoC.
        "uart"        : 0,
        "icap"        : 1,

        # PCIe.
        "pcie_phy"    : 2, #10
        "pcie_msi"    : 3, #11
        "pcie_dma0"   : 5, #12
        "PCIE_UART0"  : 13,
        "PCIE_UART1"  : 14,

        "flash"       : 15,  # 10
        "xadc"        : 16,  # 11
        "dna"         : 17,  # 12

        # XTRX.
        "i2c0"        : 18,
        "i2c1"        : 19,
        "i2c2"        : 20,

        "spimaster"   : 21,
        "spimaster1"  : 22,

        # CNTRL
        "CNTRL"       : 26,
        "afe"         : 27,

        # Analyzer.
        "analyzer"    : 31,

    }

    def __init__(self, board="hypersdr_44xx", sys_clk_freq=int(300e6), cpu_type="picorv32",
        with_bios             = False,
        with_uartbone         = False,
        with_cpu              = True, cpu_firmware=None,
        with_jtagbone         = True,
        with_bscan            = False,
        with_fft              = False,
        flash_boot            = False,
        gold_img              = False,
        firmware_flash_offset = 0x220000,
    ):

        # Platform ---------------------------------------------------------------------------------
        platform = {
            "hipersdr_44xx"         : hipersdr_44xx_platform.Platform(),
            "hipersdr_44xx_v2"      : hipersdr_44xx_v2_platform.Platform(),
        }[board]

        if gold_img:
            platform.toolchain.additional_commands += platform.gold_img_commands
        else:
            platform.toolchain.additional_commands += platform.user_img_commands

        platform.name        = "hipersdr_44xx"
        platform.vhd2v_force = False

        # Enable Compressed Instructions.
        VexRiscvSMP.with_rvc = True
        # Enable JTAG.
        if with_bscan:
            assert cpu_type == "vexriscv_smp"
            VexRiscvSMP.privileged_debug     = True
            VexRiscvSMP.hardware_breakpoints = 4

        # SoCCore ----------------------------------------------------------------------------------
        assert cpu_type in ["vexriscv", "vexriscv_smp", "picorv32", "fazyrv", "firev"]

        cpu_variant = {
            "vexriscv"     : "minimal",
            "vexriscv_smp" : "standard",
            "picorv32"     : "minimal",
            "fazyrv"       : "standard",
            "firev"        : "standard",
        }[cpu_type]

        if with_bios:
            integrated_rom_size      = 0x6800
            integrated_rom_init      = []
            integrated_main_ram_size = 0x6800
            #integrated_main_ram_size = 0x2E800
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
        else:
            integrated_rom_size      = 0x6800
            integrated_rom_init      = [0] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_main_ram_size = 0
            integrated_main_ram_init = []

        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = f"LiteX SoC on {board.capitalize()} XTRX ",
            ident_version            = True,
            cpu_type                 = cpu_type,
            cpu_variant              = cpu_variant,
            integrated_rom_size      = integrated_rom_size,
            integrated_rom_init      = integrated_rom_init,
            integrated_sram_ram_size = 0x0200,
            integrated_main_ram_size = integrated_main_ram_size,
            integrated_main_ram_init = integrated_main_ram_init,
            with_uart=False,  # needs to be false to be able to add uart manually
            #with_uartbone            = with_uartbone,
            #uart_name                = {True: "crossover", False:"serial"}[with_uartbone],
        )

        # Avoid stalling CPU at startup.
        #self.uart.add_auto_tx_flush(sys_clk_freq=sys_clk_freq, timeout=1, interval=128)

        serial_signals = Record(layout=[("tx", 1), ("rx", 1)])
        self.add_uart(name="uart", uart_name={True: "crossover", False:"serial"}[with_uartbone], baudrate=115200, fifo_depth=16, with_dynamic_baudrate=False, uart_pads=serial_signals)

        # Define platform name constant.
        self.add_constant(platform.name.upper())


        self.fpgacfg = fpgacfg_csr(gold=gold_img)

        # FPGA Cfg ---------------------------------------------------------------------------------
        #from gateware.fpgacfg import FPGACfg
        #self.fpgacfg  = FPGACfg(platform,
        #    board_id    = 27,
        #    major_rev   = 3 if not gold_img else 0xDEAD,
        #    compile_rev = 0 if not gold_img else 0xDEAD,
        #    pads        = None
        #)
        ## TODO: maybe it's possible to implement check automatically?
        #soc_has_timesource = True,
        #self.comb += self.fpgacfg.pwr_src.eq(0)
        self.periphcfg = periphcfg_csr()
        self.CNTRL = CNTRL_CSR(1, 2)
        self.irq.add("CNTRL")

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform, sys_clk_freq)

        # JTAGBone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()
            platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
            platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # JTAG CPU Debug ---------------------------------------------------------------------------
        if with_bscan:
            self.add_jtag_cpu_debug()

        # Leds -------------------------------------------------------------------------------------
        # self.led_pads = platform.request_all("user_led")
        self.led_placeholder = Signal()
        if gold_img:
            self.leds = LedChaser(
                pads         = self.led_placeholder,
                period       = 2,
                sys_clk_freq = sys_clk_freq
            )
            self.comb += platform.request("user_led",0).eq(self.led_placeholder)
            self.comb += platform.request("user_led",1).eq(self.led_placeholder)
        else:
            leds = LedChaser(
                pads         = platform.request_all("user_led"),
                period       = 1,
                sys_clk_freq = int(15.36e6)
            )
            leds = ClockDomainsRenamer("fpga_1pps")(leds)
            self.leds = leds

        # ICAP -------------------------------------------------------------------------------------
        #self.icap = ICAP()
        #self.icap.add_reload()
        #self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # SPIFlash ---------------------------------------------------------------------------------
        if flash_boot:
            from litespi.modules import N25Q256A
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="1x", module=N25Q256A(Codes.READ_1_1_1), with_master=False)

            # Add ROM linker region --------------------------------------------------------------------
            self.bus.add_region("flash", SoCRegion(
                origin = self.bus.regions["spiflash"].origin + firmware_flash_offset,
                size   = 0x40000, # 256kB
                linker = True)
            )
            # Automatically jump to pre-initialized firmware.
            self.add_constant("FLASH_BOOT_ADDRESS", self.bus.regions["flash"].origin)
        else:
            # Automatically jump to pre-initialized firmware.
            self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["main_ram"])
            #self.flash_cs_n = GPIOOut(platform.request("flash_cs_n"))
            # TODO: change this to add_spi_flash in the future
            self.flash      = USSPIFlash(sys_clk_freq, 1e6)

        # XADC -------------------------------------------------------------------------------------
        #self.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        #self.dna = DNA()
        #self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # PCIe -------------------------------------------------------------------------------------
        # cd_pcie uses 250MHz clock
        self.pcie_phy = USPPCIEPHY(platform, platform.request(f"pcie_x4"),
            speed="gen4",
            data_width  = 256,
            bar0_size   = 0x40000,
            ip_name="pcie4c_uscale_plus",
            cd          = "sys",
        )
        self.pcie_phy.update_config({
            "vendor_id"                     : "2058",
            "PF0_DEVICE_ID"                 : "001F",
            "PF0_REVISION_ID"               : "0001",
            "PF0_SUBSYSTEM_ID"              : "0001",
            "PF0_SUBSYSTEM_VENDOR_ID"       : "2058",
            "pf0_base_class_menu"           : "Wireless_controller",
            "pf0_sub_class_interface_menu"  : "Other_type_of_wireless_controller",
            "pf0_class_code_base"           : "0D",
            "pf0_class_code_sub"            : "80",
            "pf0_bar0_64bit"                : "true",
            "en_gt_selection"               : "true",
            "select_quad"                   : "GTH_Quad_224",
            "PHY_LP_TXPRESET"               : "5",
            "axisten_freq"                  : "250",
            "axisten_if_width"              : "256_bit",
            "AXISTEN_IF_RC_STRADDLE"        : "false"
            }
        )
        self.add_pcie(phy=self.pcie_phy, address_width=64, data_width=self.pcie_phy.data_width, ndmas=1,
            with_dma_buffering    = True,
            dma_buffering_depth   = 16384,
            max_pending_requests  = 16,
            with_dma_loopback     = False,
            with_dma_synchronizer = False,
            with_msi              = True
        )
        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.pcie_phy.cd_pcie.clk)

        # I2C Bus1 ---------------------------------------------------------------------------------
        # PMIC-AFE_DCDC   (LP8758       @ 0x60)
        self.i2c0 = LiteI2C(sys_clk_freq=sys_clk_freq, pads=platform.request("i2c", 0), clock_domain="sys")

        # I2C Bus2 ---------------------------------------------------------------------------------
        # Temp sensor     (TMP114NB     @ 0x4E)
        # Clock generator (LMK05318B    @ 0x65)
        self.i2c1 = LiteI2C(sys_clk_freq=sys_clk_freq, pads=platform.request("i2c", 1), clock_domain="sys")

        # I2C Bus3 ---------------------------------------------------------------------------------
        # I/O expander    (TCA6424AR    @ 0x22)
        # I/O expander    (TCA6424AR    @ 0x23)
        # Temp sensor     (TMP114NB     @ 0x4E)
        self.i2c2 = LiteI2C(sys_clk_freq=sys_clk_freq, pads=platform.request("i2c", 2), clock_domain="sys")

        # I2C Bus4 (I2C_REF/I2C5 in sch)-----------------------------------------------------------------
        # I/O expander    (TCA6424AR    @ 0x22)
        # I/O expander    (TCA6424AR    @ 0x23)
        # Temp sensor     (TMP114NB     @ 0x4E)
        # VCTCXO DAC      (DAC8050      @ 0x48) (A0 pin connected to GND)

        self.i2c3 = LiteI2C(sys_clk_freq=sys_clk_freq, pads=platform.request("i2c", 3), clock_domain="sys")

        # PMIC-FPGA --------------------------------------------------------------------------------
        # Buck0: 1.0V VCCINT + 1.0V MGTAVCC.
        # Buck1: 1.8V/3.3V VCCIO (DIGPRVDD2/DIGPRVDD3/DIGPRPOC + VDD18_TXBUF of LMS + Bank 0/14/16/34/35 of FPGA).
        # Buck2: 1.2V MGTAVTT + 1.2V VDLMS (VDD12_DIG / VDD_SPI_BUF / DVDD_SXR / DVDD_SXT / DVDD_CGEN).
        # Buck3: 1.8V VCCAUX  + 1.8V VDLMS (VDD18_DIG).

        # PMIC-LMS ---------------------------------------------------------------------------------
        # Buck0: +2.05V (used as input to 1.8V LDO for LMS analog 1.8V).
        # Buck1: +3.3V rail.
        # Buck2: +1.75V (used as input to 1.4V LDO for LMS analog 1.4V).
        # Buck3: +1.5V  (used as input to 1.25V LDO for LMS analog 1.25V).

        # Aux -------------------------------------------------------------------------------------
        #self.aux = AUX(platform.request("aux"))

        # Timing Constraints/False Paths -----------------------------------------------------------
        #for i in range(4):
        #    platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks        dna_clk] -asynchronous")
        #    platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks       jtag_clk] -asynchronous")
        #    platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks       icap_clk] -asynchronous")

        #platform.toolchain.build.vivado_opt_directive(f"Explore")
        #platform.toolchain.pre_placement_commands.append(f"set_property strategy Performance_Explore [get_runs impl_1]")

        # LimeTOP ----------------------------------------------------------------------------------
        self.lime_top = LimeTop(self, platform,
           # Configuration.
           double_channels_mode = True,
           LMS_DIQ_WIDTH        = 12,
           sink_width           = self.pcie_phy.data_width,
           sink_clk_domain      = self.crg.cd_sys.name,
           source_width         = self.pcie_phy.data_width,
           source_clk_domain    = self.crg.cd_sys.name,
           rx_sys_clk_domain    = self.crg.cd_sys.name,
           TX_N_BUFF            = TX_N_BUFF,
           TX_PCT_SIZE          = TX_PCT_SIZE,
           TX_IN_PCT_HDR_SIZE   = 16,
           tx_buffer_size       = self.pcie_phy.data_width * 4, #minimum tx cdc input buffer depth

           with_lms7002         = False,
           phy_tx_source_clk    = self.crg.cd_afe.name,
           phy_rx_sink_clk      = self.crg.cd_afe.name,
           with_rx_tx_top       = True,
           with_fft             = with_fft,

           # FPGACFG.
           #  TODO : change board ID?
           board_id             = 31,
           # GOLD image can be recognized by 0xDEAD in major and compile revisions
           major_rev            =  3 if not gold_img else 0xDEAD,
           compile_rev          =  0 if not gold_img else 0xDEAD,
           revision_pads        = None,
           # TODO: maybe it's possible to implement check automatically?
           soc_has_timesource   = False,
        )

        self.comb += self.lime_top.source.connect(self.pcie_dma0.sink, keep={"valid", "ready", "last", "data"}),

        ## PCIE DMA -> TX Path -> LMS7002 Pipeline.
        self.comb += [
           self.pcie_dma0.source.connect(self.lime_top.sink, omit=["ready"]),
           self.pcie_dma0.source.ready.eq((self.lime_top.sink.ready & self.lime_top.fpgacfg.rx_en) | ~self.pcie_dma0.reader.enable),
        ]

        self.comb += self.lime_top.rxtx_top.tx_path.ext_reset_n.eq(self.pcie_dma0.reader.enable)

        #self.comb += [
        #    self.lime_top.dma_tx.valid.eq(self.pcie_dma0.source.valid),
        #    self.lime_top.dma_tx.last.eq(self.pcie_dma0.source.last),
        #    self.lime_top.dma_tx.data.eq(self.pcie_dma0.source.data),
        #    self.pcie_dma0.source.ready.eq((self.lime_top.dma_tx.ready & self.lime_top.lms7002.tx_en.storage) | ~self.pcie_dma0.reader.enable),
        #]

        #self.comb += self.lime_top.tx_path.RESET_N.eq(self.pcie_dma0.reader.enable)

        # LMS SPI
        #self.lms_spi = SPIMaster(
        #    pads=platform.request("lms7002m_spi"),
        #    data_width=32,
        #    sys_clk_freq=sys_clk_freq,
        #    spi_clk_freq=1e6
        #)

        self.lms8001_spi_pads = platform.request("lms8001_spi")
        self.add_spi_master(name="spimaster", pads=self.lms8001_spi_pads, data_width=32, spi_clk_freq=1e6)

        # AFE SPI
        self.afe_spia_pads = platform.request("afe_spi", 0)
        self.add_spi_master(name="spimaster1", pads=self.afe_spia_pads, data_width=24, spi_clk_freq=10e6)

        # ADF SPI
        # SPI master expects a miso signal, but adf spi does not use miso
        # doing all this to add a dummy miso signal
        self.adf_spi_pads = Record(layout=[("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
        self.ref_spi_pads = platform.request("ref_spi")
        self.comb += [
            self.ref_spi_pads.clk.eq(self.adf_spi_pads.clk),
            self.ref_spi_pads.cs_n.eq(self.adf_spi_pads.cs_n),
            self.ref_spi_pads.mosi.eq(self.adf_spi_pads.mosi),
            self.adf_spi_pads.miso.eq(0)
        ]
        self.add_spi_master(name="spimaster_adf", pads=self.adf_spi_pads, data_width=24, spi_clk_freq=10e6)


        # Power control
        from gateware.board_specific.hipersdr_44xx.pwr_ctrl import PWRCtrl
        self.pwr_control_pads = platform.request("pwr_ctrl")
        self.pwr_control = PWRCtrl(platform, self.pwr_control_pads)

        # GPIO33 pins
        self.gpio33_pads = platform.request("gpio33")
        # Turn ON FANS
        self.comb += [
            self.gpio33_pads[0].eq(1), #
            self.gpio33_pads[1].eq(1),
        ]


        #AFE7901
        from gateware.LimeDFB.afe79xx.afe79xx import afe79xx
        self.afe_pads = platform.request("afe79xx_serdes_x4")
        self.afe = afe79xx(soc=self,
                           platform=platform,
                           pads=self.afe_pads,
                           with_debug=False,
                           s_clk_domain=self.crg.cd_fpga_1pps.name,
                           m_clk_domain=self.crg.cd_fpga_1pps.name,
                           demux_clk_domain=self.crg.cd_afe.name,
                           demux=True,
                           resampling_stages=0)

        self.comb += self.afe.jesd_freerun_clk.eq(self.crg.cd_jesd_freerun.clk)

        # self.debug_counter = Signal(8, reset=0)
        # # equivalent to self.sync.demux_clk_domain. That way does not work because
        # # demux_clk_domain is not a clock domain, but a string.
        # clock = self.crg.cd_afe.name
        # sync_domain = getattr(self.sync, clock)
        # sync_domain += [
        #     If((self.afe.source.valid == 1) & (self.afe.source.ready == 1),[
        #     self.debug_counter.eq(self.debug_counter + 1),
        #     ])
        # ]
        # self.comb += [
        #     self.lime_top.phy_rx_sink.valid.eq(self.afe.source.valid),
        #     self.afe.source.ready.eq(self.lime_top.phy_rx_sink.ready),
        # ]
        #
        # for i in range(8):
        #     start = i*16
        #     end = start+16
        #     self.comb += [
        #         self.lime_top.phy_rx_sink.data[start:end].eq(Cat(self.debug_counter,Constant(0,4),Constant(i,4))),
        #     ]

        self.comb += [
            self.afe.source.connect(self.lime_top.phy_rx_sink),
            self.lime_top.phy_tx_source.connect(self.afe.sink)
        ]

        self.comb +=[
            self.afe.rx_en.eq(self.lime_top.fpgacfg.rx_en),
            self.afe.tx_en.eq(self.lime_top.fpgacfg.rx_en),
        ]

        #self.afe_pads = platform.request("afe79xx_serdes_x4")
        #self.afe = afe79xx(platform, self.afe_pads, False)

        #self.comb += self.afe.source.connect(self.pcie_dma0.sink, keep={"valid", "ready", "last", "data"})
        #self.comb += self.pcie_dma0.source.connect(self.afe.sink, keep={"valid", "ready", "last", "data"})


        self.clk_ctrl_pads = platform.request("clk_ctrl")
        self.comb += self.clk_ctrl_pads.clk_src_sel.eq(1)

        self.lmk_ctrl_pads = platform.request("lmk_ctrl")

        self.comb += [
            self.lmk_ctrl_pads.lmk_syncn.eq(1),
            self.lmk_ctrl_pads.lmk_finc.eq(0),
            self.lmk_ctrl_pads.lmk_fdet.eq(0),
        ]



        # Interrupt --------------------------------------------------------------------------------
        
        self.irq.add("lime_top")

        #vctcxo_pads = platform.request("vctcxo")
        #self.comb += vctcxo_pads.sel.eq(self.fpgacfg.EXT_CLK.storage)
        #self.comb += vctcxo_pads.en.eq(self.fpgacfg.TCXO_EN.storage)
#
        #rfsw_pads = platform.request("rf_switches")
#
        #self.rfsw_control = xtrx_rfsw(platform, rfsw_pads)
        ##self.comb += rfsw_pads.tx.eq(1)

        #from gateware.AFE7901ABJ import AFE7901ABJCore
        #self.AFE7901ABJ =  AFE7901ABJCore(platform, self.crg.cd_sys.clk, sys_clk_freq)
#
#
        #self.comb += self.pcie_dma0.sink.valid.eq(self.AFE7901ABJ.jesd_rx_core.ready)
        ##self.sync +=If(self.pcie_dma0.sink.valid, self.pcie_dma0.sink.data.eq(self.AFE7901ABJ.))

        # GPIO control -----------------------------------------------------------------------------
        if board == "hipersdr_44xx":
            from gateware.board_specific.hipersdr_44xx.gpio_ctrl import GPIO_Ctrl
            self.gpio_control = GPIO_Ctrl(platform)
        else:
            from gateware.board_specific.hipersdr_44xx.gpio_ctrl_v2 import GPIO_Ctrl_v2
            self.gpio_control = GPIO_Ctrl_v2(platform,
                                             tdd_control_clk_domain=self.crg.cd_fpga_1pps.name)

        self.comb += self.gpio_control.TDDSignal.eq(self.afe.afe_sink.valid)


        self.test_1pps = Signal() #crg.cd_fpga_1pps.
        self.sync.fpga_1pps += [
            self.test_1pps.eq(~self.test_1pps)
        ]
        # Assign regular GPIOs
        self.rx_frontend_enable = platform.request("RX_EN")
        self.comb += [
            self.rx_frontend_enable.eq(self.gpio_control.GPIO.storage[0:4]),
        ]
        self.rxa_attenuator_control = platform.request("RXA_ATTENUATOR_CTRL")
        self.rxb_attenuator_control = platform.request("RXB_ATTENUATOR_CTRL")
        self.rxc_attenuator_control = platform.request("RXC_ATTENUATOR_CTRL")
        self.rxd_attenuator_control = platform.request("RXD_ATTENUATOR_CTRL")
        self.comb += [
            self.rxa_attenuator_control.eq(self.gpio_control.GPIO.storage[4:8]),
            self.rxb_attenuator_control.eq(self.gpio_control.GPIO.storage[8:12]),
            self.rxc_attenuator_control.eq(self.gpio_control.GPIO.storage[12:16]),
            self.rxd_attenuator_control.eq(self.gpio_control.GPIO.storage[16:20]),
        ]

        if board == "hipersdr_44xx":
            self.sw_rx_tddfdd = platform.request("SW_RX_TDDFDD")
            self.comb += [
                self.sw_rx_tddfdd.eq(self.gpio_control.GPIO.storage[20:24]),
            ]

            self.sw_rxtx_control = platform.request("SW_RXTX_CTRL")
            self.comb += [
                self.sw_rxtx_control.eq(self.gpio_control.GPIO.storage[24:28]),
            ]
            self.sw_pa_onoff_control = platform.request("SW_PA_ONOFF_CTRL")
            self.comb += [
                self.sw_pa_onoff_control.eq(self.gpio_control.GPIO.storage[28:32]),
            ]
            self.tx_frontend_pa_vadj_enable = platform.request("TX_EN_VADJ")
            self.tx_frontend_pa_enable = platform.request("TX_EN")
            self.comb += [
                self.tx_frontend_pa_enable.eq(self.gpio_control.GPIO2.storage[4:8]),
                self.tx_frontend_pa_vadj_enable.eq(self.gpio_control.GPIO2.storage[0:4]),
            ]

        else:
            self.sw_rx_tddfdd = platform.request("SW_RX_LB_TDD_FDD")
            self.comb += [
                self.sw_rx_tddfdd.eq(self.gpio_control.GPIO.storage[20:28]),
            ]
            self.sw_rxtx_control = platform.request("SW_RXTX_CTRL")
            self.comb += [
                self.sw_rxtx_control.eq(self.gpio_control.SW_RXTX),
            ]
            self.sw_tx_onoff_control = platform.request("SW_TX_ONOFF_CTRL")
            self.comb += [
                self.sw_tx_onoff_control.eq(self.gpio_control.SW_TX_ONOFF),
            ]
            self.tx_frontend_pa_vadj_enable = platform.request("TX_EN_VADJ")
            self.tx_frontend_pa_enable = platform.request("TX_EN")
            self.comb += [
                self.tx_frontend_pa_enable.eq(self.gpio_control.GPIO2.storage[8:12]),
                self.tx_frontend_pa_vadj_enable.eq(self.gpio_control.GPIO2.storage[4:8]),
            ]

            refctrl_pads = platform.request("refctrl")
            self.comb += refctrl_pads.sel.eq(self.gpio_control.GPIO2.storage[12])

        from gateware.EventManagers.BSPEventManager import BSPEventManager
        self.bsp = BSPEventManager(width=2)

        self.comb += self.bsp.isr_vect[0].eq(self.afe.core_ctrl.fields.afe_init_trigger)
        self.comb += self.bsp.isr_vect[1].eq(self.gpio_control.port_out_value_115.storage[8])  # Connecting PWR_LMS8_NRST bit to bsp isr

        self.irq.add("bsp")


        #TODO: place it in gateware dir
        timings_xdx_filename = "timing.xdc"
        with open(timings_xdx_filename, "w") as f:
            # Write timing constraints.
            f.write("# Renaming generated clocks\n")
            f.write("create_generated_clock -name sys -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT0]\n\n")
            f.write("create_generated_clock -name idelaye -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT1]\n\n")
            f.write("create_generated_clock -name afe -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT2]\n\n")

            f.write("set_clock_groups -name sys_async1 -asynchronous -group [get_clocks sys]\n\n")
            f.write("set_clock_groups -name sys_async2 -asynchronous -group [get_clocks afe]\n\n")
            f.write("set_clock_groups -name 1pps -asynchronous -group [get_clocks fpga_1pps_clk]\n\n")
            # f.write("set_clock_groups -name 1pps_double -asynchronous -group [get_clocks fpga_1pps_double_clk]\n\n")
        self.platform.add_source(timings_xdx_filename)


    # JTAG CPU Debug -------------------------------------------------------------------------------

    def add_jtag_cpu_debug(self):
        from litex.soc.cores.jtag import XilinxJTAG
        self.jtag = jtag = XilinxJTAG(XilinxJTAG.get_primitive(self.platform.device), chain=4)
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


    def add_debug(self):

        analyzer_signals = [
            self.afe.afe_sink.valid,
            self.afe.afe_sink.ready,
            self.afe.afe_source.valid,
            self.afe.afe_source.ready,
            self.afe.afe_sink.data,
        ]

        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 256,
            clock_domain = "fpga_1pps",
            register     = True,
            csr_csv      = "analyzer.csv"
        )
    # SoC hierarchy JSON utilities -----------------------------------------------------------------

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

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on HyperSDR.")
    parser.add_argument("--board",   default="hipersdr_44xx_v2", help="Select HyperSDR board.", choices=["hipersdr_44xx", "hipersdr_44xx_v2"])
    parser.add_argument("--cpu-type", default="vexriscv_smp", help="Select CPU.",
        choices=["vexriscv", "vexriscv_smp", "picorv32", "fazyrv", "firev"]
    )
    parser.add_argument("--with-bscan",            action="store_true",     help="Enable CPU debug over JTAG."),
    parser.add_argument("--build",                 action="store_true",     help="Build bitstream.")
    parser.add_argument("--load",                  action="store_true",     help="Load bitstream.")
    parser.add_argument("--flash",                 action="store_true",     help="Flash bitstream.")
    parser.add_argument("--cable",                 default="ft2232",        help="JTAG cable.")
    parser.add_argument("--driver",                action="store_true",     help="Generate PCIe driver from LitePCIe (override local version).")
    parser.add_argument("--flash-boot",            action="store_true",     help="Write Firmware in Flash instead of RAM.")
    parser.add_argument("--firmware-flash-offset", default=0x220000,        help="Firmware SPI Flash offset.")
    parser.add_argument("--gold",                  action="store_true",     help="Build/Flash golden image instead of user")

    # SoC parameters.
    parser.add_argument("--with-bios",      action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-uartbone",  action="store_true", help="Enable UARTBone.")

    # Examples.
    parser.add_argument("--with-fft",       action="store_true", help="Enable FFT module examples.")

    # Introspection.
    parser.add_argument("--no-soc-json",    action="store_true", help="Disable automatic SoC hierarchy JSON generation.")

    parser.add_argument("--doc", action="store_true", help="Generate SOC ducumentation")

    # Litescope Analyzer Probes.
    probeopts = parser.add_mutually_exclusive_group()
    probeopts.add_argument("--debug", action="store_true", help="Enable Debug probes.")

    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        # SoC.
        soc = BaseSoC(
            board                 = args.board,
            cpu_type              = args.cpu_type,
            with_bios             = args.with_bios,
            with_uartbone         = args.with_uartbone,
            with_cpu              = True,
            cpu_firmware          = None if prepare else "firmware/firmware.bin",
            with_jtagbone         = not args.with_bscan,
            with_bscan            = args.with_bscan,
            with_fft              = args.with_fft,
            flash_boot            = args.flash_boot,
            gold_img              = args.gold,
            firmware_flash_offset = args.firmware_flash_offset,
        )

        if args.debug:
            assert args.with_uartbone or not args.with_bscan
            soc.add_debug()


        # Always generate SoC hierarchy JSON during prepare pass unless disabled.
        if prepare and not args.no_soc_json:
            soc.print_soc_hierarchy_json()

        builder = Builder(soc, csr_csv="csr.csv", bios_console="lite")
        builder.build(run=build,
                        vivado_synth_directive                  = "PerformanceOptimized",
                        vivado_opt_directive                    = "Explore",
                        vivado_place_directive                  = "Explore",
                        vivado_post_place_phys_opt_directive    = "Explore",
                        vivado_route_directive                  = "Explore",
                        vivado_post_route_phys_opt_directive    = "Explore",
                        vivado_max_threads                      = 7,
                      )
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
                f.write("BSP_PROJECT_DIR=bsp/HiperSDR_44xx\n")
            os.system(f"cd firmware && make clean all")
            # os.system(f"cd firmware/hiper/ && make BUILD_DIR={builder.output_dir} TARGET={soc.platform.name.upper()} LINKER={linker} clean all")
            bistream_output_dir = "bitstream/{}".format(soc.get_build_name())
            if not os.path.exists(bistream_output_dir):
                os.makedirs(bistream_output_dir)

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    # Flash Bitstream.
    ## Note: enable_quad option is needed because, bitstream is generated with SPIx4 config mode
    if args.flash:
        if args.gold:
            prog = soc.platform.create_programmer(cable=args.cable)
            prog.flash(0, os.path.join(bistream_output_dir, soc.build_name + "_golden" + ".bin"))
        else: #user img
            # TODO: move user img address to a global variable somewhere instead of hardcoding
            prog = soc.platform.create_programmer(cable=args.cable)
            #prog.flash("0x00220000", os.path.join(bistream_output_dir, soc.build_name + "_user" + ".bin"), verbose_level="2", enable_quad=True)
            prog.flash("0x00310000", os.path.join(bistream_output_dir, soc.build_name + "_user" + ".bin"),
                       verbose_level="2")

    # Flash Firmware.
    if args.flash_boot and args.flash:
        from litex.soc.software.crcfbigen import insert_crc
        insert_crc("firmware/firmware.bin", fbi_mode=True, o_filename="firmware/firmware.fbi", little_endian=True)
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(args.firmware_flash_offset, "firmware/firmware.fbi")


    # Generate Litex Documentation files and if --doc option is used build also
    build_name = soc.build_name.replace("_", "-")
    soc.generate_documentation(build_name, build_html=args.doc)

if __name__ == "__main__":
    main()
