#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import argparse

import litepcie.frontend.dma
from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from boards.platforms import limesdr_xtrx_platform, fairwaves_xtrx_platform

from litex.soc.interconnect     import stream
from litex.soc.interconnect.csr import *
from litex.soc.interconnect.axi import AXIStreamInterface

from litex.soc.integration.soc      import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock     import *
from litex.soc.cores.led       import LedChaser
from litex.soc.cores.icap      import ICAP
from litex.soc.cores.xadc      import XADC
from litex.soc.cores.dna       import DNA
from litex.soc.cores.gpio      import GPIOOut
from litex.soc.cores.spi_flash import S7SPIFlash
from litex.soc.cores.bitbang   import I2CMaster
from litex.soc.cores.spi       import SPIMaster

from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP

from litepcie.phy.s7pciephy import S7PCIEPHY

from litescope import LiteScopeAnalyzer

from gateware.aux      import AUX
from gateware.GpioTop  import GpioTop
from gateware.LimeTop  import LimeTop

from litepcie.software import generate_litepcie_software, generate_litepcie_software_headers

# TODO: REMOVE WHEN NO LONGER REQUIRED
# ------------------   Monkey patching
from litepcie.common     import *
class mod_LitePCIeDMABuffering(LiteXModule):
    """LitePCIe DMA Buffering

    Optional DMA buffering with dynamically configurable depth.

    For some applications (Software Defined Radio, Video, ...), the user module consuming the data
    from the DMA Reader works at fixed rate and does not handle backpressure. (The same also applies
    to the user module generating the data to the DMA Writer). Since the PCIe bus is shared, gaps
    appears in the streams and our Writes/Reads can't be absorbed/produced at a fixed rate. A minimum
    of buffering is needed to make sure the gaps are smoothed and not propagated to user modules.
    """
    def __init__(self, data_width, with_writer, with_reader, writer_depth, reader_depth, dynamic_depth=True):
        self.sink        = stream.Endpoint(dma_layout(data_width))
        self.source      = stream.Endpoint(dma_layout(data_width))

        self.next_source = stream.Endpoint(dma_layout(data_width))
        self.next_sink   = stream.Endpoint(dma_layout(data_width))

        self.writer_enable = Signal()
        self.reader_enable = Signal()

        # Reader FIFO Control/Status.
        if with_reader:
            assert bits_for(reader_depth) < 24
            self.reader_fifo_control = CSRStorage(fields=[
                CSRField("depth", offset=0, size=24, reset=reader_depth,
                         description="DMA Reader FIFO depth (in {}-bit words).".format(data_width)),
                CSRField("scratch",    offset=24, size=4, description="Software Scratchpad."),
                CSRField("level_mode", offset=31, values=[
                    ("``0b0``", "Report Instantaneous level."),
                    ("``0b1``", "Report `Minimal` level since last clear.")
                ])
            ])
            self.reader_fifo_status = CSRStatus(fields=[
                CSRField("level", offset=0, size=24,
                         description="DMA Reader FIFO level (in {}-bit words).".format(data_width))
            ])

        # Writer FIFO Control/Status.
        if with_writer:
            assert bits_for(writer_depth) < 24
            self.writer_fifo_control = CSRStorage(fields=[
                CSRField("depth", offset=0, size=24, reset=writer_depth,
                         description="DMA Writer FIFO depth (in {}-bit words).".format(data_width)),
                CSRField("scratch",    offset=24, size=4, description="Software Scratchpad."),
                CSRField("level_mode", offset=31, values=[
                    ("``0b0``", "Report Instantaneous level."),
                    ("``0b1``", "Report `Maximal` level since last clear.")
                ])
            ])
            self.writer_fifo_status = CSRStatus(fields=[
                CSRField("level", offset=0, size=24,
                         description="DMA Writer FIFO level (in {}-bit words).".format(data_width))
            ])

        # # #

        depth_shift = log2_int(data_width//8)

        # Reader FIFO.
        if with_reader:
            reader_fifo = ResetInserter()(SyncFIFO(dma_layout(data_width), reader_depth//(data_width//8), buffered=True))
            self.submodules += reader_fifo
            self.comb += [
                reader_fifo.reset.eq(~self.reader_enable),
                # Connect Reader Sink to Reader FIFO when Level < Configured Depth.
                self.sink.connect(reader_fifo.sink, omit={"valid", "ready"}),
                If((reader_fifo.level < self.reader_fifo_control.fields.depth[depth_shift:]) | (not dynamic_depth),
                   self.sink.connect(reader_fifo.sink, keep={"valid", "ready"})
                   ),
                # Connect Reader FIFO to Reader Source.
                reader_fifo.source.connect(self.next_source),
            ]

            # Store Min.
            reader_fifo_level_min = Signal.like(reader_fifo.level)
            self.sync += If(reader_fifo.level < reader_fifo_level_min, reader_fifo_level_min.eq(reader_fifo.level))
            # Clear on Status write or when in Instantaneous mode.
            reader_fifo_level_clr = (self.reader_fifo_status.re | (self.reader_fifo_control.fields.level_mode == 0))
            self.sync += If(reader_fifo_level_clr, reader_fifo_level_min.eq(2**len(reader_fifo_level_min)-1))
            # Return Reader FIFO level.
            self.comb += [
                # Instantaneous.
                If(self.reader_fifo_control.fields.level_mode == 0,
                   self.reader_fifo_status.fields.level[depth_shift:].eq(reader_fifo.level)
                   # Min.
                   ).Else(
                    self.reader_fifo_status.fields.level[depth_shift:].eq(reader_fifo_level_min)
                )
            ]

        # Writer FIFO.
        if with_writer:
            writer_fifo = ResetInserter()(SyncFIFO(dma_layout(data_width), writer_depth//(data_width//8), buffered=True))
            self.submodules += writer_fifo
            self.comb += [
                writer_fifo.reset.eq(~self.writer_enable),
                # Connect Writer Sink to Writer FIFO when Level < Configured Depth.
                self.next_sink.connect(writer_fifo.sink, omit={"valid", "ready"}),
                If((writer_fifo.level < self.writer_fifo_control.fields.depth[depth_shift:]) | (not dynamic_depth),
                   self.next_sink.connect(writer_fifo.sink, keep={"valid", "ready"})
                   ),
                # Connect Writer FIFO to Writer Source.
                writer_fifo.source.connect(self.source),
            ]

            # Store Max.
            writer_fifo_level_max = Signal.like(writer_fifo.level)
            self.sync += If(writer_fifo.level > writer_fifo_level_max, writer_fifo_level_max.eq(writer_fifo.level))
            # Clear on Status write or when in Instantaneous mode.
            writer_fifo_level_clr = (self.writer_fifo_status.re | (self.writer_fifo_control.fields.level_mode == 0))
            self.sync += If(writer_fifo_level_clr, writer_fifo_level_max.eq(0))
            # Return Writer FIFO level.
            self.comb += [
                # Instantaneous.
                If(self.writer_fifo_control.fields.level_mode == 0,
                   self.writer_fifo_status.fields.level[depth_shift:].eq(writer_fifo.level)
                   # Min.
                   ).Else(
                    self.writer_fifo_status.fields.level[depth_shift:].eq(writer_fifo_level_max)
                )
            ]
litepcie.frontend.dma.LitePCIeDMABuffering = mod_LitePCIeDMABuffering

from litepcie.frontend.dma import *
class mod_LitePCIeDMA(LiteXModule):
    """LitePCIe DMA

    Scatter-Gather bi-directional DMA:
    - Generates a data stream from Host's memory.
    - Stores a data stream to Host's memory.

    Optional buffering, loopback, synchronization and monitoring.
    """
    def __init__(self, phy, endpoint, with_table=True, table_depth=256, address_width=32, data_width=None,
                 with_writer       = True,
                 with_reader       = True,
                 # Loopback.
                 with_loopback     = False,
                 # Synchronizer.
                 with_synchronizer = False,
                 # Buffering.
                 with_buffering    = False, buffering_depth=256*8, writer_buffering_depth=None, reader_buffering_depth=None,
                 # Monitor.
                 with_monitor      = False,
                 # Status.
                 with_status       = False, status_width=32,
                 ):
        # Parameters -------------------------------------------------------------------------------
        self.data_width = data_width or phy.data_width

        # Endoints ---------------------------------------------------------------------------------
        self.sink   = stream.Endpoint(dma_layout(self.data_width))
        self.source = stream.Endpoint(dma_layout(self.data_width))

        # Writer/Reader ----------------------------------------------------------------------------
        if with_writer:
            self.writer = LitePCIeDMAWriter(
                endpoint             = endpoint,
                port                 = endpoint.crossbar.get_master_port(write_only=True),
                with_table           = with_table,
                table_depth          = table_depth,
                address_width        = address_width,
                data_width           = self.data_width,
            )
            self.comb += self.sink.connect(self.writer.sink)

        if with_reader:
            self.reader = LitePCIeDMAReader(
                endpoint             = endpoint,
                port                 = endpoint.crossbar.get_master_port(read_only=True),
                with_table           = with_table,
                table_depth          = table_depth,
                address_width        = address_width,
                data_width           = self.data_width,
            )
            self.comb += self.reader.source.connect(self.source)

        # Loopback ---------------------------------------------------------------------------------
        if with_loopback:
            if not (with_writer and with_reader):
                raise ValueError("Loopback capability requires DMAWriter and DMAReader to be enabled.")
            self.loopback = LitePCIeDMALoopback(self.data_width)
            self.add_plugin_module(self.loopback)

        # Synchronizer -----------------------------------------------------------------------------
        if with_synchronizer:
            if not (with_writer and with_reader):
                raise ValueError("Synchronizer capability requires DMAWriter and DMAReader to be enabled.")
            self.synchronizer = LitePCIeDMASynchronizer(self.data_width)
            self.add_plugin_module(self.synchronizer)

        # Buffering --------------------------------------------------------------------------------
        if with_buffering:
            writer_depth = writer_buffering_depth if writer_buffering_depth is not None else buffering_depth
            reader_depth = reader_buffering_depth if reader_buffering_depth is not None else buffering_depth
            self.buffering = LitePCIeDMABuffering(
                data_width   = self.data_width,
                with_reader  = with_reader,
                with_writer  = with_writer,
                reader_depth = reader_depth,
                writer_depth = writer_depth,
            )
            if with_writer:
                self.comb += self.buffering.writer_enable.eq(self.writer.enable)
            if with_reader:
                self.comb += self.buffering.reader_enable.eq(self.reader.enable)

            self.add_plugin_module(self.buffering)

        # Monitor ----------------------------------------------------------------------------------
        if with_monitor:
            if with_writer:
                self.writer_monitor = stream.Monitor(self.sink,   count_width=16, with_overflows=True)
            if with_reader:
                self.reader_monitor = stream.Monitor(self.source, count_width=16, with_underflows=True)

        # Status -----------------------------------------------------------------------------------
        if with_status:
            if not (with_writer and with_reader):
                raise ValueError("Status capability requires DMAWriter and DMAReader to be enabled.")
            self.status = LitePCIeDMAStatus(
                endpoint      = endpoint,
                writer        = self.writer,
                reader        = self.reader,
                address_width = address_width,
                status_width  = status_width,

            )

    def add_plugin_module(self, m):
        self.comb += [
            self.source.connect(m.sink),
            m.source.connect(self.sink)
        ]
        self.sink, self.source = m.next_sink, m.next_source
litepcie.frontend.dma.LitePCIeDMA = mod_LitePCIeDMA
# --------------------- Monkey patching end

# Constants ----------------------------------------------------------------------------------------

STRM0_FPGA_RX_RWIDTH = 64    # Stream PC->FPGA, rd width
STRM0_FPGA_TX_WWIDTH = 64    # Stream FPGA->PC, wr width
LMS_DIQ_WIDTH        = 12
TX_IN_PCT_HDR_SIZE   = 16
TX_PCT_SIZE          = 4096  # TX packet size in bytes
TX_N_BUFF            = 2     # N 4KB buffers in TX interface (2 OR 4)

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()
        self.cd_usb = ClockDomain()
        self.cd_xo_fpga = ClockDomain()

        # # #
        if platform.default_clk_name == "clk26":
            self.clk26 = platform.request("clk26")

            self.comb += self.cd_xo_fpga.clk.eq(self.clk26)
        else: # Fairwaves XTRX
            self.clk60 = platform.request("clk60")

            self.comb += self.cd_xo_fpga.clk.eq(self.clk60)

        # Clk / Rst.
        clk125 = ClockSignal("pcie")
        rst125 = ResetSignal("pcie")

        # PLL.
        self.pll = pll = S7MMCM(speedgrade=-2)
        self.comb += pll.reset.eq(rst125)
        pll.register_clkin(clk125, 125e6)
        pll.create_clkout(self.cd_idelay, 200e6, margin=0)
        pll.create_clkout(self.cd_sys,    sys_clk_freq, margin=0)
        pll.create_clkout(self.cd_usb, 26e6, margin=0)

        # IDelayCtrl.
        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

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

        # CNTRL
        "CNTRL"       : 26,

        # Analyzer.
        "analyzer"    : 31,

    }

    def __init__(self, board="limesdr", sys_clk_freq=int(100e6), cpu_type="picorv32",
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
            "fairwaves_cs"  : fairwaves_xtrx_platform.Platform(variant="xc7a35t"),
            "fairwaves_pro" : fairwaves_xtrx_platform.Platform(variant="xc7a50t"),
            "limesdr"       : limesdr_xtrx_platform.Platform()
        }[board]

        if gold_img:
            platform.toolchain.additional_commands += platform.gold_img_commands
        else:
            platform.toolchain.additional_commands += platform.user_img_commands

        platform.name        = "limesdr_xtrx"
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
            with_uartbone            = with_uartbone,
            uart_name                = {True: "crossover", False:"serial"}[with_uartbone],
        )

        # Avoid stalling CPU at startup.
        self.uart.add_auto_tx_flush(sys_clk_freq=sys_clk_freq, timeout=1, interval=128)

        # Define platform name constant.
        self.add_constant(platform.name.upper())

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
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                period       = 1,
                sys_clk_freq = sys_clk_freq
            )

        # AXI MMAP ---------------------------------------------------------------------------------
        # AXI MMAP Bus (From CPU).
        self.mmap = axi.AXILiteInterface(address_width=32, data_width=32)
        self.ram  = axi.AXILiteSRAM(0x1000)
        self.comb += self.mmap.connect(self.ram.bus)
        # Connect MMAP interface to SoC.
        self.bus.add_slave(name="lime_top_mmap", slave=self.mmap, region=SoCRegion(origin=0x4000_000, size=0x1000))

        # ICAP -------------------------------------------------------------------------------------
        self.icap = ICAP()
        self.icap.add_reload()
        self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

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
            self.flash      = S7SPIFlash(platform.request("spiflash"), sys_clk_freq, 4e6)

        # Leds GPIO. -------------------------------------------------------------------------------
        gpio_top_led = platform.request_all("user_led2")
        self.gpio = GpioTop(platform, gpio_top_led)

        # XADC -------------------------------------------------------------------------------------
        self.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # PCIe -------------------------------------------------------------------------------------
        self.pcie_phy = S7PCIEPHY(platform, platform.request(f"pcie_x2"),
            data_width  = 64,
            bar0_size   = 0x40000,
            cd          = "sys",
        )
        self.pcie_phy.update_config({
            #"pcie_id_if"               : "false",  #check this
            "Bar0_64bit"               : "true",
            "Bar0_Prefetchable"        : "false",
            "Vendor_ID"                : "2058",
            "Device_ID"                : "001B",
            "Base_Class_Menu"          : "Wireless_controller",
            "Sub_Class_Interface_Menu" : "Other_type_of_wireless_controller",
            "Class_Code_Base"          : "0D",
            "Class_Code_Sub"           : "80",
            "Revision_ID"              : "0001",
            "Subsystem_Vendor_ID"      : "2058",
            "Subsystem_ID"             : "0001",
            "MSI_64b"                  : "true",
            "MSIx_Table_BIR"           : "BAR_1:0",
            "MSIx_PBA_BIR"             : "BAR_1:0"

            }
        )
        self.add_pcie(phy=self.pcie_phy, address_width=64, ndmas=1,
            with_dma_buffering    = True, dma_buffering_depth=8192,
            with_dma_loopback     = False,
            with_dma_synchronizer = False,
            with_msi              = True
        )
        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.pcie_phy.cd_pcie.clk)

        # I2C Bus0 ---------------------------------------------------------------------------------
        # - Temperature Sensor (TMP108  @ 0x4a) Lime: (TMP1075 @ 0x4b).
        # - PMIC-LMS           (LP8758  @ 0x60).
        # - VCTCXO DAC         Rev4: (MCP4725 @ 0x62) Rev5: (DAC60501 @ 0x4b) Lime: (AD5693 @ 0x4c).
        self.i2c0 = I2CMaster(pads=platform.request("i2c", 0))

        # I2C Bus1 ---------------------------------------------------------------------------------
        # PMIC-FPGA (LP8758 @ 0x60).
        self.i2c1 = I2CMaster(pads=platform.request("i2c", 1))

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
        self.aux = AUX(platform.request("aux"))

        # Timing Constraints/False Paths -----------------------------------------------------------
        platform.toolchain.pre_placement_commands.append(
            "set_clock_groups "
            "-group [get_clocks -include_generated_clocks main_crg_clkout1] "
            "-group [get_clocks -include_generated_clocks main_s7pciephy_clkout0] "
            "-group [get_clocks -include_generated_clocks main_s7pciephy_clkout1] "
            "-group [get_clocks -include_generated_clocks main_s7pciephy_clkout2] "
            "-group [get_clocks -include_generated_clocks main_s7pciephy_clkout3] "
            "-group [get_clocks -include_generated_clocks txoutclk_x0y0] "
            "-group [get_clocks -include_generated_clocks rxclk_global_clk] "
            "-group [get_clocks -include_generated_clocks txclk_global_clk] "
            "-group [get_clocks -include_generated_clocks clk26] "
            "-group [get_clocks -include_generated_clocks jtag_clk] "
            "-group [get_clocks -include_generated_clocks icap_clk] "
            "-group [get_clocks -include_generated_clocks dna_clk] "
            "-asynchronous"
        )

        # LimeTOP ----------------------------------------------------------------------------------
        self.lime_top = LimeTop(self, platform,
            # Configuration.
            LMS_DIQ_WIDTH        = 12,
            sink_width           = 64,
            sink_clk_domain      = "sys",
            source_width         = 64,
            source_clk_domain    = "sys",
            TX_N_BUFF            = TX_N_BUFF,
            TX_PCT_SIZE          = 4096,
            TX_IN_PCT_HDR_SIZE   = 16,
            tx_buffer_size       = 0,

            with_rx_tx_top       = True,
            with_fft             = with_fft,

            # FPGACFG.
            board_id             = 27,
            # GOLD image can be recocgnized by 0xDEAD in major and compile revisions
            major_rev            =  3 if not gold_img else 0xDEAD,
            compile_rev          =  0 if not gold_img else 0xDEAD,
            revision_pads        = None,
            # TODO: maybe it's possible to implement check automatically?
            soc_has_timesource   = True,
        )

        self.comb += self.lime_top.source.connect(self.pcie_dma0.sink, keep={"valid", "ready", "last", "data"}),

        # PCIE DMA -> TX Path -> LMS7002 Pipeline.
        self.comb += [
            self.pcie_dma0.source.connect(self.lime_top.sink, omit=["ready"]),
            self.pcie_dma0.source.ready.eq((self.lime_top.sink.ready & self.lime_top.fpgacfg.rx_en) | ~self.pcie_dma0.reader.enable),
        ]

        self.comb += self.lime_top.rxtx_top.tx_path.ext_reset_n.eq(self.pcie_dma0.reader.enable)

        # LMS SPI -----------------------------------------------------------------------------------

        self.add_spi_master(name="spimaster", pads=platform.request("lms7002m_spi"), data_width=32, spi_clk_freq=1e6)

        # Interrupt --------------------------------------------------------------------------------

        self.irq.add("lime_top")

        # GPS serial connected to LimeUART0
        from litex.soc.cores.uart import UARTPHY
        from litex.soc.cores.uart import UART

        self.gps_pads       = platform.request("gps")
        gnss_uart_pads = self.platform.request("gps_serial", loose=True)
        gnss_uart_phy  = UARTPHY(gnss_uart_pads, clk_freq=self.sys_clk_freq, baudrate=9600)
        pcie_uart0     = UART(gnss_uart_phy, tx_fifo_depth=64, rx_fifo_depth=16, rx_fifo_rx_we=True)
        self.add_module(name=f"PCIE_UART0_phy", module=gnss_uart_phy)
        self.add_module(name="PCIE_UART0", module=pcie_uart0)

        # Get UTC time from GNSS, assign UTC data to timestamp logic in rx_path
        from gateware.LimeDFB_LiteX.general.ZDAParser import ZDAParser
        self.zda_parser = ZDAParser(self)
        self.comb += [
            self.zda_parser.sink.data.eq (gnss_uart_phy.source.data ),
            self.zda_parser.sink.valid.eq(gnss_uart_phy.source.valid),
            self.lime_top.time_seconds.eq(self.zda_parser.time_seconds),
            self.lime_top.time_minutes.eq(self.zda_parser.time_minutes),
            self.lime_top.time_hours.eq  (self.zda_parser.time_hours  ),
            self.lime_top.time_day.eq    (self.zda_parser.time_day    ),
            self.lime_top.time_month.eq  (self.zda_parser.time_month  ),
            self.lime_top.time_year.eq   (self.zda_parser.time_year   ),
            self.lime_top.rxtx_top.rx_path.pps.eq(self.zda_parser.pps ),
        ]
        ####
        # Current time registers
        self.time_min_sec = CSRStatus(size= 16, description="Time in minutes and seconds, current", fields=[
            CSRField("sec", size=6, offset=0, description="Current time, seconds"),
            CSRField("min", size=6, offset=6, description="Current  time, minutes")
        ])
        self.time_mon_day_hrs = CSRStatus(size= 16, description="Time in months, days and hours, current", fields=[
            CSRField("hrs", size=5, offset=0, description="Current time, hours"),
            CSRField("day", size=5, offset=5, description="Current start time, days"),
            CSRField("mon", size=4, offset=10, description="Current start time, months"),
        ])
        self.time_yrs = CSRStatus(size= 16, description="Time in years, current", fields=[
            CSRField("yrs", size=12, offset=0, description="Current time, years")
        ])
        self.comb +=[
                self.time_min_sec.fields.sec.eq    (self.zda_parser.time_seconds),
                self.time_min_sec.fields.min.eq    (self.zda_parser.time_minutes),
                self.time_mon_day_hrs.fields.hrs.eq(self.zda_parser.time_hours  ),
                self.time_mon_day_hrs.fields.day.eq(self.zda_parser.time_day    ),
                self.time_mon_day_hrs.fields.mon.eq(self.zda_parser.time_month  ),
                self.time_yrs.fields.yrs.eq        (self.zda_parser.time_year   ),
        ]
        # CLK Tests --------------------------------------------------------------------------------

        from gateware.LimeDFB.self_test.clk_no_ref_test import clk_no_ref_test
        from gateware.LimeDFB.self_test.singl_clk_with_ref_test import singl_clk_with_ref_test
        self.sys_clock_test = clk_no_ref_test(platform=platform,test_clock_domain="sys")
        self.comb += self.sys_clock_test.RESET_N.eq(self.crg.pll.locked)

        self.lms_clock_test = singl_clk_with_ref_test(platform=platform,test_clock_domain="xo_fpga"
            , ref_clock_domain="sys")
        self.comb += self.lms_clock_test.RESET_N.eq(self.crg.pll.locked)

        # VCTCXO tamer
        self.pps_internal = Signal()
        self.comb += [
            self.zda_parser.pps.eq(self.pps_internal)
        ]

        synchro_pads = platform.request("synchro")
        self.comb += [
            If(self.periphcfg.PERIPH_INPUT_SEL_0.storage[0:1] == 0b01,
                self.pps_internal.eq(synchro_pads.pps_in)
            ).Else(
                self.pps_internal.eq(self.gps_pads.pps)
            )
        ]

        # Define a layout for vctcxo_tamer_pads
        vctcxo_tamer_layout        = [("tune_ref", 1)]  # 1-bit wide signal for tune_ref
        vctcxo_tamer_pads          = Record(vctcxo_tamer_layout)
        vctcxo_tamer_pads.tune_ref = self.pps_internal

        from gateware.LimeDFB.vctcxo_tamer.src.vctcxo_tamer_top import vctcxo_tamer_top
        self.vctcxo_tamer = vctcxo_tamer_top(platform=platform,
            vctcxo_tamer_pads = vctcxo_tamer_pads,
            clk100_domain     = "sys",
            vctcxo_clk_domain = "xo_fpga"
        )
        self.comb += self.vctcxo_tamer.RESET_N.eq(self.crg.pll.locked)

        vctcxo_tamer_serial_layout = [("rx", 1),
                                      ("tx", 1)]  # 1-bit wide signal for tune_ref
        vctcxo_tamer_serial_pads = Record(vctcxo_tamer_serial_layout)
        self.comb += vctcxo_tamer_serial_pads.rx.eq(self.vctcxo_tamer.UART_TX)
        self.comb += self.vctcxo_tamer.UART_RX.eq(vctcxo_tamer_serial_pads.tx)

        pcie_uart1_phy = UARTPHY(vctcxo_tamer_serial_pads, clk_freq=self.sys_clk_freq, baudrate=9600)
        pcie_uart1     = UART(pcie_uart1_phy, tx_fifo_depth=16, rx_fifo_depth=16, rx_fifo_rx_we=True)
        self.add_module(name=f"PCIE_UART1_phy", module=pcie_uart1_phy)
        self.add_module(name="PCIE_UART1", module=pcie_uart1)


        ### Misc assignments
        # Stream delay signals
        self.comb += [
            # self.lime_top.fpgacfg.tx_en_delay_signal[0].eq(self.zda_parser.pps_rising),
            # self.lime_top.fpgacfg.tx_en_delay_signal[1].eq(self.zda_parser.pps_rising & self.zda_parser.time_valid),
            # self.lime_top.fpgacfg.rx_en_delay_signal[0].eq(self.zda_parser.pps_rising),
            # self.lime_top.fpgacfg.rx_en_delay_signal[1].eq(self.zda_parser.pps_rising & self.zda_parser.time_valid),
            # NOTE: using rx_path synced pps, because separate tx path enable is not used, should be fine
            self.lime_top.fpgacfg.tx_en_delay_signal[0].eq(self.lime_top.rxtx_top.rx_path.pps_rising),
            self.lime_top.fpgacfg.tx_en_delay_signal[1].eq(self.lime_top.rxtx_top.rx_path.pps_rising & self.zda_parser.time_valid),
            self.lime_top.fpgacfg.rx_en_delay_signal[0].eq(self.lime_top.rxtx_top.rx_path.pps_rising),
            self.lime_top.fpgacfg.rx_en_delay_signal[1].eq(self.lime_top.rxtx_top.rx_path.pps_rising & self.zda_parser.time_valid),
        ]




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

    # LiteScope Analyzer Probes --------------------------------------------------------------------

    def add_smpl_cmp_probe(self):
        analyzer_signals = [
            self.lms7002_top.smpl_cmp_en,
            self.lms7002_top.smpl_cmp_done,
            self.lms7002_top.smpl_cmp_error,
            self.lms7002_top.smpl_cmp_cnt,
            self.lms7002_top.lms7002_ddin.rx_diq2_h,
            self.lms7002_top.lms7002_ddin.rx_diq2_l,
            self.lms7002_top.DEBUG_IQ_ERR,
            self.lms7002_top.DEBUG_AI_ERR,
            self.lms7002_top.DEBUG_AQ_ERR,
            self.lms7002_top.DEBUG_BI_ERR,
            self.lms7002_top.DEBUG_BQ_ERR,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 1024,
            clock_domain = "lms_rx",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

    def add_rx_stream_ctrl_probe(self):
        analyzer_signals = [
            self.lms7002_top.lms7002_ddin.rx_diq2_h,
            self.lms7002_top.lms7002_ddin.rx_diq2_l,
            self.lms7002_top.rx_diq2_h_mux,
            self.lms7002_top.rx_diq2_l_mux,
            self.lms7002_top.rx_cdc.sink.valid,
            self.lms7002_top.rx_cdc.sink.ready,
            self.lms7002_top.source.valid,
            self.lms7002_top.source.ready,
            self.lms7002_top.rx_reset_n,
            self.fpgacfg.rx_en,
            self.rxtx_top.rx_path.iq_to_bit_pack_tvalid,
            self.rxtx_top.rx_path.bit_pack_to_nto1_tvalid,
            self.rxtx_top.rx_path.bit_pack_to_nto1_tlast,
            self.rxtx_top.rx_path.fifo_conv.sink.ready,
            self.rxtx_top.rx_path.fifo_conv.sink.valid,
            self.rxtx_top.rx_path.source.valid,
            self.rxtx_top.rx_path.source.ready,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 1024,
            clock_domain = "lms_rx",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

    def add_rx_pct_hdr_1_probe(self):
        analyzer_signals = [
            self.rxtx_top.rx_path.sink.valid,
            self.rxtx_top.rx_path.int_clk_smpl_nr_clr,
            self.rxtx_top.rx_path.pct_hdr_1,
            self.rxtx_top.rx_path.iqsmpls_fifo.source.last,
            self.rxtx_top.rx_path.iqsmpls_fifo_source_valid,
            self.rxtx_top.rx_path.iqsmpls_fifo_source_ready,
            self.rxtx_top.rx_path.int_clk_rst_n,
            self.rxtx_top.rx_path.int_clk_mimo_en,
            self.rxtx_top.rx_path.int_clk_ch_en,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 1024,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

    def add_tx_stream_ctrl_probe(self):
        analyzer_signals = [
            self.rxtx_top.tx_path.sink.valid,
            self.rxtx_top.tx_path.sink.ready,
            self.rxtx_top.tx_path.conv_64_to_128.source.valid,
            self.rxtx_top.tx_path.conv_64_to_128.source.ready,
            self.rxtx_top.tx_path.smpl_fifo.sink.ready,
            self.rxtx_top.tx_path.smpl_fifo.sink.valid,
            self.rxtx_top.tx_path.source.valid,
            self.rxtx_top.tx_path.source.ready,
            self.rxtx_top.tx_path.p2d_rd_tready,
            self.rxtx_top.tx_path.p2d_rd_tlast,
            self.rxtx_top.tx_path.p2d_rd_tvalid,
            self.fpgacfg.tx_en,
            self.rxtx_top.tx_path.ext_reset_n,
            self.rxtx_top.tx_path.data_pad_tready,
            self.rxtx_top.tx_path.data_pad_tvalid,
            self.rxtx_top.tx_path.data_pad_tlast,
            self.rxtx_top.tx_path.curr_buf_index,
            self.rxtx_top.tx_path.conn_buf,
            self.rxtx_top.tx_path.p2d_wr_tvalid,
            self.rxtx_top.tx_path.p2d_wr_tready,
            self.rxtx_top.tx_path.p2d_wr_tlast,
        ]

        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 1024,
            clock_domain = "lms_tx",
            register     = True,
            csr_csv      = "analyzer.csv"
        )


# Build --------------------------------------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(description="LiteX SoC on Fairwaves/LimeSDR XTRX.")

    parser.add_argument("--board",    default="limesdr",      help="Select XTRX board.", choices=["fairwaves_cs", "fairwaves_pro", "limesdr"])
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

    # Litescope Analyzer Probes.
    probeopts = parser.add_mutually_exclusive_group()
    probeopts.add_argument("--with-smpl-cmp-probe",       action="store_true", help="Enable RX Sample Compare Probe.")
    probeopts.add_argument("--with-rx-stream-ctrl-probe", action="store_true", help="Enable RX Stream Control Probe.")
    probeopts.add_argument("--with-rx-pct-hdr-1-probe",   action="store_true", help="Enable RX PCT HDR 1 Probe.")
    probeopts.add_argument("--with-tx-stream-ctrl-probe", action="store_true", help="Enable TX Stream Control Probe.")

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

        # LiteScope Analyzer Probes.
        if args.with_smpl_cmp_probe or args.with_rx_stream_ctrl_probe or args.with_rx_pct_hdr_1_probe:
            assert args.with_uartbone or not args.with_bscan
            if args.with_smpl_cmp_probe:
                soc.add_smpl_cmp_probe()
            if args.with_rx_stream_ctrl_probe:
                soc.add_rx_stream_ctrl_probe()
            if args.with_rx_pct_hdr_1_probe:
                soc.add_rx_pct_hdr_1_probe()
        if args.with_tx_stream_ctrl_probe:
            assert args.with_uartbone or not args.with_bscan
            if args.with_tx_stream_ctrl_probe:
                soc.add_tx_stream_ctrl_probe()

        builder = Builder(soc, csr_csv="csr.csv", bios_console="lite")
        builder.build(run=build)
        # Firmware build.
        if prepare:
            linker = {
                True  : "linker_main_ram.ld",
                False : "linker_rom.ld",
            }[args.with_bios]
            os.system(f"cd firmware && make BUILD_DIR={builder.output_dir} TARGET={soc.platform.name.upper()} LINKER={linker} clean all")
            bistream_output_dir = "bitstream/LimeSDR_XTRX"
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
            prog.flash(0, os.path.join(bistream_output_dir, "LimeSDR_XTRX" + "_golden" + ".bin"), enable_quad=True)
        else: #user img
            # TODO: move user img address to a global variable somewhere instead of hardcoding
            prog = soc.platform.create_programmer(cable=args.cable)
            prog.flash(0X00220000, os.path.join(bistream_output_dir, "LimeSDR_XTRX" + "_user" + ".bin"), enable_quad=True)

    # Flash Firmware.
    if args.flash_boot and args.flash:
        from litex.soc.software.crcfbigen import insert_crc
        insert_crc("firmware/firmware.bin", fbi_mode=True, o_filename="firmware/firmware.fbi", little_endian=True)
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(args.firmware_flash_offset, "firmware/firmware.fbi")


if __name__ == "__main__":
    main()
