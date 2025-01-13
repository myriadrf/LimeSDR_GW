#!/usr/bin/env python3

#
# This file is part of LiteX-XTRX.
#
# Copyright (c) 2021-2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.gen import *

from boards.platforms import fairwaves_xtrx_platform
from boards.platforms import limesdr_xtrx_platform

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
from litex.soc.cores.spi_flash import S7SPIFlash
from litex.soc.cores.bitbang   import I2CMaster
from litex.soc.cores.spi       import SPIMaster

from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP

from litepcie.phy.s7pciephy import S7PCIEPHY

from litescope import LiteScopeAnalyzer

from gateware.aux import AUX
from gateware.xtrx_rfsw import xtrx_rfsw

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()
        self.cd_usb = ClockDomain()
        self.cd_xo_fpga = ClockDomain()

        # # #
        self.clk26 = platform.request("clk26")

        self.comb += self.cd_xo_fpga.clk.eq(self.clk26)

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
            self.major_rev      = CSRStatus(16, reset=2)
            self.compile_rev    = CSRStatus(16, reset=25)
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

        # CNTRL
        "CNTRL"       : 26,

        # Analyzer.
        "analyzer"    : 31,

    }

    def __init__(self, board="limesdr", sys_clk_freq=int(100e6),
        with_cpu              = True, cpu_firmware=None,
        with_jtagbone         = True,
        with_bscan            = False,
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

        # Enable Compressed Instructions.
        VexRiscvSMP.with_rvc = True
        # Enable JTAG.
        if with_bscan:
            VexRiscvSMP.privileged_debug     = True
            VexRiscvSMP.hardware_breakpoints = 4

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = f"LiteX SoC on {board.capitalize()} XTRX ",
            ident_version            = True,
            cpu_type                 = "vexriscv_smp" if with_cpu else None,
            cpu_variant              = "standard",
            integrated_rom_size      = 0x8000 if with_cpu else 0,
            integrated_sram_ram_size = 0x1000 if with_cpu else 0,
            integrated_main_ram_size = 0x4800 if with_cpu else 0, #increase when cpu debug is enabled
            integrated_main_ram_init = [] if cpu_firmware is None or flash_boot else get_mem_data(cpu_firmware, endianness="little"),
            uart_name                = "gpio_serial",#"crossover",
        )
        # Avoid stalling CPU at startup.
        self.uart.add_auto_tx_flush(sys_clk_freq=sys_clk_freq, timeout=1, interval=128)

        self.fpgacfg = fpgacfg_csr(gold=gold_img)
        self.periphcfg = periphcfg_csr()
        self.CNTRL = CNTRL_CSR(1,2)
        self.irq.add("CNTRL")

        # Clocking ---------------------------------------------------------------------------------
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
            # TODO: change this to add_spi_flash in the future
            self.flash      = S7SPIFlash(platform.request("spiflash"), sys_clk_freq, 4e6)

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
            "pcie_id_if"               : "false",  #check this
            "Bar0_64bit"               : "true",
            "Bar0_Prefetchable"        : "false",
            "Vendor_ID"                : "2058",
            "Device_ID"                : "001B",
            "Revision_ID"              : "0001",
            "Subsystem_Vendor_ID"      : "2058",
            "Subsystem_ID"             : "0001",
            "Class_Code_Base"          : "0D",
            "Class_Code_Sub"           : "80",
            "Base_Class_Menu"          : "Wireless_controller",
            "Sub_Class_Interface_Menu" : "Other_type_of_wireless_controller",
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
        for i in range(4):
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks        dna_clk] -asynchronous")
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks       jtag_clk] -asynchronous")
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks       icap_clk] -asynchronous")

        # Lime Top Level -------------------------------------------------------------------
        from gateware.LimeTop import LimeTop
        # fft example
        # from gateware.examples.fft.LimeTop_fft import LimeTop

        # Create LimeTop instance.
        self.lime_top = LimeTop(platform, sys_clk_freq)
        self.irq.add("lime_top")

        # Connect LimeTop's MMAP interface to SoC.
        self.bus.add_slave(name="lime_top_mmap", slave=self.lime_top.mmap, region=SoCRegion(origin=0x4000_000, size=0x1000))

        # Connect LimeTop's Streaming interfaces to PCIe.
        self.comb += [
            #self.pcie_dma0.source.connect(self.lime_top.dma_tx, keep={"valid", "ready", "last", "data"}),
            self.lime_top.dma_rx.connect(self.pcie_dma0.sink,   keep={"valid", "ready", "last", "data"}),
        ]



        self.comb += [
            self.lime_top.dma_tx.valid.eq(self.pcie_dma0.source.valid),
            self.lime_top.dma_tx.last.eq(self.pcie_dma0.source.last),
            self.lime_top.dma_tx.data.eq(self.pcie_dma0.source.data),
            self.pcie_dma0.source.ready.eq((self.lime_top.dma_tx.ready & self.lime_top.lms7002.tx_en.storage) | ~self.pcie_dma0.reader.enable),
        ]

        self.comb += self.lime_top.tx_path.RESET_N.eq(self.pcie_dma0.reader.enable)

        # LMS SPI
        self.lms_spi = SPIMaster(
            pads=platform.request("lms7002m_spi"),
            data_width=32,
            sys_clk_freq=sys_clk_freq,
            spi_clk_freq=1e6
        )

        vctcxo_pads = platform.request("vctcxo")
        self.comb += vctcxo_pads.sel.eq(self.fpgacfg.EXT_CLK.storage)
        self.comb += vctcxo_pads.en.eq(self.fpgacfg.TCXO_EN.storage)

        rfsw_pads = platform.request("rf_switches")

        self.rfsw_control = xtrx_rfsw(platform, rfsw_pads)
        #self.comb += rfsw_pads.tx.eq(1)

        # GPS serial connected to LimeUART0
        from litex.soc.cores.uart import UARTPHY
        from litex.soc.cores.uart import UART

        gps_pads = platform.request("gps")
        gnss_uart_pads = self.platform.request("gps_serial", loose=True)
        gnss_uart_phy = UARTPHY(gnss_uart_pads, clk_freq=self.sys_clk_freq, baudrate=9600)
        pcie_uart0 = UART(gnss_uart_phy, tx_fifo_depth=16, rx_fifo_depth=16, rx_fifo_rx_we=True)
        self.add_module(name=f"PCIE_UART0_phy", module=gnss_uart_phy)
        self.add_module(name="PCIE_UART0", module=pcie_uart0)


        # VCTCXO tamer
        self.pps_internal = Signal()

        synchro_pads = platform.request("synchro")
        self.comb += [
            If(self.periphcfg.PERIPH_INPUT_SEL_0.storage[0:1] == 0b01,
                self.pps_internal.eq(synchro_pads.pps_in)
            ).Else(
                self.pps_internal.eq(gps_pads.pps)
            )
        ]

        # Define a layout for vctcxo_tamer_pads
        vctcxo_tamer_layout = [("tune_ref", 1)]  # 1-bit wide signal for tune_ref
        vctcxo_tamer_pads = Record(vctcxo_tamer_layout)
        vctcxo_tamer_pads.tune_ref =self.pps_internal

        from gateware.LimeDFB.vctcxo_tamer.src.vctcxo_tamer_top import vctcxo_tamer_top
        self.vctcxo_tamer = vctcxo_tamer_top(platform=platform, vctcxo_tamer_pads=vctcxo_tamer_pads, clk100_domain="sys", vctcxo_clk_domain="xo_fpga")
        self.comb += self.vctcxo_tamer.RESET_N.eq(self.crg.pll.locked)


        vctcxo_tamer_serial_layout = [("rx", 1),
                                      ("tx", 1)]  # 1-bit wide signal for tune_ref
        vctcxo_tamer_serial_pads = Record(vctcxo_tamer_serial_layout)
        self.comb += vctcxo_tamer_serial_pads.rx.eq(self.vctcxo_tamer.UART_TX)
        self.comb += self.vctcxo_tamer.UART_RX.eq(vctcxo_tamer_serial_pads.tx)

        pcie_uart1_phy = UARTPHY(vctcxo_tamer_serial_pads, clk_freq=self.sys_clk_freq, baudrate=9600)
        pcie_uart1 = UART(pcie_uart1_phy, tx_fifo_depth=16, rx_fifo_depth=16, rx_fifo_rx_we=True)
        self.add_module(name=f"PCIE_UART1_phy", module=pcie_uart1_phy)
        self.add_module(name="PCIE_UART1", module=pcie_uart1)

    # CLK Tests ------------------------------------------------------------------------------------
        from gateware.LimeDFB.self_test.clk_no_ref_test import clk_no_ref_test
        from gateware.LimeDFB.self_test.singl_clk_with_ref_test import singl_clk_with_ref_test
        self.sys_clock_test = clk_no_ref_test(platform=platform,test_clock_domain="sys")
        self.comb += self.sys_clock_test.RESET_N.eq(self.crg.pll.locked)

        self.lms_clock_test = singl_clk_with_ref_test(platform=platform,test_clock_domain="xo_fpga"
                                                      , ref_clock_domain="sys")
        self.comb += self.lms_clock_test.RESET_N.eq(self.crg.pll.locked)



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

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Fairwaves/LimeSDR XTRX.")
    parser.add_argument("--board",   default="limesdr", help="Select XTRX board.", choices=["fairwaves_cs", "fairwaves_pro", "limesdr"])
    parser.add_argument("--with-bscan",            action="store_true",     help="Enable CPU debug over JTAG."),
    parser.add_argument("--build",                 action="store_true",     help="Build bitstream.")
    parser.add_argument("--load",                  action="store_true",     help="Load bitstream.")
    parser.add_argument("--flash",                 action="store_true",     help="Flash bitstream.")
    parser.add_argument("--cable",                 default="digilent_hs2",  help="JTAG cable.")
    parser.add_argument("--driver",                action="store_true",     help="Generate PCIe driver from LitePCIe (override local version).")
    parser.add_argument("--flash-boot",            action="store_true",     help="Write Firmware in Flash instead of RAM.")
    parser.add_argument("--firmware-flash-offset", default=0x220000,        help="Firmware SPI Flash offset.")
    parser.add_argument("--gold",                  action="store_true",     help="Build/Flash golden image instead of user")
    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        soc = BaseSoC(
            board                 = args.board,
            cpu_firmware          = None if prepare else "firmware/firmware.bin",
            with_jtagbone         = not args.with_bscan,
            with_bscan            = args.with_bscan,
            flash_boot            = args.flash_boot,
            gold_img              = args.gold,
            firmware_flash_offset = args.firmware_flash_offset,
        )
        builder = Builder(soc, csr_csv="csr.csv")
        builder.build(run=build)
        if prepare:
            os.system(f"cd firmware && make BUILD_DIR={builder.output_dir} clean all")
            bistream_output_dir = "bitstream/{}".format(soc.get_build_name())
            if not os.path.exists(bistream_output_dir):
                os.makedirs(bistream_output_dir)

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    # Flash Bitstream.
    if args.flash:
        if args.gold:
            prog = soc.platform.create_programmer(cable=args.cable)
            prog.flash(0, os.path.join(bistream_output_dir, soc.build_name + "_golden" + ".bin"))
        else: #user img
            # TODO: move user img address to a global variable somewhere instead of hardcoding
            prog = soc.platform.create_programmer(cable=args.cable)
            prog.flash(0X00220000, os.path.join(bistream_output_dir, soc.build_name + "_user" + ".bin"))

    # Flash Firmware.
    if args.flash_boot and args.flash:
        from litex.soc.software.crcfbigen import insert_crc
        insert_crc("firmware/firmware.bin", fbi_mode=True, o_filename="firmware/firmware.fbi", little_endian=True)
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(args.firmware_flash_offset, "firmware/firmware.fbi")


if __name__ == "__main__":
    main()
