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

#from boards.platforms import fairwaves_xtrx_platform
import limesdr_xtrx_platform

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

from gateware.aux                             import AUX
from gateware.fpgacfg                         import FPGACfg
from gateware.GpioTop                         import GpioTop
from gateware.lms7002.lms7002_top             import LMS7002Top
from gateware.rxtx_top                        import RXTXTop

from gateware.LimeDFB.tdd_control.tdd_control import TDDControl

#from software import generate_litepcie_software

# Constants ----------------------------------------------------------------------------------------

STRM0_FPGA_RX_RWIDTH = 128   # Stream PC->FPGA, rd width
STRM0_FPGA_TX_WWIDTH = 64    # Stream FPGA->PC, wr width
LMS_DIQ_WIDTH        = 12
TX_IN_PCT_HDR_SIZE   = 16
TX_PCT_SIZE          = 4096  # TX packet size in bytes
TX_N_BUFF            = 4     # N 4KB buffers in TX interface (2 OR 4)

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()

        # # #

        # Clk / Rst.
        clk125 = ClockSignal("pcie")
        rst125 = ResetSignal("pcie")

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(rst125)
        pll.register_clkin(clk125, 125e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_idelay, 200e6)

        # IDelayCtrl.
        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# LMS Control CSR----------------------------------------------------------------------------------------
class CNTRL_CSR(LiteXModule):
    def __init__(self, ndmas):
        self.cntrl          = CSRStorage(512, 0)
        self.enable         = CSRStorage()
        self.test           = CSRStorage(32)
        self.ndma           = CSRStatus(4, reset=ndmas)
        self.enable_both    = CSRStorage()

        # Create event manager for interrupt
        self.ev = EventManager()
        self.ev.cntrl_isr = EventSourceProcess()
        self.ev.finalize()

        # Trigger interrupt when cntrl register is written
        self.comb += self.ev.cntrl_isr.trigger.eq(self.cntrl.re)

# fpgacfg
class fpgacfg_csr(LiteXModule):
    def __init__(self):
        self.board_id       = CSRStatus(16, reset=27)
        self.major_rev      = CSRStatus(16, reset=1)
        self.compile_rev    = CSRStatus(16, reset=18)
        self.reserved_03    = CSRStorage(16, reset=0)
        self.reserved_04    = CSRStorage(16, reset=0)
        self.reserved_05    = CSRStorage(16, reset=0)
        self.reserved_06    = CSRStorage(16, reset=0)
        self.channel_cntrl  = CSRStorage(fields=[
            CSRField("ch_en", size=2, offset=0, values=[
                ("``2b01", "Channel A"),
                ("``2b10", "Channel B"),
                ("``2b11", "Channels A and B")
            ], reset=0)
        ])


# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    SoCCore.csr_map = {
        # SoC.
        "uart"        : 0,
        "icap"        : 1,
        "flash"       : 10, #10
        "xadc"        : 11, #11
        "dna"         : 12, #12

        # PCIe.
        "pcie_phy"    : 2, #10
        "pcie_msi"    : 3, #11
        "pcie_dma0"   : 5, #12

        # XTRX.
        "i2c0"        : 20,
        "i2c1"        : 21,

        # Analyzer.
        "analyzer"    : 30,

        # CNTRL
        "CNTRL"       : 26,
    }

    def __init__(self, board="limesdr", sys_clk_freq=int(125e6), cpu_type="picorv32",
        with_bios             = False,
        with_uartbone         = False,
        with_cpu              = True, cpu_firmware=None,
        with_jtagbone         = True,
        with_bscan            = False,
        flash_boot            = False,
        firmware_flash_offset = 0x220000,
    ):

        # Platform ---------------------------------------------------------------------------------
        platform = {
            #"fairwaves_cs"  : fairwaves_xtrx_platform.Platform(variant="xc7a35t"),
            #"fairwaves_pro" : fairwaves_xtrx_platform.Platform(variant="xc7a50t"),
            "limesdr"       : limesdr_xtrx_platform.Platform()
        }[board]

        platform.vhd2v_force = False

        # Enable Compressed Instructions.
        VexRiscvSMP.with_rvc = True
        # Enable JTAG.
        if with_bscan:
            VexRiscvSMP.privileged_debug     = True
            VexRiscvSMP.hardware_breakpoints = 4

        # SoCCore ----------------------------------------------------------------------------------
        assert cpu_type in ["vexriscv", "picorv32", "fazyrv", "firev"]

        cpu_variant = {
            "vexriscv" : "minimal",
            "picorv32" : "minimal",
            "fazyrv"   : "standard",
            "firev"    : "standard",
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

        #self.fpgacfg = fpgacfg_csr()
        self.CNTRL = CNTRL_CSR(1)
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
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
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
            "Base_Class_Menu"          : "Wireless_controller",
            "Sub_Class_Interface_Menu" : "RF_controller",
            "Class_Code_Base"          : "0D",
            "Class_Code_Sub"           : "10",
            "Revision_ID"              : "0001",
            }
        )
        self.add_pcie(phy=self.pcie_phy, address_width=32, ndmas=1,
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

        # FPGA Cfg ---------------------------------------------------------------------------------
        self.fpgacfg  = FPGACfg(platform)

        # PLL Cfg ----------------------------------------------------------------------------------
        #self.pllcfg = PLLCfg()

        # LMS7002 Top ------------------------------------------------------------------------------
        self.lms7002_top = LMS7002Top(
            platform        = platform,
            pads            = platform.request("LMS"),
            hw_ver          = Constant(0, 4),
            add_csr         = True,
            fpgacfg_manager = self.fpgacfg,
            diq_width       = LMS_DIQ_WIDTH,
        )

        # Tst Top / Clock Test ---------------------------------------------------------------------
        #self.tst_top = TstTop(platform, self.crg.ft_clk, platform.request("LMK_CLK"))

        # General Periph ---------------------------------------------------------------------------

        #gpio_pads     = platform.request("FPGA_GPIO")
        ##egpio_pads    = platform.request("FPGA_EGPIO")

        #self.general_periph = GeneralPeriphTop(platform,
        #    revision_pads = Constant(0, 4),
        #    gpio_pads     = gpio_pads,
        #    gpio_len      = len(gpio_pads),
        #    egpio_pads    = None,
        #    egpio_len     = 2,
        #)

        #self.comb += [
        #    self.general_periph.led1_mico32_busy.eq(self.busy_delay.busy_out),
        #    self.general_periph.ep03_active.eq(self.ft601.rd_active),
        #    self.general_periph.ep83_active.eq(self.ft601.wr_active),
        #]

        self.rxtx_top = RXTXTop(platform, self.fpgacfg,
            # TX parameters
            TX_IQ_WIDTH            = LMS_DIQ_WIDTH,
            TX_N_BUFF              = TX_N_BUFF,
            TX_IN_PCT_SIZE         = TX_PCT_SIZE,
            TX_IN_PCT_HDR_SIZE     = TX_IN_PCT_HDR_SIZE,
            TX_IN_PCT_DATA_W       = STRM0_FPGA_RX_RWIDTH,

            # RX parameters
            RX_IQ_WIDTH            = LMS_DIQ_WIDTH,
            rx_int_clk_domain      = "sys",
            rx_m_clk_domain        = "sys",
        )

        self.comb += [
            # LMS7002 <-> TstTop.
            # FIXME
            #self.lms7002_top.from_tstcfg_tx_tst_i.eq(self.tst_top.tx_tst_i),
            #self.lms7002_top.from_tstcfg_tx_tst_q.eq(self.tst_top.tx_tst_q),
            #self.lms7002_top.from_tstcfg_test_en.eq( self.tst_top.test_en),

            # LMS7002 <-> PLLCFG
            #self.lms7002_top.smpl_cmp_length.eq(self.pllcfg.auto_phcfg_smpls),

            # LMS7002 <-> RXTX Top.
            self.rxtx_top.rx_path.smpl_cnt_en.eq(self.lms7002_top.smpl_cnt_en),

            # FT601 <-> RXTX Top.
            #self.ft601.stream_fifo_fpga_pc_reset_n.eq(self.rxtx_top.rx_pct_fifo_aclrn_req),
            #self.ft601.stream_fifo_pc_fpga_reset_n.eq(self.rxtx_top.rx_en),

            # General Periph <-> RXTX Top.
            #self.general_periph.tx_txant_en.eq(self.rxtx_top.tx_path.tx_txant_en),

            # General Periph <-> LMS7002
            #self.lms7002_top.periph_output_val_1.eq(self.general_periph.periph_output_val_1),
        ]

        # LMS7002 -> RX Path -> FT601 Pipeline.
        self.rx_pipeline = stream.Pipeline(
            self.lms7002_top.source,
            self.rxtx_top.rx_path.sink,
        )
        self.comb += self.rxtx_top.rx_path.source.connect(self.pcie_dma0.sink, omit=["keep", "id", "dest", "user"])

        # FT601 -> TX Path -> LMS7002 Pipeline.
        self.comb += [
            self.pcie_dma0.source.connect(self.rxtx_top.tx_path.sink, omit=["ready"]),
            self.pcie_dma0.source.ready.eq((self.rxtx_top.tx_path.sink.ready & self.fpgacfg.tx_en) | ~self.pcie_dma0.reader.enable),
        ]
        self.tx_pipeline = stream.Pipeline(
        #    self.ft601.source,
            self.rxtx_top.tx_path.source,
            self.lms7002_top.sink,
        )

        # FIXME: Not connected
        #self.comb += self.lime_top.tx_path.RESET_N.eq(self.pcie_dma0.reader.enable)

        # LMS SPI -----------------------------------------------------------------------------------
        self.add_spi_master(name="spimaster", pads=platform.request("lms7002m_spi"), data_width=32, spi_clk_freq=1e6)

        # VCTCXO -----------------------------------------------------------------------------------

        vctcxo_pads = platform.request("vctcxo")
        self.comb += vctcxo_pads.sel.eq(0)
        self.comb += vctcxo_pads.en.eq(1)

        # RF Switches ------------------------------------------------------------------------------

        rfsw_pads = platform.request("rf_switches")

        self.rfsw_control = TDDControl(platform, rfsw_pads)
        #self.comb += rfsw_pads.tx.eq(1)

        # Interrupt --------------------------------------------------------------------------------
        self.ev = EventManager()
        self.ev.clk_ctrl_irq = EventSourceProcess()
        self.ev.finalize()

        self.comb += self.ev.clk_ctrl_irq.trigger.eq((self.lms7002_top.lms7002_clk.CLK_CTRL.PHCFG_START.re  & self.lms7002_top.lms7002_clk.CLK_CTRL.PHCFG_START.storage == 1)
                                                   | (self.lms7002_top.lms7002_clk.CLK_CTRL.PLLCFG_START.re & self.lms7002_top.lms7002_clk.CLK_CTRL.PLLCFG_START.storage == 1)
                                                   | (self.lms7002_top.lms7002_clk.CLK_CTRL.PLLRST_START.re & self.lms7002_top.lms7002_clk.CLK_CTRL.PLLRST_START.storage == 1) )
        self.irq.add("lime_top")

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

    # SoC parameters.
    parser.add_argument("--with-bios",      action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-uartbone",  action="store_true", help="Enable UARTBone.")
    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        # SoC.
        soc = BaseSoC(
            board                 = args.board,
            cpu_type              = "vexriscv",
            with_bios             = args.with_bios,
            with_uartbone         = args.with_uartbone,
            with_cpu              = True,
            cpu_firmware          = None if prepare else "firmware_xtrx/firmware.bin",
            with_jtagbone         = not args.with_bscan,
            with_bscan            = args.with_bscan,
            flash_boot            = args.flash_boot,
            firmware_flash_offset = args.firmware_flash_offset,
        )
        builder = Builder(soc, csr_csv="csr.csv", bios_console="lite")
        builder.build(run=build)
        # Firmware build.
        if prepare:
            linker = {
                True  : "linker_main_ram.ld",
                False : "linker_rom.ld",
            }[args.with_bios]
            os.system(f"cd firmware_xtrx && make BUILD_DIR={builder.output_dir} LINKER={linker} clean all")

    # Generate LitePCIe Driver.
    #generate_litepcie_software(soc, "software", use_litepcie_software=args.driver)

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    # Flash Bitstream.
    if args.flash:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0, os.path.join(builder.gateware_dir, soc.build_name + ".bin"))

    # Flash Firmware.
    if args.flash_boot and args.flash:
        from litex.soc.software.crcfbigen import insert_crc
        insert_crc("firmware/firmware.bin", fbi_mode=True, o_filename="firmware/firmware.fbi", little_endian=True)
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(args.firmware_flash_offset, "firmware/firmware.fbi")


if __name__ == "__main__":
    main()
