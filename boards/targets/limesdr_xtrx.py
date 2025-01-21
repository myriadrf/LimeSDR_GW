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

from gateware.aux                             import AUX
from gateware.fpgacfg                         import FPGACfg
from gateware.GpioTop                         import GpioTop
from gateware.rxtx_top                        import RXTXTop

from gateware.LimeDFB_LiteX.lms7002.src.lms7002_top           import LMS7002Top

from gateware.xtrx_rfsw import xtrx_rfsw

from litepcie.software import generate_litepcie_software, generate_litepcie_software_headers

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
        flash_boot            = False,
        gold_img              = False,
        firmware_flash_offset = 0x220000,
    ):

        # Platform ---------------------------------------------------------------------------------
        platform = {
            #"fairwaves_cs"  : fairwaves_xtrx_platform.Platform(variant="xc7a35t"),
            #"fairwaves_pro" : fairwaves_xtrx_platform.Platform(variant="xc7a50t"),
            "limesdr"       : limesdr_xtrx_platform.Platform()
        }[board]

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

        # FPGA Cfg ---------------------------------------------------------------------------------
        self.fpgacfg  = FPGACfg(platform, board_id=27, major_rev=2, compile_rev=25)

        # PLL Cfg ----------------------------------------------------------------------------------
        #self.pllcfg = PLLCfg()

        # LMS7002 Top ------------------------------------------------------------------------------
        self.lms7002_top = LMS7002Top(
            platform           = platform,
            pads               = platform.request("LMS"),
            hw_ver             = Constant(0, 4),
            add_csr            = True,
            fpgacfg_manager    = self.fpgacfg,
            diq_width          = LMS_DIQ_WIDTH,
            invert_input_clock = False,
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
            tx_s_clk_domain        = "sys",

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
        self.comb += self.rxtx_top.rx_path.source.connect(self.pcie_dma0.sink, keep={"valid", "ready", "last", "data"}),

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

        self.comb += self.rxtx_top.tx_path.ext_reset_n.eq(self.pcie_dma0.reader.enable)

        # LMS SPI -----------------------------------------------------------------------------------
        self.add_spi_master(name="spimaster", pads=platform.request("lms7002m_spi"), data_width=32, spi_clk_freq=1e6)

        # VCTCXO -----------------------------------------------------------------------------------

        vctcxo_pads = platform.request("vctcxo")
        self.comb += vctcxo_pads.sel.eq(self.fpgacfg.ext_clk)
        self.comb += vctcxo_pads.en.eq(self.fpgacfg.tcxo_en)

        # RF Switches ------------------------------------------------------------------------------

        rfsw_pads = platform.request("rf_switches")

        self.rfsw_control = xtrx_rfsw(platform, rfsw_pads)
        #self.comb += rfsw_pads.tx.eq(1)

        # Interrupt --------------------------------------------------------------------------------
        class LimeTop(LiteXModule):
            def __init__(self, lms7002_clk):
                self.ev = EventManager()
                self.ev.clk_ctrl_irq = EventSourceProcess()
                self.ev.finalize()

                self.comb += self.ev.clk_ctrl_irq.trigger.eq((lms7002_clk.CLK_CTRL.PHCFG_START.re  & lms7002_clk.CLK_CTRL.PHCFG_START.storage == 1)
                    | (lms7002_clk.CLK_CTRL.PLLCFG_START.re & lms7002_clk.CLK_CTRL.PLLCFG_START.storage == 1)
                    | (lms7002_clk.CLK_CTRL.PLLRST_START.re & lms7002_clk.CLK_CTRL.PLLRST_START.storage == 1) )

        self.lime_top = LimeTop(self.lms7002_top.lms7002_clk)

        self.irq.add("lime_top")

        # GPS serial connected to LimeUART0
        from litex.soc.cores.uart import UARTPHY
        from litex.soc.cores.uart import UART

        gps_pads = platform.request("gps")
        gnss_uart_pads = self.platform.request("gps_serial", loose=True)
        gnss_uart_phy = UARTPHY(gnss_uart_pads, clk_freq=self.sys_clk_freq, baudrate=9600)
        pcie_uart0 = UART(gnss_uart_phy, tx_fifo_depth=16, rx_fifo_depth=16, rx_fifo_rx_we=True)
        self.add_module(name=f"PCIE_UART0_phy", module=gnss_uart_phy)
        self.add_module(name="PCIE_UART0", module=pcie_uart0)

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
    parser.add_argument("--cable",                 default="digilent_hs2",  help="JTAG cable.")
    parser.add_argument("--driver",                action="store_true",     help="Generate PCIe driver from LitePCIe (override local version).")
    parser.add_argument("--flash-boot",            action="store_true",     help="Write Firmware in Flash instead of RAM.")
    parser.add_argument("--firmware-flash-offset", default=0x220000,        help="Firmware SPI Flash offset.")
    parser.add_argument("--gold",                  action="store_true",     help="Build/Flash golden image instead of user")

    # SoC parameters.
    parser.add_argument("--with-bios",      action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-uartbone",  action="store_true", help="Enable UARTBone.")

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
            cpu_firmware          = None if prepare else "firmware_xtrx/firmware.bin",
            with_jtagbone         = not args.with_bscan,
            with_bscan            = args.with_bscan,
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
            os.system(f"cd firmware_xtrx && make BUILD_DIR={builder.output_dir} LINKER={linker} clean all")

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
