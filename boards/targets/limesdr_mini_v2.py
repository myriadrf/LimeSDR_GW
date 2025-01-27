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
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

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

from gateware.fpgacfg             import FPGACfg
from gateware.pllcfg              import PLLCfg
from gateware.rxtx_top            import RXTXTop

from gateware.LimeDFB_LiteX.lms7002.src.lms7002_top           import LMS7002Top
from gateware.LimeDFB_LiteX.general.busy_delay                import BusyDelay
from gateware.LimeDFB_LiteX.general_periph.src.general_periph import GeneralPeriphTop
from gateware.LimeDFB_LiteX.FT601.src.ft601                   import FT601
from gateware.LimeDFB_LiteX.self_test.src.tst_top             import TstTop

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

        # # #

        # Clk.
        self.ft_clk = platform.request("FT_CLK")

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

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=77.5e6, cpu_type="vexriscv", toolchain="trellis",
        with_bios      = False,
        with_rx_tx_top = True,
        with_uartbone  = False,
        with_spi_flash = False,
        cpu_firmware   = None,
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
            with_uartbone            = with_uartbone,
            uart_name                = {True: "crossover", False:"serial"}[with_uartbone],
        )

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
        self.add_spi_master(name="spimaster", pads=platform.request("FPGA_SPI"), data_width=32, spi_clk_freq=10e6)

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes

            self.add_spi_flash(mode="1x", clk_freq=100_000, module=W25Q128JV(Codes.READ_1_1_1), with_master=True)

        # mico32_busy(gpo) & busy_delay ------------------------------------------------------------
        self._gpo = CSRStorage(description="GPO interface", fields=[
            CSRField("mico32_busy", size=1, offset=0, description="CPU state.", values=[
                ("``0b0``", "IDLE."),
                ("``0b1``", "BUSY."),
            ])
        ])

        self.busy_delay  = BusyDelay(platform, "sys", 25, 100)
        self.comb       += self.busy_delay.busy_in.eq(self._gpo.fields.mico32_busy)

        # FPGA Cfg ---------------------------------------------------------------------------------
        revision_pads = platform.request("revision")
        self.fpgacfg  = FPGACfg(platform, board_id=0x0011, major_rev=2, compile_rev=7, pads=revision_pads)
        self.comb += self.fpgacfg.pwr_src.eq(0)

        # PLL Cfg ----------------------------------------------------------------------------------
        self.pllcfg = PLLCfg()

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

        # LMS7002 Top ------------------------------------------------------------------------------
        self.lms7002_top = LMS7002Top(
            platform        = platform,
            pads            = platform.request("LMS"),
            hw_ver          = revision_pads.HW_VER,
            add_csr         = True,
            fpgacfg_manager = self.fpgacfg,
            pllcfg_manager  = self.pllcfg,
            diq_width       = LMS_DIQ_WIDTH,
        )

        # Tst Top / Clock Test ---------------------------------------------------------------------
        self.tst_top = TstTop(platform, self.crg.ft_clk, platform.request("LMK_CLK"))

        # General Periph ---------------------------------------------------------------------------

        gpio_pads     = platform.request("FPGA_GPIO")
        #egpio_pads    = platform.request("FPGA_EGPIO")

        self.general_periph = GeneralPeriphTop(platform,
            revision_pads = revision_pads,
            gpio_pads     = gpio_pads,
            gpio_len      = len(gpio_pads),
            egpio_pads    = None,
            egpio_len     = 2,
        )

        self.comb += [
            self.general_periph.led1_mico32_busy.eq(self.busy_delay.busy_out),
            self.general_periph.ep03_active.eq(self.ft601.rd_active),
            self.general_periph.ep83_active.eq(self.ft601.wr_active),
        ]

        # RXTX Top ---------------------------------------------------------------------------------
        if with_rx_tx_top:
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
                self.lms7002_top.from_tstcfg_tx_tst_i.eq(self.tst_top.tx_tst_i),
                self.lms7002_top.from_tstcfg_tx_tst_q.eq(self.tst_top.tx_tst_q),
                self.lms7002_top.from_tstcfg_test_en.eq( self.tst_top.test_en),

                # LMS7002 <-> PLLCFG
                self.lms7002_top.smpl_cmp_length.eq(self.pllcfg.auto_phcfg_smpls),

                # LMS7002 <-> RXTX Top.
                self.rxtx_top.rx_path.smpl_cnt_en.eq(self.lms7002_top.smpl_cnt_en),

                # FT601 <-> RXTX Top.
                self.ft601.stream_fifo_fpga_pc_reset_n.eq(self.rxtx_top.rx_pct_fifo_aclrn_req),
                self.ft601.stream_fifo_pc_fpga_reset_n.eq(self.rxtx_top.rx_en),

                # General Periph <-> RXTX Top.
                self.general_periph.tx_txant_en.eq(self.rxtx_top.tx_path.tx_txant_en),

                # General Periph <-> LMS7002
                self.lms7002_top.periph_output_val_1.eq(self.general_periph.periph_output_val_1),
            ]

            # LMS7002 -> RX Path -> FT601 Pipeline.
            self.rx_pipeline = stream.Pipeline(
                self.lms7002_top.source,
                self.rxtx_top.rx_path,
                self.ft601.sink,
            )

            # FT601 -> TX Path -> LMS7002 Pipeline.
            self.tx_pipeline = stream.Pipeline(
                self.ft601.source,
                self.rxtx_top.tx_path,
                self.lms7002_top.sink,
            )

        # RF Switches ------------------------------------------------------------------------------

        rfsw_pads  = platform.request("RFSW")
        tx_lb_pads = platform.request("TX_LB")

        self.gpio = CSRStorage(16, reset=0b0001000101000100) # fpgacfg @23
        self.comb += [
            # RF Switch.
            rfsw_pads.RX_V1.eq(self.gpio.storage[8]),
            rfsw_pads.RX_V2.eq(self.gpio.storage[9]),
            rfsw_pads.TX_V1.eq(self.gpio.storage[12]),
            rfsw_pads.TX_V2.eq(self.gpio.storage[13]),

            # TX
            tx_lb_pads.AT.eq(  self.gpio.storage[1]),
            tx_lb_pads.SH.eq(  self.gpio.storage[2]),
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

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR-Mini-V2 LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--toolchain",    default="trellis",   help="Build toolchain.", choices=["diamond", "trellis"])
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--flash",        action="store_true", help="Flash bitstream.")
    parser.add_argument("--flash-user",   action="store_true", help="Flash User bitstream.")
    parser.add_argument("--flash-golden", action="store_true", help="Flash Golden bitstream.")
    parser.add_argument("--cable",        default="ft2232",    help="JTAG cable.")

    # SoC parameters.
    parser.add_argument("--with-bios",         action="store_true", help="Enable LiteX BIOS.")
    parser.add_argument("--with-uartbone",     action="store_true", help="Enable UARTBone.")
    parser.add_argument("--without-spi-flash", action="store_true", help="Disable SPI Flash (MMAPed).")
    parser.add_argument("--cpu-type",          default="vexriscv",  help="Select CPU.", choices=[
        "vexriscv", "picorv32", "fazyrv", "firev"]),

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
            cpu_firmware   = None if prepare else "firmware_mini/firmware.bin",
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
            os.system(f"cd firmware_mini && make BUILD_DIR={builder.output_dir} TARGET={soc.platform.name.upper()} LINKER={linker} clean all")

    # Prepare User/Golden bitstream.
    if args.toolchain == "diamond":
        os.system(f"./limesdr_mini_v2_bitstream.py")
    else:
        golden = soc.platform.name + "_golden.bit"
        user   = builder.get_bitstream_filename(mode="sram", ext=".bit")
        cmd = f"ecpmulti --flashsize 128 --input {golden} --input {user} --address 0x00280000 limesdr_mini_v2.bin"
        os.system(cmd)
        os.system("srec_cat limesdr_mini_v2.bin -Binary -Bit_Reverse=2 -Byte-Swap -o limesdr_mini_v2.mcs -Intel")

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bit"))

    # Flash Bitstream.
    if args.flash:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0, builder.get_bitstream_filename(mode="sram", ext=".bit"))

    # Flash Golden Bitstream.
    if args.flash_golden:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0x00140000, soc.platform.name + "_golden.bit")

    # Flash User Bitstream.
    if args.flash_user:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0x00280000, soc.platform.name + ".bit")

if __name__ == "__main__":
    main()
