#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from litex.build.generic_platform import *
from litex.build.altera import AlteraPlatform
from litex.build.openfpgaloader import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk.
    ("LMK_CLK", 0, Pins("B12"), IOStandard("2.5 V")),

    # Leds.
    ("FPGA_LED1_G", 0, Pins("D2"),  IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED1_R", 0, Pins("E3"),  IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED2_G", 0, Pins("J4"),  IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED2_R", 0, Pins("E1"),  IOStandard("3.3-V LVCMOS")),
    ("FX3_LED_G", 0, Pins("G5"),  IOStandard("3.3-V LVCMOS")),
    ("FX3_LED_R", 0, Pins("H5"),  IOStandard("3.3-V LVCMOS")),

    # Revision.
    ("revision", 0,
        Subsignal("HW_VER",  Pins("F20 F19 G18 H17"), IOStandard("2.5 V"), Misc("WEAK_PULL_UP_RESISTOR ON")),
        Subsignal("BOM_VER", Pins("P1 R2 U2 U1"),     IOStandard("1.8 V")),
    ),

    # GPIO.
    ("FPGA_GPIO", 0, Pins("H8 H6 H2 H1 G4 G3 F2 F1"), IOStandard("3.3-V LVCMOS")),

    # I2C.
    ("FPGA_I2C", 0,
        Subsignal("scl", Pins("H7")),
        Subsignal("sda", Pins("J7")),
        IOStandard("3.3-V LVCMOS"),
        Misc("WEAK_PULL_UP_RESISTOR ON")
    ),

    # SPI.
    ("FPGA_SPI0", 0, # LMS
        Subsignal("clk",  Pins("E6")),
        Subsignal("cs_n", Pins("D10")),
        Subsignal("mosi", Pins("D7")),
        Subsignal("miso", Pins("C8")),
        IOStandard("2.5 V")
    ),
    ("FPGA_SPI1", 0, # ADF/DAC
        Subsignal("clk",  Pins("K8")),
        Subsignal("adf_cs_n", Pins("J5")),
        Subsignal("dac_cs_n", Pins("J3"), Misc("WEAK_PULL_UP_RESISTOR ON")),
        Subsignal("mosi", Pins("L8")),
        IOStandard("3.3-V LVCMOS")
    ),

    # ADF MUXOUT.
    ("ADF_MUXOUT", 0, Pins("J2"), IOStandard("3.3-V LVCMOS")),

    # USB 3.0 (FX3).
    # PCLK declared separately to be able to request it from the platform
    # without requesting the other FX3 signals
    ("FX3_PCLK", 0, Pins("T21"), IOStandard("1.8V")),
    ("FX3", 0,
        Subsignal("dq",   Pins(
            "M19 AA21 Y22 Y21 W22 W21 W20 V22 V21 U22 U21 U20 U19 M22 M21 R22",
            "R21 R20 R19 R18 P22 P21 M20 P16 P15 N22 N21 N20 N19 N18 N17 N16"
        ), Misc("FAST_OUTPUT_REGISTER ON"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl0", Pins("L6")),
        Subsignal("ctl1", Pins("L7"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl2", Pins("M1")),
        Subsignal("ctl3", Pins("M2"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl4", Pins("M3")),
        Subsignal("ctl5", Pins("M4")),
        Subsignal("ctl7", Pins("M7"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl8", Pins("M8")),
        Subsignal("ctl11", Pins("N5"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl12", Pins("N6"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        IOStandard("1.8 V"),
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",       Pins("C6")),
        Subsignal("RXEN",        Pins("C3")),
        Subsignal("TXEN",        Pins("B10")),
        Subsignal("CORE_LDO_EN", Pins("B18")),

        # RX Interface (LMS -> FPGA).
        Subsignal("DIQ1_D",      Pins("B17 B16 B15 B14 B13 C13 A18 A17 A16 A15 A14 A13"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("TXNRX1",      Pins("B9")),
        Subsignal("ENABLE_IQSEL1",      Pins("C4"), Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("MCLK1",       Pins("G21")),
        Subsignal("FCLK1",       Pins("B20"), Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),

        # TX Interface (FPGA -> LMS).
        Subsignal("DIQ2_D",      Pins("B7 B6 B4 B3 A10 A9 A8 A7 A6 A5 A4 A3"),
            Misc("CURRENT_STRENGTH_NEW \"MINIMUM CURRENT\"")),
        Subsignal("TXNRX2",      Pins("B8")),
        Subsignal("ENABLE_IQSEL2",      Pins("C7"),  Misc("CURRENT_STRENGTH_NEW \"MINIMUM CURRENT\"")),
        Subsignal("MCLK2",       Pins("B11"), Misc("CURRENT_STRENGTH_NEW \"MINIMUM CURRENT\"")),
        Subsignal("FCLK2",       Pins("E5")),

        # IOStandard.
        IOStandard("2.5 V")
    ),

    # DDRAM.
    ("ddram", 0,
        Subsignal("a", Pins(
            "W1 AA3 Y3 V6 AA1 AB3 Y2 AB5 Y1 V7 Y6 W2 T8"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ba",    Pins("V4 V3 V1"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cas_n", Pins("T4"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cke",   Pins("R7"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("clk",   Pins("U7"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("PAD_TO_CORE_DELAY 0")),
        Subsignal("clk_n", Pins("U8"),
            Misc("CURRENT_STRENGTH_NEW 12MA")),
        Subsignal("cs_n",  Pins("R6"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("dm",    Pins("AA7 V5"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dq", Pins(
            "U10 AB8 AB7 AA9 V11 W10 AA8 Y10",
            "W6 AA5 W7 V8 W8 Y7 U9 AA4"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dqs",   Pins("AB9 V10"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("odt",   Pins("P6"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ras_n", Pins("T5"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("we_n",  Pins("T7"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        IOStandard("SSTL-18 Class I")
    ),
    ("ddram", 1,
        Subsignal("a", Pins(
            "U15 AA17 T14 Y17 T16 P7 U17 N8 R16 AA20 AB17 U16 U14"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ba",    Pins("U13 T13 V2"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cas_n", Pins("T10"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cke",   Pins("AB10"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("clk",   Pins("R14"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("PAD_TO_CORE_DELAY 0")),
        Subsignal("clk_n", Pins("R15"),
            Misc("CURRENT_STRENGTH_NEW 12MA")),
        Subsignal("cs_n",  Pins("Y8"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("dm",    Pins("AA16 AA10"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dq", Pins(
            "W15 V14 AB20 AB18 T15 W17 AB16 V15",
            "W13 AB13 AA15 AB14 AA14 AB15 AA13 U12"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dqs",   Pins("V13 Y13"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("odt",   Pins("T12"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ras_n", Pins("T11"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("we_n",  Pins("T9"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        IOStandard("SSTL-18 Class I")
    ),

    # RF Loopback Control.
    ("LB", 0,
        Subsignal("TX1_AT", Pins("E16")),
        Subsignal("TX1_H",  Pins("F15")),
        Subsignal("TX1_L",  Pins("F14")),
        Subsignal("TX1_SH", Pins("G15")),
        Subsignal("TX2_AT", Pins("E15")),
        Subsignal("TX2_H",  Pins("H11")),
        Subsignal("TX2_L",  Pins("F11")),
        Subsignal("TX2_SH", Pins("F16")),
        IOStandard("2.5 V")
    ),

    # Temperature Sensor.
    ("LM75_OS", 0, Pins("J6"), IOStandard("3.3-V LVCMOS")),

    # Fan Control.
    ("FAN_CTRL", 0, Pins("E4"), IOStandard("3.3-V LVCMOS")),

    # Bridge SPI.
    ("BRDG_SPI", 0,
        Subsignal("cs_n", Pins("K7")),
        Subsignal("miso", Pins("C1")),
        Subsignal("mosi", Pins("B1")),
        Subsignal("clk",  Pins("B2")),
        IOStandard("3.3-V LVCMOS")
    ),

    # Clock Generator (Si5351).
    ("SI_CLK", 0, Pins("T2"),    IOStandard("1.8 V")),
    ("SI_CLK", 1, Pins("AA12"),  IOStandard("1.8 V")),
    ("SI_CLK", 2, Pins("AB12"),  IOStandard("1.8 V")),
    ("SI_CLK", 3, Pins("T22"),   IOStandard("1.8 V")),
    ("SI_CLK", 5, Pins("G1"),    IOStandard("3.3-V LVCMOS")),
    ("SI_CLK", 6, Pins("AA11"),  IOStandard("1.8 V")),
    ("SI_CLK", 7, Pins("AB11"),  IOStandard("1.8 V")),

    ("EXT_GND", 0, Pins("T17"), IOStandard("1.8 V")),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
    create_rbf         = False

    def __init__(self, device="EP4CE40F23C8", diq1_slow_pins=None, **kwargs):
        # {pin index: slew rate (0 slowest .. 2 fastest, default 2)} for LMS_DIQ1_D pins.
        self.diq1_slow_pins    = dict(diq1_slow_pins or {})
        AlteraPlatform.__init__(self, device, _io, **kwargs)

        # FPGA device/bitstream parameters.
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_PACKAGE FBGA")
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_PIN_COUNT 484")
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_SPEED_GRADE 8")
        self.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL \"GLOBAL CLOCK\" -to FX3_PCLK")
        self.add_platform_command("set_global_assignment -name SDC_FILE ../../../gateware/board_specific/limesdr_usb/LMS7002_timing.sdc")
        self.add_platform_command("set_global_assignment -name SDC_FILE ../../../gateware/board_specific/limesdr_usb/FX3_timing.sdc")
        self.add_platform_command("set_global_assignment -name SDC_FILE ../../../gateware/board_specific/limesdr_usb/lms7_trx_timing.sdc")

        # RAM Style & Pass-through Logic constraints for simple_dual_port_ram (lime_txpct_fifo)
        self.add_platform_command('set_instance_assignment -name RAMSTYLE_ATTRIBUTE M9K -to "*payload_mem_i*ram*"')
        self.add_platform_command('set_instance_assignment -name ADD_PASS_THROUGH_LOGIC_TO_INFERRED_RAMS OFF -to "*payload_mem_i*ram*"')
        # Packet metadata (16 x 87 bit) in logic: as M9K it takes 3 blocks for 1.4 kbit, and the
        # EP4CE40 is M9K-bound with the DDR test core (~1.4k LE instead).
        self.add_platform_command('set_instance_assignment -name RAMSTYLE_ATTRIBUTE logic -to "*metadata_mem_i*ram*"')
        # VexRiscv register file must stay in M9K: under M9K pressure Quartus converts it to logic
        # and FX3_PCLK (sys) fails by several ns.
        self.add_platform_command('set_instance_assignment -name RAMSTYLE_ATTRIBUTE M9K -to "*RegFilePlugin_regFile*"')

    def get_verilog(self, *args, **kwargs):
        """Add Quartus RAM attributes the LiteX memory emitter cannot express (M9K budget).

        - CPU ROM byte lanes (rom_grainN, N x 8): max_depth 1024, so a 3072-deep lane is three
          1024 x 8 M9K instead of four 4096 x 2 slices (-4 M9K for the 12 KB ROM).
        With dense_ram_packing (builds without DDR, --no-ddr; DDR builds keep their packing):
        - Memories of depth <= 16 and >= 1 kbit (16 x 130 TX sample buffer): ramstyle logic
          (3 M9K saved; needs LAB headroom, fine without DDR, tight for the DDR-test build).
        - 33..36-bit memories of depth >= 512 (FX3 stream FIFOs): max_depth 256 (256 x 36 blocks).
        - Every other LiteX memory of depth > 16 and >= 1 kbit: ramstyle M9K. Close to the
          block limit, Quartus synthesis otherwise leaves some of them (e.g. the 128 x 74 RX CDC
          FIFO) in registers.
        """
        import re
        v = AlteraPlatform.get_verilog(self, *args, **kwargs)
        n_rom = n_m9k = 0
        dense = getattr(self, "dense_ram_packing", False)
        def attr(m):
            nonlocal n_rom, n_m9k
            width, name, depth = int(m.group(1)) + 1, m.group(2), int(m.group(3)) + 1
            if re.fullmatch(r"rom_grain\d+", name):
                n_rom += 1
                return '(* max_depth = 1024 *) ' + m.group(0)
            if dense and depth <= 16 and width * depth >= 1024:
                # Shallow wide buffers (16 x 130 TX sample FIFO: 3 M9K for 2 kbit) go to registers.
                return '(* ramstyle = "logic" *) ' + m.group(0)
            if dense and depth >= 512 and 33 <= width <= 36:
                # 32-bit stream FIFOs (+ last/first): 256 x 36 blocks, depth/256 M9K
                # (2048 x 34 otherwise packs as 2048 x 4 slices, 9 M9K).
                n_m9k += 1
                return '(* ramstyle = "M9K", max_depth = 256 *) ' + m.group(0)
            if depth >= (17 if dense else 16) and width * depth >= 1024:
                n_m9k += 1
                return '(* ramstyle = "M9K" *) ' + m.group(0)
            return m.group(0)
        src = re.sub(r"^reg \[(\d+):0\] (\w+)\[0:(\d+)\];$", attr, v.main_source, flags=re.M)
        if n_rom != 4:
            print(f"WARNING: limesdr_usb RAM attributes: {n_rom} ROM lanes found (expected 4).")
        v.set_main_source(src)
        return v

    def create_programmer(self, cable="ft2232", fpga_part="EP4CE40"):
        return OpenFPGALoader(cable=cable, fpga_part=fpga_part)

    def do_finalize(self, fragment):
        # Clock groups should run after all module-level SDC constraints
        self.toolchain.additional_sdc_commands.append("source ../../../gateware/board_specific/limesdr_usb/Clock_groups.sdc")
        # Gray-pointer skew / net delay for the AsyncFIFO CDCs (cut by the clock groups above).
        self.toolchain.additional_sdc_commands.append("source ../../../gateware/board_specific/limesdr_usb/CDC_constraints.sdc")

        # LMS_DIQ1_D / LMS_ENABLE_IQSEL1 output delay chain (TCO, 0 or 1 = ~0.44 ns).
        # The TX interface phase is calibrated at run time (one PLL C1 phase for all 13 lines),
        # so what matters is the skew between the lines, not the absolute delay.
        # Per-pin values copied from the 2021 QSF (D5 and IQSEL1 unassigned, the fitter picks 1).
        # They split the bus into two groups ~0.7-0.9 ns apart (slow model): D1/D7/D8/D11 early,
        # the rest late. STA (3.12 build, slow 85C) leaves only ~0.21 ns of common data-valid
        # window beyond the LMS Tsu+Th. Uniform delays (every line 1) look better in STA but were
        # worse on hardware (61.44 MSPS SISO TX+RX: no clean TX phase, D9 errors everywhere): this
        # split evidently compensates board-level skew that STA does not model.
        try:
            self.lookup_request("LMS")
            diq1_delays = [1, 0, 1, 1, 1, None, 1, 0, 0, 1, 1, 0]
            for i, delay in enumerate(diq1_delays):
                if delay is not None:
                    self.add_platform_command(f"set_instance_assignment -name CLOCK_TO_OUTPUT_DELAY {delay} -to LMS_DIQ1_D[{i}]")
            # Per-pin slew rate (experiment): hardware TX phase maps at 122.88 MHz show D9
            # switching ~4 phase steps (~0.4 ns) earlier than every other DIQ1 line (its rising
            # edge in the I half bounds the window). A slower slew delays its threshold crossing.
            for i, slew in self.diq1_slow_pins.items():
                self.add_platform_command(f"set_instance_assignment -name SLEW_RATE {slew} -to LMS_DIQ1_D[{i}]")
        except ConstraintError:
            pass

        # CKN_CK_PAIR for DDRAM (from QSF)
        try:
            self.lookup_request("ddram", 0)
            self.add_platform_command("set_instance_assignment -name CKN_CK_PAIR ON -from ddram_clk_n -to ddram_clk")
        except:
            pass

        try:
            self.lookup_request("ddram", 1)
            self.add_platform_command("set_instance_assignment -name CKN_CK_PAIR ON -from ddram_1_clk_n -to ddram_1_clk")
        except:
            pass
