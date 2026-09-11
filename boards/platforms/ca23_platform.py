#
# This file is part of LimeSDR_GW and adapted from LiteX-Boards XTRX platform file.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from litex.build.generic_platform import *
from litex.build.xilinx import Xilinx7SeriesPlatform
from litex.build.openfpgaloader import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk/Rst.
    ("clk26", 0, Pins("N17"), IOStandard("LVCMOS33")),

    # Leds.
    ("user_led", 0, Pins("R18"), IOStandard("LVCMOS33")), # FPGA_LED_R
    ("user_led", 1, Pins("U14"), IOStandard("LVCMOS33")), # FPGA_LED_G

    # PCIe.
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("T3"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B8")),
        Subsignal("clk_n", Pins("A8")),
        Subsignal("rx_p",  Pins("B6")),
        Subsignal("rx_n",  Pins("A6")),
        Subsignal("tx_p",  Pins("B2")),
        Subsignal("tx_n",  Pins("A2")),
    ),

    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("T3"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B8")),
        Subsignal("clk_n", Pins("A8")),
        Subsignal("rx_p",  Pins("B6 B4")),
        Subsignal("rx_n",  Pins("A6 A4")),
        Subsignal("tx_p",  Pins("B2 D2")),
        Subsignal("tx_n",  Pins("A2 D1")),
    ),

    # SPIFlash.
    ("flash_cs_n", 0, Pins("K19"), IOStandard("LVCMOS33")),
    ("flash", 0,
        Subsignal("mosi", Pins("D18")),
        Subsignal("miso", Pins("D19")),
        Subsignal("wp",   Pins("G18")),
        Subsignal("hold", Pins("F18")),
        IOStandard("LVCMOS33")
    ),
    ("spiflash", 0,
        Subsignal("cs_n", Pins("K19")),
        Subsignal("mosi", Pins("D18")),
        Subsignal("miso", Pins("D19")),
        Subsignal("wp",   Pins("G18")),
        Subsignal("hold", Pins("F18")),
        IOStandard("LVCMOS33")
    ),

    # AD5662 VCTCXO SPI DAC (Bank 34)
    ("vctcxo_dac_spi", 0,
        Subsignal("clk",  Pins("R2")),
        Subsignal("mosi", Pins("T2")),
        Subsignal("cs_n", Pins("R3")),
        IOStandard("LVCMOS33")
    ),
    ("ad5662_spi", 0,
        Subsignal("clk",  Pins("R2")),
        Subsignal("mosi", Pins("T2")),
        Subsignal("cs_n", Pins("R3")),
        IOStandard("LVCMOS33")
    ),

    # TDD RF Switch
    ("rf_sw_tdd", 0, Pins("U15"), IOStandard("LVCMOS33")),

    # MIPI RFFE RF Switches (Bank 34)
    ("mipi_rffe", 0, Subsignal("sclk", Pins("G3")), Subsignal("sdata", Pins("G2")), IOStandard("LVCMOS18")), # RX1_RF
    ("mipi_rffe", 1, Subsignal("sclk", Pins("H2")), Subsignal("sdata", Pins("J2")), IOStandard("LVCMOS18")), # TRX1_RF
    ("mipi_rffe", 2, Subsignal("sclk", Pins("H1")), Subsignal("sdata", Pins("J1")), IOStandard("LVCMOS18")), # TRX1_ANT
    ("mipi_rffe", 3, Subsignal("sclk", Pins("K2")), Subsignal("sdata", Pins("L2")), IOStandard("LVCMOS18")), # RX2_RF
    ("mipi_rffe", 4, Subsignal("sclk", Pins("L1")), Subsignal("sdata", Pins("J3")), IOStandard("LVCMOS18")), # TRX2_RF
    ("mipi_rffe", 5, Subsignal("sclk", Pins("K3")), Subsignal("sdata", Pins("L3")), IOStandard("LVCMOS18")), # TRX2_ANT

    ("rx1_rf_sw",   0, Subsignal("sclk", Pins("G3")), Subsignal("sdata", Pins("G2")), IOStandard("LVCMOS18")),
    ("trx1_rf_sw",  0, Subsignal("sclk", Pins("H2")), Subsignal("sdata", Pins("J2")), IOStandard("LVCMOS18")),
    ("trx1_ant_sw", 0, Subsignal("sclk", Pins("H1")), Subsignal("sdata", Pins("J1")), IOStandard("LVCMOS18")),
    ("rx2_rf_sw",   0, Subsignal("sclk", Pins("K2")), Subsignal("sdata", Pins("L2")), IOStandard("LVCMOS18")),
    ("trx2_rf_sw",  0, Subsignal("sclk", Pins("L1")), Subsignal("sdata", Pins("J3")), IOStandard("LVCMOS18")),
    ("trx2_ant_sw", 0, Subsignal("sclk", Pins("K3")), Subsignal("sdata", Pins("L3")), IOStandard("LVCMOS18")),

    # Revision & Status (Bank 14)
    ("revision", 0,
        Subsignal("BOM_VER", Pins("J18 T18 V14 V7")),
        Subsignal("HW_VER",  Pins("V13 E19 K18 D17")),
        IOStandard("LVCMOS33"),
    ),

    # FPGA GPIO (Bank 14)
    ("fpga_gpio", 0, Pins("L18 N18 V19 V17"), IOStandard("LVCMOS33")),
    ("fpga_dsw_bit2", 0, Pins("W3"), IOStandard("LVCMOS33")),

    # Sync Signals (Bank 14)
    ("fpga_sync_out1", 0, Pins("T17"), IOStandard("LVCMOS33")),
    ("fpga_sync_out2", 0, Pins("U18"), IOStandard("LVCMOS33")),
    ("rpi_sync_out",   0, Pins("P18"), IOStandard("LVCMOS33")),

    # GNSS Module (Bank 16)
    ("gnss", 0,
        Subsignal("extint",  Pins("A14")),
        Subsignal("reset",   Pins("A15")),
        Subsignal("tpulse",  Pins("C16")), # PPS
        IOStandard("LVCMOS33")
    ),
    ("gnss_serial", 0,
        Subsignal("tx", Pins("B15")), # FPGA TX -> GNSS RX
        Subsignal("rx", Pins("C15")), # GNSS TX -> FPGA RX
        IOStandard("LVCMOS33")
    ),
    ("gps_serial", 0,
        Subsignal("tx", Pins("B15")),
        Subsignal("rx", Pins("C15")),
        IOStandard("LVCMOS33")
    ),
    ("gnss_ddc", 0,
        Subsignal("scl", Pins("A16")),
        Subsignal("sda", Pins("A17")),
        IOStandard("LVCMOS33")
    ),

    # I2C Slave Bus (Bank 14)
    ("fpga_i2c", 0,
        Subsignal("sda", Pins("M18"), Misc("PULLUP=True")),
        Subsignal("scl", Pins("R19"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33"),
    ),

    # Raspberry Pi SPI (Bank 16)
    ("rpi_spi", 0,
        Subsignal("sclk", Pins("B16")),
        Subsignal("mosi", Pins("C17")),
        Subsignal("miso", Pins("B17")),
        Subsignal("ss1",  Pins("B18")),
        Subsignal("ss2",  Pins("A18")),
        IOStandard("LVCMOS33")
    ),

    # Raspberry Pi UART (Bank 34)
    ("rpi_uart", 0,
        Subsignal("rx", Pins("T1")),
        Subsignal("tx", Pins("U1")),
        IOStandard("LVCMOS33")
    ),
    ("serial", 0,
        Subsignal("rx", Pins("T1")),
        Subsignal("tx", Pins("U1")),
        IOStandard("LVCMOS33")
    ),

    # M.2 Signals (Bank 34)
    ("m2", 0,
        Subsignal("devslp",     Pins("M3")),
        Subsignal("coex1",      Pins("M2")),
        Subsignal("coex2",      Pins("M1")),
        Subsignal("coex3",      Pins("N2")),
        Subsignal("w_disable_2", Pins("N1")),
        Subsignal("dpr",        Pins("N3")),
        Subsignal("reset",      Pins("P3")),
        Subsignal("fcp_off",    Pins("P1")),
        IOStandard("LVCMOS18")
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",             Pins("U19")),
        Subsignal("CORE_LDO_EN",       Pins("W17")),
        Subsignal("RXEN",              Pins("W18")),
        Subsignal("TXEN",              Pins("W19")),

        # TX-Interface LMS Port 1 (FPGA -> LMS).
        Subsignal("diq1_0",  Pins("J17"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_1",  Pins("H17"), IOStandard("LVCMOS33"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("diq1_2",  Pins("H19"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_3",  Pins("K17"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_4",  Pins("G17"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_5",  Pins("V16"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_6",  Pins("J19"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_7",  Pins("M19"), IOStandard("LVCMOS33"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("diq1_8",  Pins("P17"), IOStandard("LVCMOS33"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("diq1_9",  Pins("N19"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_10", Pins("U17"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_11", Pins("U16"), IOStandard("LVTTL"),    Misc("SLEW=FAST"), Drive("24")),

        Subsignal("TXNRX1",            Pins("V15"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("ENABLE_IQSEL1",     Pins("P19"), IOStandard("LVCMOS33"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("MCLK1",             Pins("L17")),
        Subsignal("FCLK1",             Pins("G19"), IOStandard("LVTTL"), Misc("SLEW=FAST"), Drive("24")),

        # RX-Interface LMS Port 2 (LMS -> FPGA).
        Subsignal("DIQ2_D",            Pins("W2 U2 U3 V3 V4 V2 V5 W4 V8 U4 U8 U7")),
        Subsignal("TXNRX2",            Pins("U5")),
        Subsignal("ENABLE_IQSEL2",     Pins("W7")),
        Subsignal("MCLK2",             Pins("W5")),
        Subsignal("FCLK2",             Pins("W6")),

        # IOStandard/Slew Rate.
        IOStandard("LVCMOS33"),
        Misc("SLEW=FAST"),
    ),

    # RF-IC / LMS7002M SPI.
    ("lms7002m_spi", 0,
        Subsignal("clk",  Pins("W14")),
        Subsignal("cs_n", Pins("W13")),
        Subsignal("mosi", Pins("W16"), Misc("PULLDOWN=True")),
        Subsignal("miso", Pins("W15"), Misc("PULLDOWN=True")),
        IOStandard("LVCMOS33"),
        Misc("SLEW=FAST"),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(Xilinx7SeriesPlatform):
    default_clk_name   = "clk26"
    default_clk_period = 1e9/26e6

    def __init__(self, toolchain="vivado"):
        Xilinx7SeriesPlatform.__init__(self, "xc7a50tcpg236-2", _io, toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.UNUSEDPIN Pulldown [current_design]",
            "set_property CONFIG_MODE SPIx4 [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN Disable [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 66 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]
        # TODO: set multiboot adress as a variable somewhere else instead of hardcoding it here
        self.gold_img_commands = [
            # Multiboot SPI-Flash Golden bitstream generation.
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK ENABLE [current_design]",
            "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 32'h00220000 [current_design]",
            "set_property BITSTREAM.CONFIG.TIMER_CFG 0x493E0 [current_design]",
            # USR_ACCESS Field at 00007C-00007F binfile For Bitstream identification:
            #[32:24] - DEVICE ID
            #[23:20] - HW_VER
            #[19:16] - Image identifier ( 0 - Gold image, 1- User image)
            #[15: 0] - Reserved
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X21200000 [current_design]",
            # "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00400000 [current_design]",
            "write_bitstream -force CA23_golden.bit ",
            "file copy -force CA23_golden.bit ../../../bitstream/CA23/CA23_golden.bit",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 CA23_golden.bit\" -file ../../../bitstream/CA23/CA23_golden.bin"
        ]
        self.user_img_commands = [

            # Multiboot SPI-Flash user bitstream generation.
            "set_property BITSTREAM.CONFIG.TIMER_CFG 0x493E0 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK Enable [current_design]",
            # USR_ACCESS Field at 00007C-00007F binfile For Bitstream identification:
            #[32:24] - DEVICE ID
            #[23:20] - HW_VER
            #[19:16] - Image identifier ( 0 - Gold image, 1- User image)
            #[15: 0] - Reserved
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X21210000 [current_design]",
            "write_bitstream -force CA23_user.bit ",
            "file copy -force CA23_user.bit ../../../bitstream/CA23/CA23_user.bit",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 CA23_user.bit\" -file ../../../bitstream/CA23/CA23_user.bin",

            # Set output file path and name
            "set golden_bit_path ../../../bitstream/CA23/CA23_golden.bit",
            "set user_bit_path ../../../bitstream/CA23/CA23_user.bit",
            "set bit_string   \"up 0x00000000 $golden_bit_path up 0x220000 $user_bit_path\"",
            "write_cfgmem  -format bin -force -size 4 -interface SPIx4 -loadbit $bit_string -file ../../../bitstream/CA23/CA23_combined.bin"
        ]

        # soc.get_build_name()
        # self.toolchain.
        self.toolchain.additional_commands = [
            # non-multiboot flash images should not be used, so we don't generate them
            # Non-Multiboot SPI-Flash bitstream generation.
            # "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file ../../../bitstream/{build_name}/{build_name}.bin",

        ]

    def create_programmer(self, cable="digilent_hs2"):
        return OpenFPGALoader(cable=cable, fpga_part=f"xc7a50tcpg236", freq=10e5)

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk26", loose=True), 1e9/26e6)
