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

    ("clk26", 0, Pins("N3"), IOStandard("LVCMOS18")),

    # Leds.
    ("user_led", 0, Pins("K3"),  IOStandard("LVCMOS18")), #GPLEDINT

    # PCIe.
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("P1"), IOStandard("LVCMOS18"), Misc("PULLUP=TRUE")), # PCI_PERST_FPGA1P8
        Subsignal("clk_p", Pins("B8")),  #MGT_REF_P
        Subsignal("clk_n", Pins("A8")),  #MGT_REF_N
        Subsignal("rx_p",  Pins("B6")),  #GTP_RX_P
        Subsignal("rx_n",  Pins("A6")),  #GTP_RX_N
        Subsignal("tx_p",  Pins("B2")),  #MGTP_TX_P
        Subsignal("tx_n",  Pins("A2")),  #MGTP_TX_N
    ),

    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("P1"), IOStandard("LVCMOS18"), Misc("PULLUP=TRUE")), # PCI_PERST_FPGA1P8
        Subsignal("clk_p", Pins("B8")),    #MGT_REF_P
        Subsignal("clk_n", Pins("A8")),    #MGT_REF_N
        Subsignal("rx_p",  Pins("B6 B4")), #GTP_RX_P
        Subsignal("rx_n",  Pins("A6 A4")), #GTP_RX_N
        Subsignal("tx_p",  Pins("B2 D2")), #MGTP_TX_P
        Subsignal("tx_n",  Pins("A2 D1")), #MGTP_TX_N
    ),

    # USB
    ("usb", 0,
        Subsignal("usb_d",    Pins("A18 B16 A17 B15 A16 C15 A15 A14")),
        Subsignal("usb_stp",  Pins("C17"), Misc("PULLUP=TRUE")),
        Subsignal("usb_clk",  Pins("C16")),
        Subsignal("usb_dir",  Pins("B18")),
        Subsignal("usb_nxt",  Pins("B17")),
        Subsignal("usb_nrst", Pins("D17"), Misc("PULLDOWN=True")),
        IOStandard("LVCMOS18")
    ),

    # SPIFlash.
    ("spiflash", 0,
        Subsignal("cs_n", Pins("K19")),
        Subsignal("mosi", Pins("D18")),
        Subsignal("miso", Pins("D19")),
        Subsignal("wp",   Pins("G18")),
        Subsignal("hold", Pins("F18")),
        IOStandard("LVCMOS18")
    ),

    # I2C buses.
    ("i2c", 0,
        Subsignal("scl", Pins("V14"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("W14"), Misc("PULLUP=True")),
        IOStandard("LVCMOS18"),
    ),

    # Synchro.
    ("synchro", 0,
        Subsignal("pps_in", Pins("M3")), # GPIO0
        Subsignal("pps_out",Pins("L3")), # GPIO1
        IOStandard("LVCMOS18"),
    ),

    # VCTCXO.
    ("vctcxo", 0,
        Subsignal("sel", Pins("G17"), Misc("PULLDOWN=True")), #INTREF_SW
        IOStandard("LVCMOS18")
    ),

    # GPIO
    ("gpio", 0, Pins("L2 L1 K2 J1 H1 J2 H2 M1 N1 N2"), IOStandard("LVCMOS18")), #GPIO[6:0], GPLED[1:0]

    # RF-Switches / SKY13330, SKY13384.
    ("rf_switches", 0,
        Subsignal("tx", Pins("P3"),    Misc("PULLUP=True")),  #TXSW
        Subsignal("rx", Pins("G19"), Misc("PULLUP=True")),    #RXSW
        IOStandard("LVCMOS18")
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",             Pins("J3")),  #LMS_RESET#
        Subsignal("CORE_LDO_EN",       Pins("R19")), #LMS_LDO_EN
        Subsignal("RXEN",              Pins("R18")), #RXEN
        Subsignal("TXEN",              Pins("U14")), #TXEN

        # RX-Interface (LMS -> FPGA).
        Subsignal("diq1_0",  Pins("H17"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_1",  Pins("K18"), IOStandard("LVCMOS18"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("diq1_2",  Pins("H19"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_3",  Pins("M18"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_4",  Pins("N18"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_5",  Pins("N19"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_6",  Pins("J17"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_7",  Pins("J19"), IOStandard("LVCMOS18"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("diq1_8",  Pins("P18"), IOStandard("LVCMOS18"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("diq1_9",  Pins("J18"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_10", Pins("T17"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("diq1_11", Pins("P19"), IOStandard("LVCMOS18"), Misc("SLEW=FAST"), Drive("24")),
        Subsignal("TXNRX1",            Pins("L3"), IOStandard("LVCMOS18"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("ENABLE_IQSEL1",     Pins("M19"), IOStandard("LVCMOS18"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("MCLK1",             Pins("L17")),
        Subsignal("FCLK1",             Pins("K17"), IOStandard("LVCMOS18"), Misc("SLEW=FAST"), Drive("24")),

        # RX-Interface (FPGA -> LMS).
        Subsignal("DIQ2_D",            Pins("W16 W17 V16 U17 W18 W19 V19 U18 V17 U19 U16 U15")),
        Subsignal("TXNRX2_or_CLK_SEL", Pins("G3")),
        Subsignal("ENABLE_IQSEL2",     Pins("T18")),
        Subsignal("MCLK2",             Pins("N17")),
        Subsignal("FCLK2",             Pins("P17")),

        # IOStandard/Slew Rate.
        IOStandard("LVCMOS18"),
        Misc("SLEW=FAST"),
    ),

    # RF-IC / LMS7002M.
    ("lms7002m_spi", 0,
     # SPI.
        Subsignal("clk",  Pins("W13")),
        Subsignal("cs_n", Pins("V13 L18")),
        Subsignal("mosi", Pins("V15"), Misc("PULLDOWN=True")),
        Subsignal("miso", Pins("W15"), Misc("PULLDOWN=True")),

     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
     Misc("SLEW=FAST"),
     ),

    # RF-IC / LMS8001.
    ("lms8", 0,
        Subsignal("reset_n", Pins("E18")),
     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
     ),

    ("lms8_misc", 0,
     Subsignal("gpio", Pins("G2 H2 M3")),
     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
     ),

    # GPIO Serial.
    ("serial", 0,
        Subsignal("tx", Pins("H1")), #GPIO5 -> GPIO33_2 -> J9_2
        Subsignal("rx", Pins("M2")), #GPIO6 -> PGIO33_1 -> J9_3
        IOStandard("LVCMOS18")
    ),

    ("pwr", 0,
     Subsignal("ldoen", Pins("M3")),
     IOStandard("LVCMOS18")
     )
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
            "set_property CFGBVS GND [current_design]",
            "set_property CONFIG_VOLTAGE 1.8 [current_design]",
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
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X20200000 [current_design]",
            # "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00400000 [current_design]",
            "write_bitstream -force SSDR_golden.bit ",
            "file copy -force SSDR_golden.bit ../../../bitstream/SSDR/SSDR_golden.bit",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 SSDR_golden.bit\" -file ../../../bitstream/SSDR/SSDR_golden.bin"
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
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X20210000 [current_design]",
            "write_bitstream -force SSDR_user.bit ",
            "file copy -force SSDR_user.bit ../../../bitstream/SSDR/SSDR_user.bit",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 SSDR_user.bit\" -file ../../../bitstream/SSDR/SSDR_user.bin",

            # Set output file path and name
            "set golden_bit_path ../../../bitstream/SSDR/SSDR_golden.bit",
            "set user_bit_path ../../../bitstream/SSDR/SSDR_user.bit",
            "set bit_string   \"up 0x00000000 $golden_bit_path up 0x220000 $user_bit_path\"",
            "write_cfgmem  -format bin -force -size 4 -interface SPIx4 -loadbit $bit_string -file ../../../bitstream/SSDR/SSDR_combined.bin"
        ]

        # soc.get_build_name()
        # self.toolchain.
        self.toolchain.additional_commands = [
            # non-multiboot flash images should not be used, so we don't generate them
            # Non-Multiboot SPI-Flash bitstream generation.
            # "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file ../../../bitstream/{build_name}/{build_name}.bin",

        ]

    def create_programmer(self, cable="digilent_hs2"):
        return OpenFPGALoader(cable=cable, fpga_part=f"xc7a50tcpg236", freq=10e6)

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk26", loose=True), 1e9/26e6)
