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
    ("user_led", 0, Pins("N18"),  IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("V19"),  IOStandard("LVCMOS33")),
    ("user_led2", 0, Pins("G3 M2 G2"),  IOStandard("LVCMOS33")),

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

    # USB
    ("usb", 0,
        Subsignal("usb_d",    Pins("B17 A17 B16 A16 B15 A15 A14 C15")),
        Subsignal("usb_stp",  Pins("C17"), Misc("PULLUP=TRUE")),
        Subsignal("usb_clk",  Pins("C16")),
        Subsignal("usb_dir",  Pins("B18")),
        Subsignal("usb_nxt",  Pins("A18")),
        Subsignal("usb_nrst", Pins("M18"), Misc("PULLDOWN=True")),
        Subsignal("usb_26m",  Pins("E19")),
        IOStandard("LVCMOS33")
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

    # I2C buses.
    ("i2c", 0,
        Subsignal("scl", Pins("M1"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("N1"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33"),
    ),
    ("i2c", 1,
        Subsignal("scl", Pins("U14"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("U15"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33"),
    ),

    # XSYNC SPI bus.
    ("xsync_spi", 1,
        Subsignal("cs_n", Pins("H1")), # GPIO9
        Subsignal("clk",  Pins("J1")), # GPIO10
        Subsignal("mosi", Pins("N3")), # GPIO8
        IOStandard("LVCMOS33"),
    ),

    # Synchro.
    ("synchro", 0,
        Subsignal("pps_in", Pins("M3")), # GPIO0
        Subsignal("pps_out",Pins("L3")), # GPIO1
        IOStandard("LVCMOS33"),
    ),

    # GPS.
    ("gps", 0,
        Subsignal("rst", Pins("U18"), IOStandard("LVCMOS33")),
        Subsignal("pps", Pins("P3"),  Misc("PULLDOWN=True")),
        #Subsignal("tx" , Pins("N2"),  Misc("PULLUP=True")),
        #Subsignal("rx" , Pins("L1"),  Misc("PULLUP=True")),
        Subsignal("hw_s",Pins("L18"), IOStandard("LVCMOS33")),
        Subsignal("fix", Pins("R18"), IOStandard("LVCMOS33")),
        IOStandard("LVCMOS33")
    ),

    # GPS Serial.
    ("gps_serial", 0,
        Subsignal("tx", Pins("N2"), Misc("PULLUP=True")),
        Subsignal("rx", Pins("L1"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33")
     ),

    # VCTCXO.
    ("vctcxo", 0,
        Subsignal("en",  Pins("R19"), Misc("PULLUP=True")),
        Subsignal("sel", Pins("V17"), Misc("PULLDOWN=True")), # ext_clk
        #Subsignal("clk", Pins("N17"), Misc("PULLDOWN=True")),
        IOStandard("LVCMOS33")
    ),

    # GPIO (X12, 8-pin FPC connector)
    ("gpio", 0, Pins("H1 J1 K2 L2"), IOStandard("LVCMOS33")),

    # AUX.
    ("aux", 0,
        Subsignal("en_smsigio", Pins("D17")),
        Subsignal("gpio13",     Pins("T17")),
        IOStandard("LVCMOS33")
    ),

    # RF-Switches / SKY13330, SKY13384.
    ("rf_switches", 0,
        Subsignal("tx", Pins("P1"),    Misc("PULLUP=True")),
        Subsignal("rx", Pins("K3 J3"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33")
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",             Pins("U19")),
        Subsignal("CORE_LDO_EN",       Pins("W17")),
        Subsignal("RXEN",              Pins("W18")),
        Subsignal("TXEN",              Pins("W19")),

        # RX-Interface (LMS -> FPGA).
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
        #  Subsignal("diq1_0", Pins("J17") ,IOStandard("LVCMOS33")  ,),
        #  Subsignal("diq1_1", Pins("H17") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_2", Pins("H19") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_3", Pins("K17") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_4", Pins("G17") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_5", Pins("V16") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_6", Pins("J19") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_7", Pins("M19") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_8", Pins("P17") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_9", Pins("N19") ,IOStandard("LVCMOS33")  ),
        #  Subsignal("diq1_10", Pins("U17"), IOStandard("LVCMOS33") ),
        #  Subsignal("diq1_11", Pins("U16"), IOStandard("LVTTL")     ),

        #Subsignal("DIQ1_D",            Pins("J17 H17 H19 K17 G17 V16 J19 M19 P17 N19 U17 U16")),
        Subsignal("TXNRX1",            Pins("V15"), IOStandard("LVCMOS33"), Misc("SLEW=SLOW"), Drive("4")),
        Subsignal("ENABLE_IQSEL1",     Pins("P19"), IOStandard("LVCMOS33"), Misc("SLEW=FAST"), Drive("16")),
        Subsignal("MCLK1",             Pins("L17")),
        Subsignal("FCLK1",             Pins("G19"), IOStandard("LVTTL"), Misc("SLEW=FAST"), Drive("24")),

        # RX-Interface (FPGA -> LMS).
        Subsignal("DIQ2_D",            Pins("W2 U2 U3 V3 V4 V2 V5 W4 V8 U4 U8 U7")),
        Subsignal("TXNRX2_or_CLK_SEL", Pins("U5")),
        Subsignal("ENABLE_IQSEL2",     Pins("W7")),
        Subsignal("MCLK2",             Pins("W5")),
        Subsignal("FCLK2",             Pins("W6")),

        # IOStandard/Slew Rate.
        IOStandard("LVCMOS33"),
        Misc("SLEW=FAST"),
    ),

    # RF-IC / LMS7002M.
    ("lms7002m_spi", 0,
     # SPI.
        Subsignal("clk",  Pins("W14")),
        Subsignal("cs_n", Pins("W13")),
        Subsignal("mosi", Pins("W16"), Misc("PULLDOWN=True")),
        Subsignal("miso", Pins("W15"), Misc("PULLDOWN=True")),

     # IOStandard/Slew Rate.
     IOStandard("LVCMOS33"),
     Misc("SLEW=FAST"),
     ),

    # SIM.
    ("sim", 0,
        Subsignal("mode",    Pins("R3")),
        Subsignal("enable",  Pins("U1")),
        Subsignal("clk",     Pins("T1")),
        Subsignal("reset",   Pins("R2")),
        Subsignal("data",    Pins("T2")),
        IOStandard("LVCMOS33")
    ),

    # GPIO Serial.
    ("serial", 0,
        Subsignal("tx", Pins("H2")),
        Subsignal("rx", Pins("J2")),
        IOStandard("LVCMOS33")
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
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X1B200000 [current_design]",
            # "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00400000 [current_design]",
            "write_bitstream -force LimeSDR_XTRX_golden.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 LimeSDR_XTRX_golden.bit\" -file ../../../bitstream/LimeSDR_XTRX/LimeSDR_XTRX_golden.bin"
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
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X1B210000 [current_design]",
            "write_bitstream -force LimeSDR_XTRX_user.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 LimeSDR_XTRX_user.bit\" -file ../../../bitstream/LimeSDR_XTRX/LimeSDR_XTRX_user.bin",
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
