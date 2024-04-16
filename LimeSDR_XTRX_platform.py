#
# This file is part of XTRX-Julia.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# https://www.crowdsupply.com/lime-micro/limesdr-xtrx

from litex.build.generic_platform import *
from litex.build.xilinx import Xilinx7SeriesPlatform
from litex.build.openocd import OpenOCD

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk/Rst.
    ("FPGA_CLK", 0, Pins("N17"), IOStandard("LVCMOS33")),

    # Leds.
    ("FPGA_LED1", 0, Pins("N18"),  IOStandard("LVCMOS33")),
    ("FPGA_LED2", 0, Pins("V19"),  IOStandard("LVCMOS33")),

    # PCIe.
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("T3"), IOStandard("LVCMOS25"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B8")),
        Subsignal("clk_n", Pins("A8")),
        Subsignal("rx_p",  Pins("B6")),
        Subsignal("rx_n",  Pins("A6")),
        Subsignal("tx_p",  Pins("B2")),
        Subsignal("tx_n",  Pins("A2")),
    ),

    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("T3"), IOStandard("LVCMOS25"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B8")),
        Subsignal("clk_n", Pins("A8")),
        Subsignal("rx_p",  Pins("B6 B4")),
        Subsignal("rx_n",  Pins("A6 A4")),
        Subsignal("tx_p",  Pins("B2 D2")),
        Subsignal("tx_n",  Pins("A2 D1")),
    ),

    # USB
    ("usb", 0,
        Subsignal("USB_D", Pins("B17 A17 B16 A16 B15 A15 A14 C15")),
        Subsignal("USB_STP", Pins("C17"), Misc("PULLUP=TRUE")),
        Subsignal("USB_CLK", Pins("C16")),
        Subsignal("USB_DIR", Pins("B18")),
        Subsignal("USB_NXT", Pins("A18")),
        Subsignal("USB_NRST", Pins("M18"), Misc("PULLDOWN=True")),
        Subsignal("USB_26M", Pins("E19")),
        IOStandard("LVCMOS33")
    ),

    # SPIFlash.
    ("FPGA_CFG_CS", 0, Pins("K19"), IOStandard("LVCMOS33")),
    ("flash", 0,
        Subsignal("mosi", Pins("D18")),
        Subsignal("miso", Pins("D19")),
        Subsignal("wp",   Pins("G18")),
        Subsignal("hold", Pins("F18")),
        IOStandard("LVCMOS33")
    ),

    # I2C buses.
    ("i2c", 0,
        Subsignal("scl",Pins("M1"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("N1"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33"),
    ),
    ("i2c", 1,
        Subsignal("scl", Pins("U14"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("U15"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33"),
    ),

    # GPS.
    ("gps", 0,
        Subsignal("GNSS_HW_R",    Pins("U18"), IOStandard("LVCMOS33")), # enable>>
        Subsignal("GNSS_HW_S",    Pins("L18"), IOStandard("LVCMOS33")),
        Subsignal("GNSS_1PPS",    Pins("P3"),  Misc("PULLDOWN=True")),
        Subsignal("GNSS_TXD" ,    Pins("N2"),  Misc("PULLUP=True")),
        Subsignal("GNSS_RXD" ,    Pins("L1"),  Misc("PULLUP=True")),
        IOStandard("LVCMOS33")
    ),

    # VCTCXO.
    ("vctcxo", 0,
        Subsignal("EN_TCXO",    Pins("R19"), Misc("PULLUP=True")),
        Subsignal("EXT_CLK",    Pins("V17"), Misc("PULLDOWN=True")), # ext_clk
        Subsignal("FPGA_CLK",   Pins("N17"), Misc("PULLDOWN=True")),
        IOStandard("LVCMOS33")
    ),

    # RF-Switches / SKY13330, SKY13384.
    ("rf_switches", 0,
        Subsignal("TX_SW", Pins("P1"),    Misc("PULLUP=True")),
        Subsignal("RX_SW", Pins("K3 J3"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33")
    ),

    # RF-IC / LMS7002M.
    ("lms7002m", 0,
        # Control.
        Subsignal("LMS_RST",        Pins("U19")),
        Subsignal("LMS_CORE_LDO_EN",Pins("W17")),
        Subsignal("LMS_RXEN",       Pins("W18")),
        Subsignal("LMS_TXEN",       Pins("W19")),

        # SPI.
        Subsignal("FPGA_SPI_SCLK",  Pins("W14")),
        Subsignal("FPGA_SPI_LMS_SS",Pins("W13")),
        Subsignal("FPGA_SPI_MOSI",  Pins("W16"), Misc("PULLDOWN=True")),
        Subsignal("FPGA_SPI_MISO",  Pins("W15"), Misc("PULLDOWN=True")),

        # RX-Interface (LMS -> FPGA).
        Subsignal("LMS_DIQ1",       Pins("J17 H17 H19 K17 G17 V16 J19 M19 P17 N19 U17 U16")),
        Subsignal("LMS_TXNRX1",     Pins("V15")),
        Subsignal("LMS_EN_IQSEL1",  Pins("P19")),
        Subsignal("LMS_MCLK1",      Pins("L17")),
        Subsignal("LMS_FCLK1",      Pins("G19")),

        # RX-Interface (FPGA -> LMS).
        Subsignal("LMS_DIQ2",       Pins("W2 U2 U3 V3 V4 V2 V5 W4 V8 U4 U8 U7")),
        Subsignal("LMS_TXNRX2",     Pins("U5")),
        Subsignal("LMS_EN_IQSEL2",  Pins("W7")),
        Subsignal("LMS_MCLK2",      Pins("W5")),
        Subsignal("LMS_FCLK2",      Pins("W6")),

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

]

# Platform -----------------------------------------------------------------------------------------

class Platform(Xilinx7SeriesPlatform):
    default_clk_name   = "vctcxo_FPGA_CLK"
    default_clk_period = 1e9/26e6
    dev_short_string = ""

    def __init__(self, variant="xc7a50t", toolchain="vivado"):
        assert variant in ["xc7a35t", "xc7a50t"]
        self.variant = variant
        device = {
            "xc7a35t" : "xc7a35tcpg236-3",
            "xc7a50t" : "xc7a50tcpg236-2",
        }[variant]

        if variant == "xc7a50t":
            self.dev_short_string = "a50t"
        else:
            self.dev_short_string = "a35t"

        Xilinx7SeriesPlatform.__init__(self, "xc7a50tcpg236-2", _io, toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.UNUSEDPIN Pulldown [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN Disable [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 66 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]",
            #"set_property BITSTREAM.config.SPI_opcode 0x6B [current_design ]",

            # Xilinx tools ask for this
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]
        self.toolchain.additional_commands = [
            # Non-Multiboot SPI-Flash bitstream generation.
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin",

            # Multiboot SPI-Flash Operational bitstream generation.
            "set_property BITSTREAM.CONFIG.TIMER_CFG 0x0001fbd0 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK Enable [current_design]",
            "write_bitstream -force {build_name}_operational.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}_operational.bit\" -file {build_name}_operational.bin",

            # Multiboot SPI-Flash Fallback bitstream generation.
            "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00400000 [current_design]",
            "write_bitstream -force {build_name}_fallback.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}_fallback.bit\" -file {build_name}_fallback.bin"
        ]

    def create_programmer(self):
        return OpenOCD("openocd_xc7_ft2232.cfg", "bscan_spi_xc7"+self.dev_short_string+".bit")

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("vctcxo_FPGA_CLK", loose=True), 1e9/26e6)
