#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2021-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2024 Gwenhael Goavec-Merou <gwenhael@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# https://www.crowdsupply.com/lime-micro/limesdr-xtrx

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxUSPPlatform
from litex.build.openfpgaloader import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk/Rst.
    ("clk26", 0, Pins("A9"), IOStandard("LVCMOS33")),

    ("FPGA_SYSREF", 0,
     Subsignal("p", Pins("T24")),
     Subsignal("n", Pins("U24")),
     IOStandard("LVDS")
     ),

    ("FPGA_1PPS", 0,
     Subsignal("p", Pins("T25")),
     Subsignal("n", Pins("U25")),
     IOStandard("LVDS")
     ),


    # Leds.
    ("user_led", 0, Pins("AD13"),  IOStandard("LVCMOS33")),  # LED33_0
    ("user_led", 1, Pins("AF13"),  IOStandard("LVCMOS33")),  # LED33_1
    ("user_led", 2, Pins("AE13"),  IOStandard("LVCMOS33")),  # LED33_2
    ("user_led2", 0, Pins("B10 B11 C9"),  IOStandard("LVCMOS33")),

    # PCIe.
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("T3"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("AB7")),
        Subsignal("clk_n", Pins("AB6")),
        Subsignal("rx_p",  Pins("B6")),
        Subsignal("rx_n",  Pins("A6")),
        Subsignal("tx_p",  Pins("B2")),
        Subsignal("tx_n",  Pins("A2")),
    ),

    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("T3"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("AB7")),
        Subsignal("clk_n", Pins("AB6")),
        Subsignal("rx_p",  Pins("B6 B4")),
        Subsignal("rx_n",  Pins("A6 A4")),
        Subsignal("tx_p",  Pins("B2 D2")),
        Subsignal("tx_n",  Pins("A2 D1")),
    ),

    ("pcie_x4", 0,
     #Subsignal("rst_n", Pins("AF13"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
     Subsignal("clk_p", Pins("AB7")),
     Subsignal("clk_n", Pins("AB6")),
     Subsignal("rx_p", Pins("AF2 AE4 AD2 AB2")),
     Subsignal("rx_n", Pins("AF1 AE3 AD1 AB1")),
     Subsignal("tx_p", Pins("AF7 AE9 AD7 AC5")),
     Subsignal("tx_n", Pins("AF6 AE8 AD6 AC4")),
     ),

    ("afe79xx_serdes_x8", 0,
     Subsignal("fpga_gt_aferef_p", Pins("M7")),
     Subsignal("fpga_gt_aferef_n", Pins("M6")),
     Subsignal("fpga_grx_p", Pins("Y2 V2 T2 P2 M2 K2 H2 F2")),
     Subsignal("fpga_grx_n", Pins("Y1 V1 T1 P1 M1 K1 H1 F1")),
     Subsignal("fpga_gtx_p", Pins("AA5 W5 U5 R5 N5 L5 J5 G5")),
     Subsignal("fpga_gtx_n", Pins("AA4 W4 U4 R4 N4 L4 J4 G4")),
     ),

    ("afe79xx_serdes_x4", 0,
    Subsignal("fpga_gt_aferef_p", Pins("M7")),
    Subsignal("fpga_gt_aferef_n", Pins("M6")),
    Subsignal("fpga_grx_p", Pins("Y2 V2 T2 P2")),
    Subsignal("fpga_grx_n", Pins("Y1 V1 T1 P1")),
    Subsignal("fpga_gtx_p", Pins("AA5 W5 U5 R5")),
    Subsignal("fpga_gtx_n", Pins("AA4 W4 U4 R4")),
    Subsignal("AFE_RESET", Pins("Y26"), IOStandard("LVCMOS18")),
    Subsignal("AFE_TRST", Pins("B26"), IOStandard("LVCMOS18")),
    Subsignal("AFE_SLEEP", Pins("D26"), IOStandard("LVCMOS18")),
    Subsignal("DAC_SYNC_p",Pins("N21 R22"), IOStandard("LVDS")),
    Subsignal("DAC_SYNC_n",Pins("N22 R23"), IOStandard("LVDS")),
    Subsignal("ADC_SYNC_p", Pins("N19 R20"), IOStandard("LVDS")),
    Subsignal("ADC_SYNC_n", Pins("P19 R21"), IOStandard("LVDS")),

    ),

    ("RX_EN", 0,
     Pins("AD26 AE23 AB19 AC16"), # CHA CHB CHC CHD
     IOStandard("LVCMOS18"),
     ),

    ("TX_EN_VADJ", 0, #VADJ_EN_CH in the sch
     Pins("AC26 AF24 AB20 AB17"), # CHA CHB CHC CHD
     IOStandard("LVCMOS18"),
     ),

    ("TX_EN", 0, #VADJ_EN_CH in the sch
     Pins("AF25 Y18 AC17 AD21"), # CHA CHB CHC CHD
     IOStandard("LVCMOS18"),
     ),

    ("RXA_ATTENUATOR_CTRL", 0,
     Pins("AA14 Y15 AD14 AC14"), # V1 V2 V3 V3
     IOStandard("LVCMOS33")
     ),

    ("RXB_ATTENUATOR_CTRL", 0,
     Pins("W12 Y13 AC13 AA13"), # V1 V2 V3 V3
     IOStandard("LVCMOS33")
     ),

    ("RXC_ATTENUATOR_CTRL", 0,
     Pins("AB14 AA15 AB15 Y16"), # V1 V2 V3 V3
     IOStandard("LVCMOS33")
     ),

    ("RXD_ATTENUATOR_CTRL", 0,
     Pins("AB16 W14 W16 W15"), # V1 V2 V3 V3
     IOStandard("LVCMOS33")
     ),

    ("SW_PA_ONOFF_CTRL", 0,
     Pins("AD24 AF23 AA17 AF22"),  # CHA CHB CHC CHD
     # Is this really right? Switches are powered by 3.3V
     IOStandard("LVCMOS18"),
     ),

    ("SW_RXTX_CTRL", 0,
     Pins("AD25 AC22 Y17 AE21"),  # CHA CHB CHC CHD
     # Is this really right? Switches are powered by 3.3V
     IOStandard("LVCMOS18"),
     ),

    ("SW_RX_TDDFDD", 0,
     Pins("AA22 AC24 AC21 AB21"),  # CHA CHB CHC CHD
     # Is this really right? Switches are powered by 3.3V
     IOStandard("LVCMOS18"),
     ),

    ("afe_refclk", 0,
     Subsignal("clk_p", Pins("M7")),
     Subsignal("clk_n", Pins("M6")),
     ),

    ("afe_jesd_tx", 0,
     Subsignal("p", Pins("AA5")),
     Subsignal("n", Pins("AA4")),
     ),

    ("afe_jesd_tx", 1,
     Subsignal("p", Pins("W5")),
     Subsignal("n", Pins("W4")),
     ),

    ("afe_jesd_rx", 0,
     Subsignal("p", Pins("Y2")),
     Subsignal("n", Pins("Y1")),
     ),

    ("afe_jesd_rx", 1,
     Subsignal("p", Pins("V2")),
     Subsignal("n", Pins("V1")),
     ),

    ("afe_spi", 0,
     Subsignal("clk" , Pins("P25")),                        #AFE_SPIA_CLK
     Subsignal("cs_n", Pins("P24")),                        #AFE_SPIA_SEN
     Subsignal("mosi", Pins("W26"), Misc("PULLDOWN=True")), #AFE_SPIA_SDIO
     Subsignal("miso", Pins("P26"), Misc("PULLDOWN=True")), #AFE_SPIA_SDO

     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
     ),

    ("afe_spi", 1,
     Subsignal("clk", Pins("Y22")),  # AFE_SPIB_CLK
     Subsignal("cs_n", Pins("Y23")), # AFE_SPIB_SEN
     Subsignal("mosi", Pins("V21"), Misc("PULLDOWN=True")),  # AFE_SPIB_SDIO
     Subsignal("miso", Pins("W21"), Misc("PULLDOWN=True")),  # AFE_SPIB_SDO

     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
     ),

    ("afe_spi", 2,
     Subsignal("clk", Pins("V23")),  # AFE_SPIC_CLK
     Subsignal("cs_n", Pins("W23")), # AFE_SPIC_SEN
     Subsignal("mosi", Pins("W20"), Misc("PULLDOWN=True")),  # AFE_SPIC_SDIO
     Subsignal("miso", Pins("W24"), Misc("PULLDOWN=True")),  # AFE_SPIC_SDO

     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
     ),



    ("fpga_sysref", 0,
     Subsignal("p", Pins("T24"), IOStandard("LVDS")),
     Subsignal("n", Pins("U24"), IOStandard("LVDS")),
     ),

    ("dac_sync", 0,
     Subsignal("p", Pins("R22"), IOStandard("LVDS")),
     Subsignal("n", Pins("R23"), IOStandard("LVDS")),
     ),

    ("adc_sync", 0,
     Subsignal("p", Pins("R20"), IOStandard("LVDS")),
     Subsignal("n", Pins("R21"), IOStandard("LVDS")),
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
    ("flash_cs_n", 0, Pins("AA12"), IOStandard("LVCMOS18")),
    ("flash", 0,
        Subsignal("mosi", Pins("AD11")),
        Subsignal("miso", Pins("AC12")),
        Subsignal("wp",   Pins("AC11")),
        Subsignal("hold", Pins("AE11")),
        IOStandard("LVCMOS18")
    ),
    ("spiflash", 0,
        Subsignal("cs_n", Pins("AA12")),
        Subsignal("mosi", Pins("AD11")),
        Subsignal("miso", Pins("AC12")),
        Subsignal("wp",   Pins("AC11")),
        Subsignal("hold", Pins("AE11")),
        IOStandard("LVCMOS18")
    ),

    # I2C buses.
    # SDA1 and SCL1 by schematic
    ("i2c", 0,
        Subsignal("scl", Pins("C9"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("B9"), Misc("PULLUP=True")),
        IOStandard("LVCMOS18"),
    ),
    # SDA2 and SCL2 by schematic
    ("i2c", 1,
        Subsignal("scl", Pins("F13"), Misc("PULLUP=True")),
        Subsignal("sda", Pins("F14"), Misc("PULLUP=True")),
        IOStandard("LVCMOS18"),
    ),
    # SDA3 and SCL3 by schematic
    ("i2c", 2,
     Subsignal("scl", Pins("AD23"), Misc("PULLUP=True")),
     Subsignal("sda", Pins("AC23"), Misc("PULLUP=True")),
     IOStandard("LVCMOS18"),
     ),
    # REF_SDA and REF_SCL by schematic
    ("i2c", 3,
     Subsignal("scl", Pins("AB26"), Misc("PULLUP=True")),
     Subsignal("sda", Pins("AB25"), Misc("PULLUP=True")),
     IOStandard("LVCMOS18"),
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
        Subsignal("tx" , Pins("N2"),  Misc("PULLUP=True")),
        Subsignal("rx" , Pins("L1"),  Misc("PULLUP=True")),
        Subsignal("hw_s",Pins("L18"), IOStandard("LVCMOS33")),
        Subsignal("fix", Pins("R18"), IOStandard("LVCMOS33")),
        IOStandard("LVCMOS33")
    ),

    # VCTCXO.
    ("vctcxo", 0,
        Subsignal("en",  Pins("E10"), Misc("PULLUP=True")),
        Subsignal("sel", Pins("E11"), Misc("PULLDOWN=True")), # ext_clk
        Subsignal("clk", Pins("F9"), Misc("PULLDOWN=True")),
        IOStandard("LVCMOS33")
    ),

    # GPIO (X12, 8-pin FPC connector)
    ("gpio", 0, Pins("F10 G9 G10 G11"), IOStandard("LVCMOS33")),

    ("gpio33", 0, Pins("AF15 AF14"), IOStandard("LVCMOS33")), #GPIO33_2, GPIO33_3 (GPIO33_0 and GPIO33_1 ar used for serial)

    # AUX.
    ("aux", 0,
        Subsignal("en_smsigio", Pins("D17")),
        Subsignal("gpio13",     Pins("T17")),
        IOStandard("LVCMOS33")
    ),

    # RF-Switches / SKY13330, SKY13384.
    ("rf_switches", 0,
        Subsignal("tx", Pins("H9"),    Misc("PULLUP=True")),
        Subsignal("rx", Pins("H11 J9"), Misc("PULLUP=True")),
        IOStandard("LVCMOS33")
    ),


    ("ref_spi", 0,
     Subsignal("cs_n", Pins("J14")),
     Subsignal("clk", Pins("J15")),
     Subsignal("mosi", Pins("G14")),
     # no miso on this spi
     IOStandard("LVCMOS18"),
     ),

    # RF-IC
    ("lms8001_spi", 0,
     # SPI.
     Subsignal("clk", Pins("AA20")),                                #SPI_CLK
     Subsignal("cs_n", Pins("AE26 AA18 AB24 AE22 AA19 AF20")),      #LMS8001B_RX_U1_SEN, LMS8001B_RX_U2_SEN, LMS8001A_RX_U3_SEN, LMS8001A_RX_U4_SEN, LMS8001B_TX_U5_SEN, LMS8001B_TX_U6_SEN
     Subsignal("mosi", Pins("AB22"), Misc("PULLDOWN=True")),        #SPI_SDIO
     Subsignal("miso", Pins("Y20"), Misc("PULLDOWN=True")),         #SPI_SDO

     # IOStandard/Slew Rate.
     IOStandard("LVCMOS18"),
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
        Subsignal("tx", Pins("AD15")), # GPIO33_0
        Subsignal("rx", Pins("AE15")), # GPIO33_1
        IOStandard("LVCMOS33")
    ),


    # Power
    ("pwr_ctrl", 0,
     Subsignal("PWR_EN_LMK",        Pins("E12")),
     Subsignal("PWR_EN_2P05",       Pins("E11")),
     Subsignal("PWR_EN_TCXO",       Pins("F9")),
     Subsignal("PG_EN_2P05",        Pins("E10"), Misc("PULLUP=True")),
     Subsignal("LMK_PDN",           Pins("H12")),
     Subsignal("PAFE_EN_D1P0",      Pins("D9")),
     Subsignal("PAFE_EN_A1P2",      Pins("B10")),
     Subsignal("PAFE_EN_A1P8",      Pins("A9")),
     Subsignal("PAFE_EN_A1P8_1",    Pins("J10")),
     Subsignal("PG_AFE_AVDD_1P2",   Pins("H9")),
     Subsignal("AFE_DCDC_1P0_NRST", Pins("D10")),

     IOStandard("LVCMOS18")
     ),

    ("clk_ctrl", 0,
     Subsignal("clk_src_sel", Pins("w13")),
     IOStandard("LVCMOS33")
     ),

    ("lmk_ctrl", 0,
     Subsignal("lmk_syncn", Pins("H14"), Misc("PULLUP=True")),
     Subsignal("lmk_finc", Pins("H13"), Misc("PULLDOWN=True")),
     Subsignal("lmk_fdet", Pins("J12"), Misc("PULLDOWN=True")),

     IOStandard("LVCMOS18")
     ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxUSPPlatform):
    default_clk_name   = "clk26"
    default_clk_period = 1e9/26e6

    def __init__(self, toolchain="vivado"):
        XilinxUSPPlatform.__init__(self, "xcau15p-ffvb676-2-i", _io, toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.UNUSEDPIN Pulldown [current_design]",
            "set_property CONFIG_MODE SPIx4 [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN Disable [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 63.8 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]",
            "set_property CFGBVS GND [current_design]",
            "set_property CONFIG_VOLTAGE 1.8 [current_design]",
        ]
        # TODO: set multiboot adress as a variable somewhere else instead of hardcoding it here
        self.gold_img_commands = [
            # Multiboot SPI-Flash Golden bitstream generation.
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK ENABLE [current_design]",
            "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 32'h00310000 [current_design]",
            #"set_property BITSTREAM.CONFIG.TIMER_CFG 0x493E0 [current_design]",
            # USR_ACCESS Field at 00007C-00007F binfile For Bitstream identification:
            #[32:24] - DEVICE ID
            #[23:20] - HW_VER
            #[19:16] - Image identifier ( 0 - Gold image, 1- User image)
            #[15: 0] - Reserved
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X1B200000 [current_design]",
            # "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00400000 [current_design]",
            "write_bitstream -force {build_name}_golden.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 32 -loadbit \"up 0x0 {build_name}_golden.bit\" -file ../../../bitstream/{build_name}/{build_name}_golden.bin"
        ]
        self.user_img_commands = [

            # Multiboot SPI-Flash user bitstream generation.
            #"set_property BITSTREAM.CONFIG.TIMER_CFG 0x493E0 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK Enable [current_design]",
            # USR_ACCESS Field at 00007C-00007F binfile For Bitstream identification:
            #[32:24] - DEVICE ID
            #[23:20] - HW_VER
            #[19:16] - Image identifier ( 0 - Gold image, 1- User image)
            #[15: 0] - Reserved
            "set_property BITSTREAM.CONFIG.USR_ACCESS 0X1B210000 [current_design]",
            "write_bitstream -force {build_name}_user.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 32 -loadbit \"up 0x0 {build_name}_user.bit\" -file ../../../bitstream/{build_name}/{build_name}_user.bin",
        ]

        # soc.get_build_name()
        # self.toolchain.
        self.toolchain.additional_commands = [
            # non-multiboot flash images should not be used, so we don't generate them
            # Non-Multiboot SPI-Flash bitstream generation.

            # "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file ../../../bitstream/{build_name}/{build_name}.bin",
        ]

        self.add_source("./gateware/limesdr_hiper_constrs.xdc")

    def create_programmer(self, cable="digilent_hs2"):
        return OpenFPGALoader(cable=cable, fpga_part="xcau15p-ffvb676", freq=20e6)

    def do_finalize(self, fragment):
        XilinxUSPPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk26", loose=True), 1e9/26e6)
