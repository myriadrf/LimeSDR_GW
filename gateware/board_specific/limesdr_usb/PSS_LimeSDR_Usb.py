from gateware.GpioTop import GpioTop
from litei2c import LiteI2C
from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *

# PSS (Peripheral Support Subsystem) -----------------------------------------------------------------

class PSS_LimeSDR_Usb(LiteXModule):
    def __init__(self, soc, platform, sys_clk_freq):

        # SPI1 - TCXO DAC, ADF4002
        # Need to do some trickery here to add a dummy miso
        # as well as to deal with two differently named chip selects
        spi1_phy_pads = platform.request("FPGA_SPI1")
        spi1_pads = Record([
            ("clk",  1),
            ("cs_n", 2),
            ("mosi", 1),
            ("miso", 1),
        ])
        self.comb += [
            spi1_phy_pads.clk.eq(spi1_pads.clk),
            spi1_phy_pads.adf_cs_n.eq(spi1_pads.cs_n[0]),
            spi1_phy_pads.dac_cs_n.eq(spi1_pads.cs_n[1]),
            spi1_phy_pads.mosi.eq(spi1_pads.mosi),
            spi1_pads.miso.eq(Constant(0)),
        ]
        soc.add_spi_master(name="fpga_spi1", pads=spi1_pads, data_width=32, spi_clk_freq=1e6)

        # BRDG SPI - I2C-SPI bridge
        # bridge isn't actually fitted onboard, commenting out spimaster to not waste resources
        # self.add_spi_master(name="brdg_spi", pads=platform.request("BRDG_SPI"), data_width=32, spi_clk_freq=1e6)

        # I2C - SI5351C, Temperature sensor, EEPROM, Port Expander (Bus shared with FX3)
        self.i2c = LiteI2C(sys_clk_freq=sys_clk_freq,pads=platform.request("FPGA_I2C"),clock_domain="sys")

        # GPIO
        self.gpio = GpioTop(platform=platform,pads=platform.request("FPGA_GPIO"))


