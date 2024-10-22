#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

# Layout -------------------------------------------------------------------------------------------

from_periphcfg_layout = [
    ("BOARD_GPIO_OVRD",      16),
    ("BOARD_GPIO_DIR",       16),
    ("BOARD_GPIO_VAL",       16),
    ("PERIPH_OUTPUT_OVRD_0", 16),
    ("PERIPH_OUTPUT_VAL_0",  16),
    ("PERIPH_OUTPUT_OVRD_1", 16),
    ("PERIPH_OUTPUT_VAL_1",  16),
]

to_periphcfg_layout = [
    ("BOARD_GPIO_RD",     16),
    ("PERIPH_INPUT_RD_0", 16),
    ("PERIPH_INPUT_RD_1", 16),
]

def FromPeriphCfg():
    return Record(from_periphcfg_layout)

def ToPeriphCfg():
    return Record(to_periphcfg_layout)

# General Periph Top -------------------------------------------------------------------------------

class GeneralPeriphTop(LiteXModule):
    def __init__(self, platform,
        dev_family = "CYCLONE IV E",
        gpio_pads  = None, gpio_len=8,
        egpio_pads = None, egpio_len=2,
        ):

        self.HW_VER           = Signal(4)

        self.to_periphcfg     = ToPeriphCfg()
        self.from_periphcfg   = FromPeriphCfg()

        self.led1_mico32_busy = Signal()
        self.led1_ctrl        = Signal(3)
        self.led2_ctrl        = Signal(3)
        self.fx3_led_ctrl     = Signal(3)
        self.tx_txant_en      = Signal()

        self.ep03_active      = Signal()
        self.ep83_active      = Signal()

        # # #

        N_GPIO = gpio_len + egpio_len

        # Signals.
        # --------
        gpio_out_val = Signal(N_GPIO)

        self.comb += [
            gpio_out_val[8:10].eq(0),
            gpio_out_val[7].eq(~self.ep03_active),
            gpio_out_val[6].eq(1),
            gpio_out_val[5].eq(1),
            gpio_out_val[4].eq(~self.ep83_active),
            gpio_out_val[1:4].eq(0),
            gpio_out_val[0].eq(self.tx_txant_en),
        ]

        # General_periph_top wrapper (required due to record).
        # ----------------------------------------------------
        self.specials += Instance("general_periph_top_wrapper",
            # Parameters
            #p_DEV_FAMILY           = dev_family,
            p_N_GPIO               = N_GPIO,

            # General ports
            i_clk                  = ClockSignal("sys"), # Free running clock
            i_reset_n              = ~ResetSignal("sys"),

            # to_periphcfg
            o_BOARD_GPIO_RD        = self.to_periphcfg.BOARD_GPIO_RD,
            o_PERIPH_INPUT_RD_0    = self.to_periphcfg.PERIPH_INPUT_RD_0,
            o_PERIPH_INPUT_RD_1    = self.to_periphcfg.PERIPH_INPUT_RD_1,
            # from_periphcfg
            i_BOARD_GPIO_OVRD      = self.from_periphcfg.BOARD_GPIO_OVRD,
            i_BOARD_GPIO_DIR       = self.from_periphcfg.BOARD_GPIO_DIR,
            i_BOARD_GPIO_VAL       = self.from_periphcfg.BOARD_GPIO_VAL,
            i_PERIPH_OUTPUT_OVRD_0 = self.from_periphcfg.PERIPH_OUTPUT_OVRD_0,
            i_PERIPH_OUTPUT_VAL_0  = self.from_periphcfg.PERIPH_OUTPUT_VAL_0,
            i_PERIPH_OUTPUT_OVRD_1 = self.from_periphcfg.PERIPH_OUTPUT_OVRD_1,
            i_PERIPH_OUTPUT_VAL_1  = self.from_periphcfg.PERIPH_OUTPUT_VAL_1,

            # Dual colour LEDs
            # LED1 (Clock and PLL lock status)
            i_led1_mico32_busy      = self.led1_mico32_busy,
            i_led1_ctrl             = self.led1_ctrl,
            o_led1_g                = platform.request("FPGA_LED1_G"),
            o_led1_r                = platform.request("FPGA_LED1_R"),

            # LED2 (TCXO control status)
            i_led2_clk             = Constant(0, 1),
            i_led2_adf_muxout      = Constant(0, 1),
            i_led2_dac_ss          = Constant(0, 1),
            i_led2_adf_ss          = Constant(0, 1),
            i_led2_ctrl            = self.led2_ctrl,
            o_led2_g               = Open(),
            o_led2_r               = Open(),

            # LED3 (FX3 and NIOS CPU busy)
            i_led3_g_in            = Constant(0, 1),
            i_led3_r_in            = Constant(0, 1),
            i_led3_ctrl            = self.fx3_led_ctrl,
            i_led3_hw_ver          = self.HW_VER,
            o_led3_g               = Open(),
            o_led3_r               = Open(),

            # GPIO
            i_gpio_dir             = Replicate(1, N_GPIO),
            i_gpio_out_val         = gpio_out_val,
            o_gpio_rd_val          = Open(N_GPIO),
            io_gpio                = gpio_pads,
            io_egpio               = egpio_pads if egpio_pads is not None else Open(egpio_len),

            # Fan control
            i_fan_sens_in          = platform.request("LM75_OS"),
            o_fan_ctrl_out         = platform.request("FAN_CTRL"),
        )

        self.add_sources(platform)

    def add_sources(self, platform):
        general_periph_files = [
            "LimeSDR-Mini_lms7_trx/src/general_periph/synth/general_periph_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/alive.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/FPGA_LED_cntrl.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/FPGA_LED2_ctrl.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/FX3_LED_ctrl.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/gpio_ctrl_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/gpio_ctrl.vhd",
            "LimeSDR-Mini_lms7_trx/src/general/general_pkg.vhd",
            "gateware/general_periph_top_wrapper.vhd",
        ]

        for file in general_periph_files:
            platform.add_source(file)
