#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# Layout -------------------------------------------------------------------------------------------

from_periphcfg_layout = [
    ("board_gpio_ovrd",      16),
    ("board_gpio_dir",       16),
    ("board_gpio_val",       16),
    ("periph_output_ovrd_0", 16),
    ("periph_output_val_0",  16),
    ("periph_output_ovrd_1", 16),
    ("periph_output_val_1",  16),
]

to_periphcfg_layout = [
    ("board_gpio_rd",     16),
    ("periph_input_rd_0", 16),
    ("periph_input_rd_1", 16),
]

def FromPeriphCfg():
    return Record(from_periphcfg_layout)

def ToPeriphCfg():
    return Record(to_periphcfg_layout)

# General Periph Top -------------------------------------------------------------------------------

class GeneralPeriphTop(LiteXModule):
    def __init__(self, platform,
        dev_family = "CYCLONE IV E",
        revision_pads = None,
        gpio_pads     = None, gpio_len=8,
        egpio_pads    = None, egpio_len=2,
        add_csr       = True,
        ):

        self.LMS_TXNRX2_or_CLK_SEL = Signal(1)

        # to_periphcfg
        self.board_gpio_rd         = Signal(16)
        self.periph_input_rd_0     = Signal(16)
        self.periph_input_rd_1     = Signal(16)
        # from_periphcfg
        self.board_gpio_ovrd       = Signal(16)
        self.board_gpio_dir        = Signal(16)
        self.board_gpio_val        = Signal(16)
        self.periph_output_ovrd_0  = Signal(16)
        self.periph_output_val_0   = Signal(16)
        self.periph_output_ovrd_1  = Signal(16)
        self.periph_output_val_1   = Signal(16)

        self.led1_mico32_busy      = Signal()
        self.led1_ctrl             = Signal(3)
        self.led2_ctrl             = Signal(3)
        self.fx3_led_ctrl          = Signal(3)
        self.tx_txant_en           = Signal()

        self.ep03_active           = Signal()
        self.ep83_active           = Signal()

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
            o_BOARD_GPIO_RD        = self.board_gpio_rd,
            o_PERIPH_INPUT_RD_0    = self.periph_input_rd_0,
            o_PERIPH_INPUT_RD_1    = self.periph_input_rd_1,
            # from_periphcfg
            i_BOARD_GPIO_OVRD      = self.board_gpio_ovrd,
            i_BOARD_GPIO_DIR       = self.board_gpio_dir,
            i_BOARD_GPIO_VAL       = self.board_gpio_val,
            i_PERIPH_OUTPUT_OVRD_0 = self.periph_output_ovrd_0,
            i_PERIPH_OUTPUT_VAL_0  = self.periph_output_val_0,
            i_PERIPH_OUTPUT_OVRD_1 = self.periph_output_ovrd_1,
            i_PERIPH_OUTPUT_VAL_1  = self.periph_output_val_1,

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
            i_led3_hw_ver          = revision_pads.HW_VER,
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

        if add_csr:
            self.add_csr()

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

    def add_csr(self):
        self._board_gpio_OVRD      = CSRStorage(16, reset=0xf) # 0
        self._board_gpio_RD        = CSRStatus(16)             # 2
        self._board_gpio_DIR       = CSRStorage(16)            # 4
        self._board_gpio_VAL       = CSRStorage(16)            # 6
        self._periph_input_RD_0    = CSRStatus(16)             # 8
        self._periph_input_RD_1    = CSRStatus(16)             # 9
        self._periph_output_OVRD_0 = CSRStorage(16)            # 12
        self._periph_output_VAL_0  = CSRStorage(16)            # 13
        self._periph_output_OVRD_1 = CSRStorage(16)            # 14
        self._periph_output_VAL_1  = CSRStorage(16)            # 15

        # from fpgacfg
        self.fpga_led_ctrl         = CSRStorage(fields=[       # fpgacfg.26
            CSRField("LED1_CTRL", size=3, offset=0),
            CSRField("LED2_CTRL", size=3, offset=4),
        ])
        self._FX3_LED_CTRL         = CSRStorage(3)             # fpgacfg.28

        self.comb += [
            self._board_gpio_RD.status.eq(    self.board_gpio_rd),
            self._periph_input_RD_0.status.eq(self.periph_input_rd_0),
            self._periph_input_RD_1.status.eq(self.periph_input_rd_1),

            self.board_gpio_ovrd.eq(          self._board_gpio_OVRD.storage),
            self.board_gpio_dir.eq(           self._board_gpio_DIR.storage),
            self.board_gpio_val.eq(           self._board_gpio_VAL.storage),
            self.periph_output_ovrd_0.eq(     self._periph_output_OVRD_0.storage),
            self.periph_output_val_0.eq(      self._periph_output_VAL_0.storage),
            self.periph_output_ovrd_1.eq(     self._periph_output_OVRD_1.storage),
            self.periph_output_val_1.eq(      self._periph_output_VAL_1.storage),

            self.led1_ctrl.eq(                self.fpga_led_ctrl.fields.LED1_CTRL),
            self.led2_ctrl.eq(                self.fpga_led_ctrl.fields.LED2_CTRL),
            self.fx3_led_ctrl.eq(             self._FX3_LED_CTRL.storage),
        ]

