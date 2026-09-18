#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *

# CA23 RFSW ----------------------------------------------------------------------------------------

class ca23_rfsw(LiteXModule):
    def __init__(self, platform, mipi_pads, tdd_pad):
        # TDD Control CSRs (register 0x0A)
        self.tdd_manual_val = CSRStorage(1, reset=0,
            description="TDD Signal manual control value"
        )
        self.tdd_auto_en = CSRStorage(1, reset=0,
            description="0- TDD auto control disabled, 1- TDD auto control enabled"
        )
        self.tdd_invert = CSRStorage(1, reset=0,
            description="0- TDD Control signal not inverted, 1- TDD Control signal inverted"
        )
        self.rfsw_auto_en = CSRStorage(1, reset=0,
            description="0- RFSW Auto control disabled, 1- RFSW Auto control enabled"
        )

        # Add HDL source files
        platform.add_source("./gateware/tdd_control.vhd")
        platform.add_source("./gateware/LimeDFB/mipi/mipi_rffe_master_phy.vhd")
        platform.add_source("./gateware/LimeDFB/mipi/mipi_rffe_updater.vhd")

        # TDD control signals
        self.AUTO_IN = Signal()
        self.TDD_OUT = Signal()

        # Connect TDD control module
        self.specials += Instance("tdd_control",
            i_MANUAL_VALUE       = self.tdd_manual_val.storage,
            i_AUTO_ENABLE        = self.tdd_auto_en.storage,
            i_AUTO_IN            = self.AUTO_IN,
            i_AUTO_INVERT        = self.tdd_invert.storage,
            i_RX_RF_SW_IN        = Constant(0, 2),
            i_TX_RF_SW_IN        = Constant(0, 1),
            i_RF_SW_AUTO_ENANBLE = self.rfsw_auto_en.storage,
            o_TDD_OUT            = tdd_pad,
            o_RX_RF_SW_OUT       = Signal(2),
            o_TX_RF_SW_OUT       = Signal(1)
        )

        # 6 MIPI RFFE switch signals
        self.rx1_data_in      = Signal(8)
        self.rx1_data_out     = Signal(8)
        self.rx1_interface_ok = Signal(2)
        self.rx1_test_done    = Signal()

        self.trx1_data_in      = Signal(8)
        self.trx1_data_out     = Signal(8)
        self.trx1_interface_ok = Signal(2)
        self.trx1_test_done    = Signal()

        self.trx1_ant_data_in      = Signal(8)
        self.trx1_ant_data_out     = Signal(8)
        self.trx1_ant_interface_ok = Signal(2)
        self.trx1_ant_test_done    = Signal()

        self.rx2_data_in      = Signal(8)
        self.rx2_data_out     = Signal(8)
        self.rx2_interface_ok = Signal(2)
        self.rx2_test_done    = Signal()

        self.trx2_data_in      = Signal(8)
        self.trx2_data_out     = Signal(8)
        self.trx2_interface_ok = Signal(2)
        self.trx2_test_done    = Signal()

        self.trx2_ant_data_in      = Signal(8)
        self.trx2_ant_data_out     = Signal(8)
        self.trx2_ant_interface_ok = Signal(2)
        self.trx2_ant_test_done    = Signal()

        # Instantiate 6 MIPI RFFE updaters
        # 1. RX1_RF
        self.specials += Instance("mipi_rffe_updater",
            p_G_SLAVE_ADDR               = Instance.PreformattedParam("4'b1010"),
            p_G_TEST_ADDR                = Instance.PreformattedParam("5'b11110"),
            p_G_TEST_VAL                 = Instance.PreformattedParam("8'b10100101"),
            p_G_TARGET_ADDR              = Instance.PreformattedParam("5'b00000"),
            p_G_DISABLE_TRIGGERS         = Instance.PreformattedParam("3'b001"),
            p_G_TRIGGER                  = Instance.PreformattedParam("3'b000"),
            p_G_STATUS_REG_READBACK_ADDR = Instance.PreformattedParam("8'b00100100"),
            p_G_STATUS_READBACK_ENABLED  = Instance.PreformattedParam("1'b1"),

            i_CLK          = ClockSignal("sys"),
            i_RESET_N      = ~ResetSignal("sys"),
            o_INTERFACE_OK = self.rx1_interface_ok,
            o_TEST_DONE    = self.rx1_test_done,
            o_STATUS_REG   = Signal(8),
            i_DATA_IN      = self.rx1_data_in,
            o_DATA_OUT     = self.rx1_data_out,
            o_SCLK         = mipi_pads["rx1_rf"].sclk,
            io_SDATA       = mipi_pads["rx1_rf"].sdata,
            o_DEBUG_SDATA  = Signal()
        )

        # 2. TRX1_RF
        self.specials += Instance("mipi_rffe_updater",
            p_G_SLAVE_ADDR               = Instance.PreformattedParam("4'b1010"),
            p_G_TEST_ADDR                = Instance.PreformattedParam("5'b11110"),
            p_G_TEST_VAL                 = Instance.PreformattedParam("8'b10100101"),
            p_G_TARGET_ADDR              = Instance.PreformattedParam("5'b00000"),
            p_G_DISABLE_TRIGGERS         = Instance.PreformattedParam("3'b001"),
            p_G_TRIGGER                  = Instance.PreformattedParam("3'b000"),
            p_G_STATUS_REG_READBACK_ADDR = Instance.PreformattedParam("8'b00100100"),
            p_G_STATUS_READBACK_ENABLED  = Instance.PreformattedParam("1'b1"),

            i_CLK          = ClockSignal("sys"),
            i_RESET_N      = ~ResetSignal("sys"),
            o_INTERFACE_OK = self.trx1_interface_ok,
            o_TEST_DONE    = self.trx1_test_done,
            o_STATUS_REG   = Signal(8),
            i_DATA_IN      = self.trx1_data_in,
            o_DATA_OUT     = self.trx1_data_out,
            o_SCLK         = mipi_pads["trx1_rf"].sclk,
            io_SDATA       = mipi_pads["trx1_rf"].sdata,
            o_DEBUG_SDATA  = Signal()
        )

        # 3. TRX1_ANT
        self.specials += Instance("mipi_rffe_updater",
            p_G_SLAVE_ADDR               = Instance.PreformattedParam("4'b1010"),
            p_G_TEST_ADDR                = Instance.PreformattedParam("5'b11110"),
            p_G_TEST_VAL                 = Instance.PreformattedParam("8'b10100101"),
            p_G_TARGET_ADDR              = Instance.PreformattedParam("5'b00000"),
            p_G_DISABLE_TRIGGERS         = Instance.PreformattedParam("3'b001"),
            p_G_TRIGGER                  = Instance.PreformattedParam("3'b000"),
            p_G_STATUS_REG_READBACK_ADDR = Instance.PreformattedParam("8'b00000000"),
            p_G_STATUS_READBACK_ENABLED  = Instance.PreformattedParam("1'b0"),

            i_CLK          = ClockSignal("sys"),
            i_RESET_N      = ~ResetSignal("sys"),
            o_INTERFACE_OK = self.trx1_ant_interface_ok,
            o_TEST_DONE    = self.trx1_ant_test_done,
            o_STATUS_REG   = Signal(8),
            i_DATA_IN      = self.trx1_ant_data_in,
            o_DATA_OUT     = self.trx1_ant_data_out,
            o_SCLK         = mipi_pads["trx1_ant"].sclk,
            io_SDATA       = mipi_pads["trx1_ant"].sdata,
            o_DEBUG_SDATA  = Signal()
        )

        # 4. RX2_RF
        self.specials += Instance("mipi_rffe_updater",
            p_G_SLAVE_ADDR               = Instance.PreformattedParam("4'b1010"),
            p_G_TEST_ADDR                = Instance.PreformattedParam("5'b11110"),
            p_G_TEST_VAL                 = Instance.PreformattedParam("8'b10100101"),
            p_G_TARGET_ADDR              = Instance.PreformattedParam("5'b00000"),
            p_G_DISABLE_TRIGGERS         = Instance.PreformattedParam("3'b001"),
            p_G_TRIGGER                  = Instance.PreformattedParam("3'b000"),
            p_G_STATUS_REG_READBACK_ADDR = Instance.PreformattedParam("8'b00000000"),
            p_G_STATUS_READBACK_ENABLED  = Instance.PreformattedParam("1'b0"),

            i_CLK          = ClockSignal("sys"),
            i_RESET_N      = ~ResetSignal("sys"),
            o_INTERFACE_OK = self.rx2_interface_ok,
            o_TEST_DONE    = self.rx2_test_done,
            o_STATUS_REG   = Signal(8),
            i_DATA_IN      = self.rx2_data_in,
            o_DATA_OUT     = self.rx2_data_out,
            o_SCLK         = mipi_pads["rx2_rf"].sclk,
            io_SDATA       = mipi_pads["rx2_rf"].sdata,
            o_DEBUG_SDATA  = Signal()
        )

        # 5. TRX2_RF
        self.specials += Instance("mipi_rffe_updater",
            p_G_SLAVE_ADDR               = Instance.PreformattedParam("4'b1010"),
            p_G_TEST_ADDR                = Instance.PreformattedParam("5'b11110"),
            p_G_TEST_VAL                 = Instance.PreformattedParam("8'b10100101"),
            p_G_TARGET_ADDR              = Instance.PreformattedParam("5'b00000"),
            p_G_DISABLE_TRIGGERS         = Instance.PreformattedParam("3'b001"),
            p_G_TRIGGER                  = Instance.PreformattedParam("3'b000"),
            p_G_STATUS_REG_READBACK_ADDR = Instance.PreformattedParam("8'b00000000"),
            p_G_STATUS_READBACK_ENABLED  = Instance.PreformattedParam("1'b0"),

            i_CLK          = ClockSignal("sys"),
            i_RESET_N      = ~ResetSignal("sys"),
            o_INTERFACE_OK = self.trx2_interface_ok,
            o_TEST_DONE    = self.trx2_test_done,
            o_STATUS_REG   = Signal(8),
            i_DATA_IN      = self.trx2_data_in,
            o_DATA_OUT     = self.trx2_data_out,
            o_SCLK         = mipi_pads["trx2_rf"].sclk,
            io_SDATA       = mipi_pads["trx2_rf"].sdata,
            o_DEBUG_SDATA  = Signal()
        )

        # 6. TRX2_ANT
        self.specials += Instance("mipi_rffe_updater",
            p_G_SLAVE_ADDR               = Instance.PreformattedParam("4'b1010"),
            p_G_TEST_ADDR                = Instance.PreformattedParam("5'b11110"),
            p_G_TEST_VAL                 = Instance.PreformattedParam("8'b10100101"),
            p_G_TARGET_ADDR              = Instance.PreformattedParam("5'b00000"),
            p_G_DISABLE_TRIGGERS         = Instance.PreformattedParam("3'b001"),
            p_G_TRIGGER                  = Instance.PreformattedParam("3'b000"),
            p_G_STATUS_REG_READBACK_ADDR = Instance.PreformattedParam("8'b00000000"),
            p_G_STATUS_READBACK_ENABLED  = Instance.PreformattedParam("1'b0"),

            i_CLK          = ClockSignal("sys"),
            i_RESET_N      = ~ResetSignal("sys"),
            o_INTERFACE_OK = self.trx2_ant_interface_ok,
            o_TEST_DONE    = self.trx2_ant_test_done,
            o_STATUS_REG   = Signal(8),
            i_DATA_IN      = self.trx2_ant_data_in,
            o_DATA_OUT     = self.trx2_ant_data_out,
            o_SCLK         = mipi_pads["trx2_ant"].sclk,
            io_SDATA       = mipi_pads["trx2_ant"].sdata,
            o_DEBUG_SDATA  = Signal()
        )
