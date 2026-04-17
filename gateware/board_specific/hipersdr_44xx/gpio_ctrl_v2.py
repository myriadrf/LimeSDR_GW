#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0
#
# CSR register wrapper for gpio control for hipersdr_44xx board

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.LimeDFB.misc.TDDController import TDDController


class GPIO_Ctrl_v2(LiteXModule):
    def __init__(self, platform,
                 tdd_control_clk_domain="sys"):
        """
        Wrapper for CSR registers holding the values used to control the GPIOs.
        Created for hipersdr_44xx board, where GPIOs are controlled by a set of
        TCA6424 I/O expanders, as well as GPIOs controlled directly by the FPGA.
        Expanders are assumed to be in this order:
        1. U114 - I2C2 (I2C3 in schematic)
        2. U113 - I2C2 (I2C3 in schematic)
        3. U115 I2C3 (REF_I2C/I2C5 in schematic)
        Max size of a CSR register is 32 bits - not enough to store values for all three
        I/O expanders, so a separate CSR register is provided for each expander
        Each IO expander is subdivided into 3 ports of 8 bits each.

        (fw offset N) indicates that values in the table are addressed in the firmware as number + N

        Pin assignments:
        ---------------------------------------------------------------------------------------------------------------
        | U114 (fw offset 0)         | U113 (fw offset 24)   | U115 (fw offset 48)        | U110 (fw offset 72)      ||
        ---------------------------------------------------------------------------------------------------------------
        -- PORT 0                    |                       |                            |                ------------
        ---------------------------------------------------------------------------------------------------------------
        0: SW_RX_FILBANK_OUT_V2_CHD  | 0: VSD_LNA_IF_CHA_RX  | 0: ENABLE_6VIN             | 0: SW_IN_RX_V2_CHA
        1: SW_RX_FILBANK_OUT_V1_CHD  | 1: VBYP_LNA_IF_CHA_RX | 1: ENABLE_7.5VIN           | 1: SW_IN_RX_V1_CHA
        2: SW_RX_FILBANK_OUT_V1_CHC  | 2: VSD_LNA_IF_CHB_RX  | 2: REF_EN_GPS              | 2: SW_IN_RX_V2_CHB
        3: SW_RX_FILBANK_OUT_V2_CHC  | 3: VBYP_LNA_IF_CHB_RX | 3: REF_EN_OSC              | 3: SW_IN_RX_V1_CHB
        4: SW_RX_FILBANK_OUT_V2_CHB  | 4: VSD_LNA_IF_CHC_RX  | 4: PG_8PO                  | 4: SW_IN_RX_V2_CHC
        5: SW_RX_FILBANK_OUT_V1_CHB  | 5: VBYP_LNA_IF_CHC_RX | 5: PG_6PO                  | 5: SW_IN_RX_V1_CHC
        6: SW_RX_FILBANK_OUT_V2_CHA  | 6: VSD_LNA_IF_CHD_RX  | 6: ENABLE_5VIN_EXTLO       | 6: SW_IN_RX_V2_CHD
        7: SW_RX_FILBANK_OUT_V1_CHA  | 7: VBYP_LNA_IF_CHD_RX | 7: ENABLE_5VIN             | 7: SW_IN_RX_V1_CHD
        ---------------------------------------------------------------------------------------------------------------
        -- PORT 1                    |                       |                            |                ------------
        ---------------------------------------------------------------------------------------------------------------
        8:  SW_RX_FILBANK_IN_V3_CHA  | 8:  SW_OUT_TX_CHD      | 8:  PWR_LMS8_NRST          | 8:  LCHA_G
        9:  SW_RX_FILBANK_IN_V3_CHB  | 9:  SW_OUT_TX_CHC      | 9:  LMS8001B_TX_U6_RESETN  | 9:  LCHA_R
        10: SW_RX_FILBANK_IN_V3_CHC  | 10: SW_OUT_TX_CHB      | 10: LMS8001B_TX_U5_RESETN  | 10: LCHB_G
        11: SW_RX_FILBANK_IN_V3_CHD  | 11: SW_OUT_TX_CHA      | 11: LMS8001A_RX_U3_RESETN  | 11: LCHB_R
        12: SW_RX_FILBANK_OUT_V3_CHD | 12: SW_OUT_RX_V1_CHC   | 12: LMS8001B_RX_U1_RESETN  | 12: LCHC_G
        13: SW_RX_FILBANK_OUT_V3_CHC | 13: SW_OUT_RX_V2_CHC   | 13: LMS8001A_RX_U4_RESETN  | 13: LCHC_R
        14: SW_RX_FILBANK_OUT_V3_CHB | 14: SW_OUT_RX_V1_CHD   | 14: LMS8001A_RX_U2_RESETN  | 14: LCHD_G
        15: SW_RX_FILBANK_OUT_V3_CHA | 15: SW_OUT_RX_V2_CHD   | 15: NOT USED               | 15: LCHD_R
        --------------------------------------------------------------------------------- ----------------------------
        -- PORT 2                    |                        |                            |              ------------
        --------------------------------------------------------------------------------- ----------------------------
        16: SW_RX_FILBANK_IN_V1_CHD  | 16: SW_OUT_RX_V1_CHB   | 16: PA_BYPASS_CTRL_D       | 16: NOT USED
        17: SW_RX_FILBANK_IN_V2_CHD  | 17: SW_OUT_RX_V2_CHB   | 17: PA_BYPASS_CTRL_C       | 17: NOT USED
        18: SW_RX_FILBANK_IN_V1_CHC  | 18: SW_OUT_RX_V1_CHA   | 18: PA_BYPASS_CTRL_B       | 18: NOT USED
        19: SW_RX_FILBANK_IN_V2_CHC  | 19: SW_OUT_RX_V2_CHA   | 19: PA_BYPASS_CTRL_A       | 19: NOT USED
        20: SW_RX_FILBANK_IN_V1_CHB  | 20: SW_IN_TX_CHA       | 20: SW_50R_LNAtoPA_CHA     | 20: LRCHA
        21: SW_RX_FILBANK_IN_V2_CHB  | 21: SW_IN_TX_CHB       | 21: SW_50R_LNAtoPA_CHB     | 21: LRCHB
        22: SW_RX_FILBANK_IN_V1_CHA  | 22: SW_IN_TX_CHC       | 22: SW_50R_LNAtoPA_CHC     | 22: LRCHC
        23: SW_RX_FILBANK_IN_V2_CHA  | 23: SW_IN_TX_CHD       | 23: SW_50R_LNAtoPA_CHD     | 23: LRCHD

        ---------------------------------------------------------------------------------
        -- Non-Expander GPIO mappings -self.GPIO (fw offset 96)                    ------
        ---------------------------------------------------------------------------------
        0:  EN_CHA
        1:  EN_CHB
        2:  EN_CHC
        3:  EN_CHD
        4:  ATT_RX_V1_CHA
        5:  ATT_RX_V2_CHA
        6:  ATT_RX_V3_CHA
        7:  ATT_RX_V4_CHA
        8:  ATT_RX_V1_CHB
        9:  ATT_RX_V2_CHB
        10: ATT_RX_V3_CHB
        11: ATT_RX_V4_CHB
        12: ATT_RX_V1_CHC
        13: ATT_RX_V2_CHC
        14: ATT_RX_V3_CHC
        15: ATT_RX_V4_CHC
        16: ATT_RX_V1_CHD
        17: ATT_RX_V2_CHD
        18: ATT_RX_V3_CHD
        19: ATT_RX_V4_CHD
        20: SW_RX_LB_TDD_FDD_CHA_V1
        21: SW_RX_LB_TDD_FDD_CHA_V2
        22: SW_RX_LB_TDD_FDD_CHB_V1
        23: SW_RX_LB_TDD_FDD_CHB_V2
        24: SW_RX_LB_TDD_FDD_CHC_V1
        25: SW_RX_LB_TDD_FDD_CHC_V2
        26: SW_RX_LB_TDD_FDD_CHD_V1
        27: SW_RX_LB_TDD_FDD_CHD_V2
        28: SW_RXTX_CHA
        29: SW_RXTX_CHB
        30: SW_RXTX_CHC
        31: SW_RXTX_CHD
        ---------------------------------------------------------------------------------
        -- Non-Expander GPIO mappings, extra set - self.GPIO2 (fw offset 128)      ------
        ---------------------------------------------------------------------------------
        0:  SW_TX_ONOFF_CHA
        1:  SW_TX_ONOFF_CHB
        2:  SW_TX_ONOFF_CHC
        3:  SW_TX_ONOFF_CHD
        4:  VADJ_EN_CHA
        5:  VADJ_EN_CHB
        6:  VADJ_EN_CHC
        7:  VADJ_EN_CHD
        8:  PA_EN_CHA
        9:  PA_EN_CHB
        10: PA_EN_CHC
        11: PA_EN_CHD
        12: REFCTRL_SEL
        """

        self.port_direction_u114 = CSRStorage(24,description="GPIO direction: 1: Input, 0: Output")
        self.port_direction_u113 = CSRStorage(24,description="GPIO direction: 1: Input, 0: Output")
        self.port_direction_u115 = CSRStorage(24,description="GPIO direction: 1: Input, 0: Output")
        self.port_direction_u110 = CSRStorage(24, description="GPIO direction: 1: Input, 0: Output")  # in V2 Hardware
        self.polarity_inversion_u114 = CSRStorage(24,description="GPIO polarity: 1: Inverted, 0: Normal")
        self.polarity_inversion_u113 = CSRStorage(24,description="GPIO polarity: 1: Inverted, 0: Normal")
        self.polarity_inversion_u115 = CSRStorage(24,description="GPIO polarity: 1: Inverted, 0: Normal")
        self.polarity_inversion_u110 = CSRStorage(24, description="GPIO polarity: 1: Inverted, 0: Normal") # in V2 Hardware
        self.port_out_value_114 = CSRStorage(24,description="GPIO output value: 1: High, 0: Low")
        self.port_out_value_113 = CSRStorage(24,description="GPIO output value: 1: High, 0: Low")
        self.port_out_value_115 = CSRStorage(24,description="GPIO output value: 1: High, 0: Low")
        self.port_out_value_110 = CSRStorage(24, description="GPIO output value: 1: High, 0: Low") # in V2 Hardware
        self.port_in_value_114 = CSRStatus(24,description="GPIO input value: 1: High, 0: Low")
        self.port_in_value_113 = CSRStatus(24,description="GPIO input value: 1: High, 0: Low")
        self.port_in_value_115 = CSRStatus(24,description="GPIO input value: 1: High, 0: Low")
        self.port_in_value_110 = CSRStatus(24, description="GPIO input value: 1: High, 0: Low") # in V2 Hardware

        # Non-expander, direct control GPIOs
        # (fw offset 96)
        self.GPIO = CSRStorage(32,description="GPIO output value: 1: High, 0: Low")
        
        # (fw offset 128)
        self.GPIO2 = CSRStorage(13, description="GPIO output value: 1: High, 0: Low")

        self.TDDSignal = Signal(1)
        self.TDD_TXANT_PRE = CSRStorage(16, description="Number of cycles to delay enabling TDD signal")
        self.TDD_TXANT_POST = CSRStorage(16, description="Number of cycles to delay disabling TDD signal")
        self.TDDControlEnable = CSRStorage(4, description="TDD Control Enable, bit per channel")
        self.TDDSignalInvert = CSRStorage(4, description="TDD Signal Invert, bit per channel")
        channels = ["A", "B", "C", "D"]
        self.SW_RXTX = Signal(4)
        self.SW_TX_ONOFF = Signal(4)
        for i, char in enumerate(channels):
            # RX_TX switch control ------
            tdd_instance = TDDController(
                width=1,
                tdd_on_setting=0b0,
                tdd_off_setting=0b1,
                delay_clk_domain=tdd_control_clk_domain
            )
            self.comb += [
                #copy-paste start
                tdd_instance.InvertTDDSignal.eq(self.TDDSignalInvert.storage[i]),
                tdd_instance.txant_pre.eq(self.TDD_TXANT_PRE.storage),
                tdd_instance.txant_post.eq(self.TDD_TXANT_POST.storage),
                tdd_instance.TDDSignal.eq(self.TDDSignal),
                tdd_instance.ControlEnable.eq(self.TDDControlEnable.storage[i]),
                #copy-paste end
                tdd_instance.PassthroughSignal.eq(self.GPIO.storage[28+i]),
                self.SW_RXTX[i].eq(tdd_instance.OutputSignal)
            ]
            setattr(self.submodules, f"tdd_controller_RX_TX_{char}", tdd_instance)
            # TX_ONOFF switch control ---
            tdd_instance = TDDController(
                width=1,
                tdd_on_setting=0b0,
                tdd_off_setting=0b1,
                delay_clk_domain=tdd_control_clk_domain
            )
            self.comb += [
                #copy-paste start
                tdd_instance.InvertTDDSignal.eq(self.TDDSignalInvert.storage[i]),
                tdd_instance.txant_pre.eq(self.TDD_TXANT_PRE.storage),
                tdd_instance.txant_post.eq(self.TDD_TXANT_POST.storage),
                tdd_instance.TDDSignal.eq(self.TDDSignal),
                tdd_instance.ControlEnable.eq(self.TDDControlEnable.storage[i]),
                #copy-paste end
                tdd_instance.PassthroughSignal.eq(self.GPIO2.storage[i]),
                self.SW_TX_ONOFF[i].eq(tdd_instance.OutputSignal)
            ]
            setattr(self.submodules, f"tdd_controller_SW_TX_ONOFF_{char}", tdd_instance)

        # Register control CSR:
        # register values:
        # 1 - start operation/operation in progress, 0 - idle
        # operation description:
        # write - write settings from the appropriate register to the I/O expander
        # read  - read settings from the I/O expander and store them in the appropriate register
        self.register_control = CSRStorage(21, description= "register control bitfield", fields=[

            CSRField("u114_direction_write", size=1, offset=0, reset=0),
            CSRField("u113_direction_write", size=1, offset=1, reset=0),
            CSRField("u115_direction_write", size=1, offset=2, reset=0),
            CSRField("u110_direction_write", size=1, offset=3, reset=0),

            CSRField("u114_polarity_write", size=1, offset=4, reset=0),
            CSRField("u113_polarity_write", size=1, offset=5, reset=0),
            CSRField("u115_polarity_write", size=1, offset=6, reset=0),
            CSRField("u110_polarity_write", size=1, offset=7, reset=0),

            CSRField("u114_out_value_write", size=1, offset=8, reset=0),
            CSRField("u113_out_value_write", size=1, offset=9, reset=0),
            CSRField("u115_out_value_write", size=1, offset=10, reset=0),
            CSRField("u110_out_value_write", size=1, offset=11, reset=0),

            CSRField("u114_direction_read", size=1, offset=12, reset=0),
            CSRField("u113_direction_read", size=1, offset=13, reset=0),
            CSRField("u115_direction_read", size=1, offset=14, reset=0),
            CSRField("u110_direction_read", size=1, offset=15, reset=0),

            CSRField("u114_polarity_read", size=1, offset=16, reset=0),
            CSRField("u113_polarity_read", size=1, offset=17, reset=0),
            CSRField("u115_polarity_read", size=1, offset=18, reset=0),
            CSRField("u110_polarity_read", size=1, offset=19, reset=0),

            CSRField("u114_out_value_read", size=1, offset=20, reset=0),
            CSRField("u113_out_value_read", size=1, offset=21, reset=0),
            CSRField("u115_out_value_read", size=1, offset=22, reset=0),
            CSRField("u110_out_value_read", size=1, offset=23, reset=0),

            CSRField("u114_in_value_read", size=1, offset=24, reset=0),
            CSRField("u113_in_value_read", size=1, offset=25, reset=0),
            CSRField("u115_in_value_read", size=1, offset=26, reset=0),
            CSRField("u110_in_value_read", size=1, offset=27, reset=0),
        ])
        # TODO: REGISTER CONTROL DOES NOT DO ANYTHING AT THE TIME
        #       ADD FIRMWARE SUPPORT AND POLLING/IRQ
        #       FOR NOW THESE CSR's ARE MOSTLY PLACEHOLDERS
