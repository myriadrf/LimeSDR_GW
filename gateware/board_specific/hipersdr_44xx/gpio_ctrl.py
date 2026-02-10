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


# PWR Cfg -----------------------------------------------------------------------------------------

class GPIO_Ctrl(LiteXModule):
    def __init__(self, platform):
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
        ----------------------------------------------------------------------------------
        | U114 (fw offset 0)         | U113 (fw offset 24)   | U115 (fw offset 48)       |
        ----------------------------------------------------------------------------------
        -- PORT 0                    |                       |                      ------
        ----------------------------------------------------------------------------------
        0: SW_RX_FILBANK_OUT_V2_CHD  | 0: VSD_LNA_IF_CHA_RX  | 0: ENABLE_5VIN
        1: SW_RX_FILBANK_OUT_V1_CHD  | 1: VBYP_LNA_IF_CHA_RX | 1: ENABLE_7.5VIN
        2: SW_RX_FILBANK_OUT_V1_CHC  | 2: VSD_LNA_IF_CHB_RX  | 2: REF_EN_GPS
        3: SW_RX_FILBANK_OUT_V2_CHC  | 3: VBYP_LNA_IF_CHB_RX | 3: REF_EN_OSC
        4: SW_RX_FILBANK_OUT_V2_CHB  | 4: VSD_LNA_IF_CHC_RX  | 4: PG_8PO
        5: SW_RX_FILBANK_OUT_V1_CHB  | 5: VBYP_LNA_IF_CHC_RX | 5: PG_6PO
        6: SW_RX_FILBANK_OUT_V2_CHA  | 6: VSD_LNA_IF_CHD_RX  | 6: ENABLE_TCXO_LMS8002M
        7: SW_RX_FILBANK_OUT_V1_CHA  | 7: VBYP_LNA_IF_CHD_RX | 7: Not connected
        ----------------------------------------------------------------------------------
        -- PORT 1                    |                       |                      ------
        ----------------------------------------------------------------------------------
        8: SW_RX_FILBANK_IN_V3_CHA   | 8:  SW_OUT_TX_CHD      | 8:  Not connected
        9: SW_RX_FILBANK_IN_V3_CHB   | 9:  SW_OUT_TX_CHC      | 9:  LMS8001B_TX_U6_RESETN
        10: SW_RX_FILBANK_IN_V3_CHC  | 10: SW_OUT_TX_CHB      | 10: LMS8001B_TX_U5_RESETN
        11: SW_RX_FILBANK_IN_V3_CHD  | 11: SW_OUT_TX_CHA      | 11: LMS8001A_RX_U3_RESETN
        12: SW_RX_FILBANK_OUT_V3_CHD | 12: SW_OUT_RX_CHC      | 12: LMS8001B_RX_U1_RESETN
        13: SW_RX_FILBANK_OUT_V3_CHC | 13: SW_OUT_RX_CHD      | 13: LMS8001A_RX_U4_RESETN
        14: SW_RX_FILBANK_OUT_V3_CHB | 14: SW_OUT_RX_CHA      | 14: LMS8001A_RX_U2_RESETN
        15: SW_RX_FILBANK_OUT_V3_CHA | 15: SW_OUT_RX_CHB      | 15: Not connected
        ---------------------------------------------------------------------------------
        -- PORT 2                    |                        |                    ------
        ---------------------------------------------------------------------------------
        16: SW_RX_FILBANK_IN_V1_CHD  | 16: SW_IN_RX_CHA       | 16: FAN_TACH1
        17: SW_RX_FILBANK_IN_V2_CHD  | 17: SW_IN_RX_CHB       | 17: FAN_PWM1
        18: SW_RX_FILBANK_IN_V1_CHC  | 18: SW_IN_RX_CHC       | 18: FAN_TACH0
        19: SW_RX_FILBANK_IN_V2_CHC  | 19: SW_IN_RX_CHD       | 19: FAN_PWM0
        20: SW_RX_FILBANK_IN_V1_CHB  | 20: SW_IN_TX_CHA       | 20: SW_50R_LNAtoPA_CHA
        21: SW_RX_FILBANK_IN_V2_CHB  | 21: SW_IN_TX_CHB       | 21: SW_50R_LNAtoPA_CHB
        22: SW_RX_FILBANK_IN_V1_CHA  | 22: SW_IN_TX_CHC       | 22: SW_50R_LNAtoPA_CHC
        23: SW_RX_FILBANK_IN_V2_CHA  | 23: SW_IN_TX_CHD       | 23: SW_50R_LNAtoPA_CHD

        ---------------------------------------------------------------------------------
        -- Non-Expander GPIO mappings -self.GPIO (fw offset 72)                    ------
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
        20: SW_RX_TDDFDD_CHA
        21: SW_RX_TDDFDD_CHB
        22: SW_RX_TDDFDD_CHC
        23: SW_RX_TDDFDD_CHD
        24: SW_RXTX_CHA
        25: SW_RXTX_CHB
        26: SW_RXTX_CHC
        27: SW_RXTX_CHD
        28: SW_PA_ONOFF_CHA
        29: SW_PA_ONOFF_CHB
        30: SW_PA_ONOFF_CHC
        31: SW_PA_ONOFF_CHD
        ---------------------------------------------------------------------------------
        -- Non-Expander GPIO mappings, extra set - self.GPIO2 (fw offset 103)      ------
        ---------------------------------------------------------------------------------
        0:  VADJ_EN_CHA
        1:  VADJ_EN_CHB
        2:  VADJ_EN_CHC
        3:  VADJ_EN_CHD
        4:  PA_EN_CHA
        5:  PA_EN_CHB
        6:  PA_EN_CHC
        7:  PA_EN_CHD
        """

        self.port_direction_u114 = CSRStorage(24,description="GPIO direction: 1: Input, 0: Output")
        self.port_direction_u113 = CSRStorage(24,description="GPIO direction: 1: Input, 0: Output")
        self.port_direction_u115 = CSRStorage(24,description="GPIO direction: 1: Input, 0: Output")
        self.polarity_inversion_u114 = CSRStorage(24,description="GPIO polarity: 1: Inverted, 0: Normal")
        self.polarity_inversion_u113 = CSRStorage(24,description="GPIO polarity: 1: Inverted, 0: Normal")
        self.polarity_inversion_u115 = CSRStorage(24,description="GPIO polarity: 1: Inverted, 0: Normal")
        self.port_out_value_114 = CSRStorage(24,description="GPIO output value: 1: High, 0: Low")
        self.port_out_value_113 = CSRStorage(24,description="GPIO output value: 1: High, 0: Low")
        self.port_out_value_115 = CSRStorage(24,description="GPIO output value: 1: High, 0: Low")
        self.port_in_value_114 = CSRStatus(24,description="GPIO input value: 1: High, 0: Low")
        self.port_in_value_113 = CSRStatus(24,description="GPIO input value: 1: High, 0: Low")
        self.port_in_value_115 = CSRStatus(24,description="GPIO input value: 1: High, 0: Low")

        # Non-expander, direct control GPIOs
        self.GPIO = CSRStorage(32,description="GPIO output value: 1: High, 0: Low")
        self.GPIO2 = CSRStorage(8, description="GPIO output value: 1: High, 0: Low")

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

            CSRField("u114_polarity_write", size=1, offset=3, reset=0),
            CSRField("u113_polarity_write", size=1, offset=4, reset=0),
            CSRField("u115_polarity_write", size=1, offset=5, reset=0),

            CSRField("u114_out_value_write", size=1, offset=6, reset=0),
            CSRField("u113_out_value_write", size=1, offset=7, reset=0),
            CSRField("u115_out_value_write", size=1, offset=8, reset=0),

            CSRField("u114_direction_read", size=1, offset=9, reset=0),
            CSRField("u113_direction_read", size=1, offset=10, reset=0),
            CSRField("u115_direction_read", size=1, offset=11, reset=0),

            CSRField("u114_polarity_read", size=1, offset=12, reset=0),
            CSRField("u113_polarity_read", size=1, offset=13, reset=0),
            CSRField("u115_polarity_read", size=1, offset=14, reset=0),

            CSRField("u114_out_value_read", size=1, offset=15, reset=0),
            CSRField("u113_out_value_read", size=1, offset=16, reset=0),
            CSRField("u115_out_value_read", size=1, offset=17, reset=0),

            CSRField("u114_in_value_read", size=1, offset=18, reset=0),
            CSRField("u113_in_value_read", size=1, offset=19, reset=0),
            CSRField("u115_in_value_read", size=1, offset=20, reset=0),
        ])
        # TODO: REGISTER CONTROL DOES NOT DO ANYTHING AT THE TIME
        #       ADD FIRMWARE SUPPORT AND POLLING/IRQ
        #       FOR NOW THESE CSR's ARE MOSTLY PLACEHOLDERS