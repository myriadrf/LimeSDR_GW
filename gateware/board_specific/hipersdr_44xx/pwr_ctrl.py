#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.common import FromFPGACfg

# PWR Cfg -----------------------------------------------------------------------------------------

class PWRCtrl(LiteXModule):
    def __init__(self, platform, pads=None):

        self.comb += [
            pads.LMK_PDN.eq(1),
            pads.PWR_EN_TCXO.eq(1)
            ]


        # CSRs.
        # -----
        self.reg00             = CSRStorage(fields=[
            CSRField("pwr_en_2p05",         size=1, offset=0, reset=0),
            CSRField("pwr_en_lmk",          size=1, offset=1, reset=0),
            CSRField("pafe_en_d1p0",        size=1, offset=2, reset=0),
            CSRField("pafe_en_a1p2",        size=1, offset=3, reset=0),
            CSRField("pafe_en_a1p8",        size=1, offset=4, reset=0),
            CSRField("pafe_en_a1p8_1",      size=1, offset=5, reset=0),
            CSRField("afe_dcdc_1p0_nrst",   size=1, offset=6, reset=0),
        ])

        self.reg01             = CSRStatus(fields=[
            CSRField("pg_en_2p05",      size=1, offset=0, reset=0),
            CSRField("pg_afe_avdd_1p2", size=1, offset=1,  reset=0),
        ])


        self.comb += [
            # export.
            # -------
            # FPGA Cfg.
            pads.PWR_EN_2P05.eq(        self.reg00.fields.pwr_en_2p05),
            pads.PWR_EN_LMK.eq(         self.reg00.fields.pwr_en_lmk),
            pads.PAFE_EN_D1P0.eq(       self.reg00.fields.pafe_en_d1p0),
            pads.PAFE_EN_A1P2.eq(       self.reg00.fields.pafe_en_a1p2),
            pads.PAFE_EN_A1P8.eq(       self.reg00.fields.pafe_en_a1p8),
            pads.PAFE_EN_A1P8_1.eq(     self.reg00.fields.pafe_en_a1p8_1),
            pads.AFE_DCDC_1P0_NRST.eq(  self.reg00.fields.afe_dcdc_1p0_nrst),


            self.reg01.fields.pg_en_2p05.eq(pads.PG_EN_2P05),
            self.reg01.fields.pg_afe_avdd_1p2.eq(pads.PG_AFE_AVDD_1P2),

        ]

