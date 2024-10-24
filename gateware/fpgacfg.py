#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# Constants ----------------------------------------------------------------------------------------
MAJOR_REV           = 2
MINOR_REV           = 2
BETA_REV            = 1
COMPILE_REV         = 6
COMPILE_YEAR_STAMP  = 19
COMPILE_MONTH_STAMP = 5
COMPILE_DAY_STAMP   = 7
COMPILE_HOUR_STAMP  = 14

MAGIC_NUM           = 0xD8A5F009
BOARD_ID            = 0x0011 # LimeSDR-MINI

# FPGA Cfg -----------------------------------------------------------------------------------------

class FPGACfg(LiteXModule):
    def __init__(self):
        self.board_id       = CSRStatus(16,  reset=BOARD_ID)
        self.major_rev      = CSRStatus(16,  reset=MAJOR_REV)
        self.compile_rev    = CSRStatus(16,  reset=COMPILE_REV)
        self.reserved_03    = CSRStorage(16, reset=0)
        self.reserved_04    = CSRStorage(16, reset=0)
        self.reserved_05    = CSRStorage(16, reset=0)
        self.reserved_06    = CSRStorage(16, reset=0)
        self.channel_cntrl  = CSRStorage(fields=[
            CSRField("ch_en", size=2, offset=0, values=[
                ("``2b01", "Channel A"),
                ("``2b10", "Channel B"),
                ("``2b11", "Channels A and B")
            ], reset=0)
        ])
