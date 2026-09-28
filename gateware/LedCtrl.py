#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *
from litex.gen import *

# Heartbeat ------------------------------------------------------------------------------------------

class Heartbeat(LiteXModule):
    """Free-running counter producing a slow (~1Hz-ish) toggle, replacing legacy `alive.vhd`.

    `width` matches `alive.vhd`'s 25-bit counter (`beat`, driven from bit 24), so the blink rate
    is identical to the legacy implementation for the same system clock frequency. Use
    `ClockDomainsRenamer` at the instantiation site if a clock domain other than "sys" is needed.
    """
    def __init__(self, width=25):
        self.beat = Signal()

        # # #

        cnt = Signal(width)
        self.sync += cnt.eq(cnt + 1)
        self.comb += self.beat.eq(cnt[width - 1])

# Adf Dac Led Status ----------------------------------------------------------------------------------

class AdfDacLedStatus(LiteXModule):
    """Reproduces `FPGA_LED2_ctrl.vhd`'s `last_val` DAC/ADF chip-select tracking register.

    Tracks which SPI peripheral (DAC or ADF) was last selected: if the DAC was last selected,
    both `default_g`/`default_r` are held low (LED2 off); otherwise (ADF last selected, which is
    also the reset state) they follow `adf_muxout` (green: locked, red: unlocked). Use
    `ClockDomainsRenamer` at the instantiation site if a clock domain other than "sys" is needed.
    """
    def __init__(self, adf_muxout, dac_cs_n, adf_cs_n):
        self.default_g = Signal()
        self.default_r = Signal()

        # # #

        # 0: ADF last selected (reset state), 1: DAC last selected.
        dac_selected = Signal(reset=0)
        self.sync += [
            If(dac_cs_n == 0,
                dac_selected.eq(1),
            ).Elif(adf_cs_n == 0,
                dac_selected.eq(0),
            )
        ]

        self.comb += [
            self.default_g.eq(~dac_selected & adf_muxout),
            self.default_r.eq(~dac_selected & ~adf_muxout),
        ]
