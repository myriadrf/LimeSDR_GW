#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from migen import *

from litex.gen import *

from litex.soc.interconnect     import wishbone
from litex.soc.interconnect.csr import CSRStatus, CSRStorage

# Max10 OnChipFlash --------------------------------------------------------------------------------

class Max10OnChipFlash(LiteXModule):
    def __init__(self, platform):

        self.bus               = wishbone.Interface(data_width=32, adr_width=18)

        # On-Chip-Flash Status register.
        self._status_register  = CSRStatus(32,                       description="On-Chip Flash Status Register.")
        # On-Chip-Flash Control register.
        self._control_register = CSRStorage(32, write_from_dev=True, description="On-Chip Flash Control Register.")

        # # #

        # Signals.
        # --------

        avmm_csr_addr  = Signal()
        avmm_csr_read  = Signal()
        avmm_csr_rdata = Signal(32)

        # Data Interface (Avalon MM for Data Access)
        avmm_data_addr          = Signal(18)
        avmm_data_read          = Signal()
        avmm_data_write         = Signal()
        avmm_data_writedata     = Signal(32)
        avmm_data_readdata      = Signal(32)
        avmm_data_waitrequest   = Signal()
        avmm_data_readdatavalid = Signal()
        avmm_data_burstcount    = Signal(4)

        # max10_onchipflash Instance.
        # -------------------------------------
        self.specials += Instance("max10_onchipflash",
            # Clk/Reset.
            i_clock                   = ClockSignal("sys"),             # clk.clk
            i_reset_n                 = ~ResetSignal("sys"),            # nreset.reset_n

            # CSR Interface.
            i_avmm_csr_addr           = avmm_csr_addr,                  # csr.address
            i_avmm_csr_read           = avmm_csr_read,                  #    .read
            o_avmm_csr_readdata       = avmm_csr_rdata,                 #    .readdata
            i_avmm_csr_writedata      = self._control_register.storage, #    .writedata
            i_avmm_csr_write          = self._control_register.re,      #    .write

            # Data Interface (Avalon MM)
            i_avmm_data_addr          = avmm_data_addr,
            i_avmm_data_read          = avmm_data_read,
            i_avmm_data_write         = avmm_data_write,
            i_avmm_data_writedata     = avmm_data_writedata,
            o_avmm_data_readdata      = avmm_data_readdata,
            o_avmm_data_waitrequest   = avmm_data_waitrequest,
            o_avmm_data_readdatavalid = avmm_data_readdatavalid,
            i_avmm_data_burstcount    = avmm_data_burstcount,
        );

        # Logic.
        # ------

        # CSR.
        self.comb += [
            avmm_csr_addr.eq(                 ~self._status_register.we), # 0: status, 1: control
            avmm_csr_read.eq(                 self._status_register.we | self._control_register.we),
            self._status_register.status.eq(  avmm_csr_rdata),
            self._control_register.storage.eq(avmm_csr_rdata),
        ]

        # Connect Data Interface to Wishbone.
        self.comb += [
            avmm_data_addr.eq(      self.bus.adr),
            avmm_data_writedata.eq( self.bus.dat_w),
            avmm_data_write.eq(     self.bus.we & self.bus.stb),
            avmm_data_read.eq(      ~self.bus.we & self.bus.stb),
            self.bus.dat_r.eq(      avmm_data_readdata),
            self.bus.ack.eq(        ~avmm_data_waitrequest),
            avmm_data_burstcount.eq(1),
        ]

        self.add_sources(platform)


    def add_sources(self, platform):
        base_path = "gateware/max10_onchipflash/synthesis"
        files = [
            "max10_onchipflash.v",
            "submodules/altera_onchip_flash_avmm_csr_controller.v",
            "submodules/altera_onchip_flash.v",
            "submodules/altera_onchip_flash_avmm_data_controller.v",
            "submodules/altera_onchip_flash_util.v",
            "submodules/rtl/altera_onchip_flash_block.v",
        ]

        for file in files:
            platform.add_source(os.path.join(base_path, file))
