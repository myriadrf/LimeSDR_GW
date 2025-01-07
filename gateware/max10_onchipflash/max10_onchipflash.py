#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from shutil import which
import subprocess

from migen import *

from litex.gen import *

from litex.soc.interconnect     import wishbone
from litex.soc.interconnect.csr import CSRStatus, CSR

# Max10 OnChipFlash --------------------------------------------------------------------------------

class Max10OnChipFlash(LiteXModule):
    def __init__(self, platform):

        self.bus               = wishbone.Interface(data_width=32, adr_width=18)

        # On-Chip-Flash Status register.
        self._status_register  = CSRStatus(32, description="On-Chip Flash Status Register.")
        # On-Chip-Flash Control register.
        self._control_register = CSR(32)

        self.platform          = platform

        # # #

        # Signals.
        # --------

        avmm_csr_addr  = Signal()
        avmm_csr_read  = Signal()
        avmm_csr_rdata = Signal(32)

        # Data Interface (Avalon MM for Data Access)
        self.avmm_read               = avmm_read               = Signal()
        self.avmm_write              = avmm_write              = Signal()
        self.avmm_data_waitrequest   = avmm_data_waitrequest   = Signal()
        self.avmm_data_readdatavalid = avmm_data_readdatavalid = Signal()
        self.avmm_data_burstcount    = avmm_data_burstcount    = Signal(4)

        # max10_onchipflash Instance.
        # -------------------------------------
        self.specials += Instance("max10_onchipflash",
            # Clk/Reset.
            i_clock                   = ClockSignal("sys"),        # clk.clk
            i_reset_n                 = ~ResetSignal("sys"),       # nreset.reset_n

            # CSR Interface.
            i_avmm_csr_addr           = avmm_csr_addr,             # csr.address
            i_avmm_csr_read           = avmm_csr_read,             #    .read
            o_avmm_csr_readdata       = avmm_csr_rdata,            #    .readdata
            i_avmm_csr_writedata      = self._control_register.r,  #    .writedata
            i_avmm_csr_write          = self._control_register.re, #    .write

            # Data Interface (Avalon MM)
            i_avmm_data_addr          = self.bus.adr,
            i_avmm_data_read          = avmm_read,
            i_avmm_data_write         = avmm_write,
            i_avmm_data_writedata     = self.bus.dat_w,
            o_avmm_data_readdata      = self.bus.dat_r,
            o_avmm_data_waitrequest   = avmm_data_waitrequest,
            o_avmm_data_readdatavalid = avmm_data_readdatavalid,
            i_avmm_data_burstcount    = avmm_data_burstcount,
        )

        # Logic.
        # ------

        # CSR.
        self.comb += [
            avmm_csr_addr.eq(               ~self._status_register.we), # 0: status, 1: control
            avmm_csr_read.eq(               self._status_register.we | self._control_register.w),
            self._status_register.status.eq(avmm_csr_rdata),
            self._control_register.w.eq(    avmm_csr_rdata),
        ]

        # Connect Data Interface to Wishbone.
        self.comb += [
            self.bus.ack.eq(        (~avmm_data_waitrequest & self.bus.cyc & self.bus.stb)),
            avmm_data_burstcount.eq(1),
            avmm_write.eq(          self.bus.we & self.bus.stb & self.bus.cyc),
            avmm_read.eq(           ~self.bus.we & self.bus.stb & self.bus.cyc),
        ]

    def do_finalize(self):

        curr_dir = os.path.abspath(os.path.dirname(__file__))

        qsys_file = os.path.join(curr_dir, "max10_onchipflash.qsys")
        self.platform.add_ip(qsys_file)

        if which("qsys-edit") is None:
            msg = "Unable to find Quartus toolchain, please:\n"
            msg += "- Add Quartus toolchain to your $PATH."
            raise OSError(msg)

        command = f"qsys-generate --synthesis {qsys_file}"

        ret = subprocess.run(command, shell=True)
        if ret.returncode != 0:
            raise OSError("Error occured during Quartus's script execution.")

