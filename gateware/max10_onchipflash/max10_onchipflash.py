#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from shutil import which, copyfile
import subprocess

from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect     import wishbone
from litex.soc.interconnect.csr import CSRStatus, CSRStorage

# Max10 OnChipFlash --------------------------------------------------------------------------------

class Max10OnChipFlash(LiteXModule):
    def __init__(self, platform, ufm_hex=None):

        self.bus               = wishbone.Interface(data_width=32, adr_width=18)

        # On-Chip-Flash Status register.
        self._status_register  = CSRStatus(32,  description="On-Chip Flash Status Register.")
        # On-Chip-Flash Control register.
        self._control_register = CSRStorage(32, description="On-Chip Flash Control Register.")

        self.platform          = platform

        self.ufm_hex           = ufm_hex

        # # #

        # Signals.
        # --------

        avmm_csr_addr  = Signal()
        avmm_csr_read  = Signal()
        avmm_csr_rdata = Signal(32)
        avmm_csr_write = Signal()
        avmm_csr_wdata = Signal(32)

        # Data Interface (Avalon MM for Data Access)
        avmm_data_addr          = Signal(18)
        avmm_data_read          = Signal()
        avmm_data_readdata      = Signal(32)
        avmm_data_readdatavalid = Signal()
        avmm_data_write         = Signal()
        avmm_data_writedata     = Signal(32)
        avmm_data_waitrequest   = Signal()
        avmm_data_burstcount    = Signal(4)

        # max10_onchipflash Instance.
        # -------------------------------------
        self.specials += Instance("max10_onchipflash",
            # Clk/Reset.
            i_clock                   = ClockSignal("sys"),  # clk.clk
            i_reset_n                 = ~ResetSignal("sys"), # nreset.reset_n

            # CSR Interface.
            i_avmm_csr_addr           = avmm_csr_addr,       # csr.address
            i_avmm_csr_read           = avmm_csr_read,       #    .read
            o_avmm_csr_readdata       = avmm_csr_rdata,      #    .readdata
            i_avmm_csr_write          = avmm_csr_write,      #    .write
            i_avmm_csr_writedata      = avmm_csr_wdata,      #    .writedata

            # Data Interface (Avalon MM)
            i_avmm_data_addr          = avmm_data_addr,
            i_avmm_data_read          = avmm_data_read,
            i_avmm_data_write         = avmm_data_write,
            i_avmm_data_writedata     = avmm_data_writedata,
            o_avmm_data_readdata      = avmm_data_readdata,
            o_avmm_data_waitrequest   = avmm_data_waitrequest,
            o_avmm_data_readdatavalid = avmm_data_readdatavalid,
            i_avmm_data_burstcount    = avmm_data_burstcount,
        )

        # Logic.
        # ------

        # CSR.
        self.comb += [
            avmm_csr_addr.eq(               self._control_register.re), # 0: status, 1: control
            avmm_csr_read.eq(               self._status_register.we),
            self._status_register.status.eq(avmm_csr_rdata),
            avmm_csr_write.eq(              self._control_register.re),
            avmm_csr_wdata.eq(              self._control_register.storage),
        ]

        # Connect Data Interface to Wishbone.
        self.comb += avmm_data_burstcount.eq(1),

        first = Signal()
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(first, 1),
            If(self.bus.stb & self.bus.cyc,
                If(self.bus.we,
                    NextState("WRITE")
                ).Else(
                    NextState("READ-REQ")
                )
            ),
        )
        fsm.act("WRITE",
            NextValue(first, 0),
            avmm_data_write.eq(1),
            avmm_data_addr.eq(self.bus.adr),
            avmm_data_writedata.eq(self.bus.dat_w),
            If(~first & ~avmm_data_waitrequest,
                self.bus.ack.eq(1),
                NextState("IDLE")
            )
        )
        fsm.act("READ-REQ",
            NextValue(first, 0),
            avmm_data_read.eq(1),
            avmm_data_addr.eq(self.bus.adr),
            If(~first & ~avmm_data_waitrequest,
                NextState("READ-DAT")
            )
        )
        fsm.act("READ-DAT",
            If(avmm_data_readdatavalid,
                self.bus.ack.eq(1),
                self.bus.dat_r.eq(avmm_data_readdata),
                NextState("IDLE")
            )
        )

    def do_finalize(self):
        curr_dir = os.path.abspath(os.path.dirname(__file__))

        # Updates firmware path.
        qsys_src = os.path.join(curr_dir, "max10_onchipflash_template.qsys")
        qsys_dst = os.path.join(curr_dir, "max10_onchipflash.qsys")
        copyfile(qsys_src, qsys_dst)

        if self.ufm_hex is not None:
            tools.replace_in_file(qsys_dst, "INITFLASHCONTENT", "true")
            tools.replace_in_file(qsys_dst, "UFM_HEX", self.ufm_hex)
            tools.replace_in_file(qsys_dst, "USENONDEFAULTINITFILE", "true")
        else:
            tools.replace_in_file(qsys_dst, "INITFLASHCONTENT", "false")
            tools.replace_in_file(qsys_dst, "UFM_HEX", "altera_onchip_flash.hex")
            tools.replace_in_file(qsys_dst, "USENONDEFAULTINITFILE", "false")

        self.platform.add_ip(qsys_dst)

        if which("qsys-edit") is None:
            msg = "Unable to find Quartus toolchain, please:\n"
            msg += "- Add Quartus toolchain to your $PATH."
            raise OSError(msg)

        command = f"qsys-generate --synthesis {qsys_dst}"

        ret = subprocess.run(command, shell=True)
        if ret.returncode != 0:
            raise OSError("Error occured during Quartus's script execution.")
