#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *

# IO Override Top ------------------------------------------------------------------------------------

class IoOverrideTop(LiteXModule):
    """Generic, board-agnostic override-CSR wrapper for GPIO/LED/Fan-like pins.

    Supports three independent, optional pad categories, so the same override/CSR plumbing can
    serve bidirectional GPIO, output-only LEDs/Fan, and plain status inputs alike. Only the
    overriding mechanics live here; the default (non-overridden) behaviour is computed by the
    caller and fed in as plain Signal inputs (`dir`/`out_val`/`out_default`).

    - inout_pads (optional): bidirectional, tri-stated pads (e.g. FPGA_GPIO). Plain `dir`/
      `out_val` Signal inputs carry the caller-supplied default direction/value; `override`/
      `override_dir`/`override_val` CSRs let firmware force a direction/value; `val` CSRStatus
      reads the current pad value.
    - out_pads (optional): output-only pads (e.g. LEDs, Fan). A plain `out_default` Signal input
      carries the caller-supplied default value; `out_override`/`out_override_val` CSRs let
      firmware force a value.
    - in_pads (optional): input-only pads. `in_val` CSRStatus reads the current pad value (no
      override possible, since a physical input's electrical state cannot be forced).

    `name` prefixes all CSRs of this instance, so multiple instances can coexist on the same SoC.
    """
    def __init__(self, platform, name,
        inout_pads = None,
        out_pads   = None,
        in_pads    = None,
        ):
        self.platform = platform

        if inout_pads is None and out_pads is None and in_pads is None:
            raise ValueError("IoOverrideTop: at least one of inout_pads/out_pads/in_pads must be specified.")

        # Inout pads (bidirectional, tri-state) -------------------------------------------------
        if inout_pads is not None:
            n = len(inout_pads)

            self.dir     = Signal(n) # Default direction (0: Output, 1: Input), driven by the caller.
            self.out_val = Signal(n) # Default output value, driven by the caller.

            self.override = CSRStorage(n, name=f"{name}_override",
                description="GPIO Mode: 0: normal operation, 1: control is overriden."
            )
            self.override_dir = CSRStorage(n, name=f"{name}_override_dir", reset=(2**n - 1),
                description="GPIO override direction: 0: Output, 1: Input."
            )
            self.override_val = CSRStorage(n, name=f"{name}_override_val",
                description="GPIO Logic level: 0: High, 1: Low. (Dir must be set to output)"
            )
            self.val = CSRStatus(n, name=f"{name}_val", description="GPIO current value")

            val_bits = []
            for i, pad in enumerate(inout_pads):
                t = TSTriple()
                self.specials += t.get_tristate(pad)

                b = Signal()
                val_bits.append(b)
                self.comb += [
                    If(self.override.storage[i],
                        t.oe.eq(~self.override_dir.storage[i]),
                        t.o.eq( self.override_val.storage[i]),
                    ).Else(
                        t.oe.eq(~self.dir[i]),
                        t.o.eq( self.out_val[i]),
                    ),
                    # Always return the actual value present on the pin regardless of the
                    # selected direction (returns the driven value if direction is Output).
                    If(self.dir[i],
                        b.eq(t.i),
                    ).Else(
                        b.eq(t.o),
                    ),
                ]
            self.comb += self.val.status.eq(Cat(*val_bits))

        # Out pads (output-only) -----------------------------------------------------------------
        if out_pads is not None:
            m = len(out_pads)

            self.out_default = Signal(m) # Default output value, driven by the caller.

            self.out_override = CSRStorage(m, name=f"{name}_out_override",
                description="Output Mode: 0: normal operation, 1: control is overriden."
            )
            self.out_override_val = CSRStorage(m, name=f"{name}_out_override_val",
                description="Output override value."
            )

            for i in range(m):
                self.comb += [
                    If(self.out_override.storage[i],
                        out_pads[i].eq(self.out_override_val.storage[i]),
                    ).Else(
                        out_pads[i].eq(self.out_default[i]),
                    ),
                ]

        # In pads (input-only) -------------------------------------------------------------------
        if in_pads is not None:
            self.in_val = CSRStatus(len(in_pads), name=f"{name}_in_val", description="Input current value")
            self.comb += self.in_val.status.eq(in_pads)
