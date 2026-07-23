#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2019-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2025 Lime Microsystems.
#
# SPDX-License-Identifier: BSD-2-Clause

"""
Monkey-patch: runtime-selectable SPI clock phase (CPHA) for the LiteX ``SPIMaster``.

Background
----------
The stock LiteX ``SPIMaster`` (``litex/soc/cores/spi/spi_master.py``) is hard-wired to
SPI Mode 0 (CPOL=0, CPHA=0): MOSI is launched on the SCLK *falling* edge and MISO is
sampled on the SCLK *rising* edge. Some devices (e.g. the AD5601 VCTCXO DAC on the
LimeSDR-USB) latch SDIN on the *falling* edge of SCLK, i.e. they require CPHA=1
(SPI Mode 1). When such a device shares one fixed-mode master with a Mode-0 device
(the ADF4002 PLL on ``fpga_spi1``), a per-transfer clock-phase control is needed.

This module adds a runtime-selectable ``cpha`` control to ``SPIMaster``:

    * ``cpha = 0`` -> CPHA=0 (SPI Mode 0 with CPOL=0): MOSI launched on the falling
      edge, MISO sampled on the rising edge. This is the *default* and is bit-for-bit
      identical to the un-patched core, so all existing Mode-0 devices keep working
      unchanged.
    * ``cpha = 1`` -> CPHA=1 (SPI Mode 1 with CPOL=0): MOSI launched on the rising
      edge, MISO sampled on the falling edge.

The control is exposed both as a plain Migen ``Signal`` (``spim.cpha``) for gateware
use and, when the core builds its CSRs, as a ``<name>_phase`` CSR with a single
``cpha`` field for firmware use.

This is deliberately implemented as a monkey-patch (rather than editing the pinned
``deps/litex`` sources, which ``setup_litex.sh`` re-clones at a fixed SHA) so the
functionality is available immediately; the same change is intended to be submitted
upstream to LiteX as a pull request.

Usage
-----
Import and apply the patch *before* any ``SPIMaster`` / ``add_spi_master`` is
instantiated (typically at the top of a board target, next to the other patches)::

    from tools.spi_cpha_patch import patch_spi_master_cpha
    patch_spi_master_cpha()

Then, per transfer, drive the phase either from gateware::

    self.comb += spim.cpha.eq(<condition>)   # e.g. select Mode 1 for a given CS

or from firmware via the generated CSR (``<name>_phase``), e.g.::

    fpga_spi1_phase_write(1);   // Mode 1 before a DAC transfer
    ...                         // do the transfer
    fpga_spi1_phase_write(0);   // back to Mode 0 for the ADF4002
"""

import math

from migen import *

from litex.soc.interconnect.csr import *

# Import the module (not just the class) so we patch the single class object that every
# ``from litex.soc.cores.spi import SPIMaster`` reference resolves to.
from litex.soc.cores.spi import spi_master as _spi_master_mod
from litex.soc.cores.spi.spi_master import SPIMaster as _SPIMaster


# Patched SPIMaster.__init__ (adds cpha + edge selection) ------------------------------------------

def _spi_master_init_cpha(self, pads, data_width, sys_clk_freq, spi_clk_freq, with_csr=True, mode="raw"):
    assert mode in ["raw", "aligned"]
    self.mode = mode
    if pads is None:
        pads = Record(self.pads_layout)
    if not hasattr(pads, "cs_n"):
        pads.cs_n = Signal()
    assert len(pads.cs_n) <= 16
    self.pads       = pads
    self.data_width = data_width

    self.start       = Signal()
    self.length      = Signal(8)
    self.done        = Signal()
    self.irq         = Signal()
    self.mosi        = Signal(data_width)
    self.miso        = Signal(data_width)
    self.cs          = Signal(len(pads.cs_n), reset=1)
    self.cs_mode     = Signal()
    self.loopback    = Signal()
    self.clk_divider = Signal(16, reset=math.ceil(sys_clk_freq/spi_clk_freq))
    # [CPHA patch] Runtime-selectable clock phase (CPOL is fixed at 0).
    #   cpha = 0 : SPI Mode 0 - MOSI launched on falling edge, MISO sampled on rising edge (default).
    #   cpha = 1 : SPI Mode 1 - MOSI launched on rising  edge, MISO sampled on falling edge.
    self.cpha        = Signal()

    if with_csr:
        self.add_csr()

    # # #

    clk_enable  = Signal()
    xfer_enable = Signal()
    count       = Signal(max=data_width)
    mosi_latch  = Signal()
    miso_latch  = Signal()

    # Clock generation -------------------------------------------------------------------------
    clk_divider = Signal(16)
    clk_rise    = Signal()
    clk_fall    = Signal()
    self.comb += clk_rise.eq(clk_divider == (self.clk_divider[1:] - 1))
    self.comb += clk_fall.eq(clk_divider == (self.clk_divider     - 1))
    self.sync += [
        clk_divider.eq(clk_divider + 1),
        If(clk_rise,
            pads.clk.eq(clk_enable),
        ).Elif(clk_fall,
            clk_divider.eq(0),
            pads.clk.eq(0),
        )
    ]

    # [CPHA patch] Launch/sample edges selected by cpha.
    #   cpha = 0 -> launch on fall, sample on rise (Mode 0, unchanged).
    #   cpha = 1 -> launch on rise, sample on fall (Mode 1).
    mosi_edge = Signal()
    miso_edge = Signal()
    self.comb += [
        mosi_edge.eq(Mux(self.cpha, clk_rise, clk_fall)),
        miso_edge.eq(Mux(self.cpha, clk_fall, clk_rise)),
    ]

    # Control FSM ------------------------------------------------------------------------------
    self.fsm = fsm = FSM(reset_state="IDLE")
    fsm.act("IDLE",
        self.done.eq(1),
        If(self.start,
            self.done.eq(0),
            mosi_latch.eq(1),
            NextState("START")
        )
    )
    fsm.act("START",
        NextValue(count, 0),
        If(clk_fall,
            xfer_enable.eq(1),
            NextState("RUN")
        )
    )
    fsm.act("RUN",
        clk_enable.eq(1),
        xfer_enable.eq(1),
        If(clk_fall,
            NextValue(count, count + 1),
            If(count == (self.length - 1),
                NextState("STOP")
            )
        )
    )
    fsm.act("STOP",
        xfer_enable.eq(1),
        If(clk_rise,
            miso_latch.eq(1),
            self.irq.eq(1),
            NextState("IDLE")
        )
    )

    # Chip Select generation -------------------------------------------------------------------
    if hasattr(pads, "cs_n"):
        for i in range(len(pads.cs_n)):
            # CS set when enabled and (Xfer enabled or Manual CS mode selected).
            cs = (self.cs[i] & (xfer_enable | (self.cs_mode == 1)))
            # CS Output/Invert.
            self.sync += pads.cs_n[i].eq(~cs)

    # Master Out Slave In (MOSI) generation (launched on mosi_edge) ----------------------------
    mosi_data  = Signal(data_width)
    mosi_array = Array(mosi_data[i] for i in range(data_width))
    mosi_sel   = Signal(max=data_width)
    self.sync += [
        If(mosi_latch,
            mosi_data.eq(self.mosi),
            mosi_sel.eq((self.length-1) if mode == "aligned" else (data_width-1)),
        ).Elif(mosi_edge,
            If(xfer_enable, pads.mosi.eq(mosi_array[mosi_sel])),
            mosi_sel.eq(mosi_sel - 1)
        ),
    ]

    # Master In Slave Out (MISO) capture (sampled on miso_edge) --------------------------------
    miso      = Signal()
    miso_data = Signal(data_width)
    self.sync += [
        If(miso_edge,
            If(self.loopback,
                miso_data.eq(Cat(pads.mosi, miso_data))
            ).Else(
                miso_data.eq(Cat(pads.miso, miso_data))
            )
        )
    ]
    self.sync += If(miso_latch, self.miso.eq(miso_data))


# Patched SPIMaster.add_csr (adds a <name>_phase CSR) ---------------------------------------------

def _spi_master_add_csr_cpha(self, with_cs=True, with_loopback=True, with_cpha=True):
    # Control / Status.
    self._control = CSRStorage(description="SPI Control.", fields=[
        CSRField("start",  size=1, offset=0, pulse=True, description="SPI Xfer Start (Write ``1`` to start Xfer)."),
        CSRField("length", size=8, offset=8,             description="SPI Xfer Length (in bits).")
    ])
    self._status = CSRStatus(description="SPI Status.", fields=[
        CSRField("done", size=1, offset=0, description="SPI Xfer Done (when read as ``1``)."),
        CSRField("mode", size=1, offset=1, description="SPI mode", values=[
            ("``0b0``", "Raw    : MOSI transfers aligned on core's data-width."),
            ("``0b1``", "Aligned: MOSI transfers aligned on transfers' length."),
        ]),
    ])
    self.comb += [
        self.start.eq(self._control.fields.start),
        self.length.eq(self._control.fields.length),
        self._status.fields.done.eq(self.done),
        self._status.fields.mode.eq({"raw": 0b0, "aligned": 0b1}[self.mode]),
    ]

    # MOSI/MISO.
    self._mosi = CSRStorage(self.data_width, reset_less=True, description="SPI MOSI data (MSB-first serialization).")
    self._miso = CSRStatus(self.data_width,                   description="SPI MISO data (MSB-first de-serialization).")
    self.comb += [
        self.mosi.eq(self._mosi.storage),
        self._miso.status.eq(self.miso),
    ]

    # Chip Select.
    if with_cs:
        self._cs = CSRStorage(description="SPI CS Chip-Select and Mode.", fields=[
            CSRField("sel",  size=len(self.cs), offset=0,  reset=1, values=[
                ("``0b0..001``", "Chip ``0`` selected for SPI Xfer."),
                ("``0b1..000``", "Chip ``N`` selected for SPI Xfer.")
            ]),
            CSRField("mode", size=1,            offset=16, reset=0, values=[
                ("``0b0``", "Normal operation (CS handled by Core)."),
                ("``0b1``", "Manual operation (CS handled by User, direct recopy of ``sel``), useful for Bulk transfers.")
            ]),
        ])
        self.comb += [
            self.cs.eq(self._cs.fields.sel),
            self.cs_mode.eq(self._cs.fields.mode)
        ]

    # Loopback.
    if with_loopback:
        self._loopback = CSRStorage(description="SPI Loopback Mode.", fields=[
            CSRField("mode", size=1, values=[
                ("``0b0``", "Normal operation."),
                ("``0b1``", "Loopback operation (MOSI to MISO).")
            ])
        ])
        self.comb += self.loopback.eq(self._loopback.fields.mode)

    # [CPHA patch] Clock Phase.
    if with_cpha:
        self._phase = CSRStorage(description="SPI Clock Phase (CPHA, CPOL fixed at 0).", fields=[
            CSRField("cpha", size=1, offset=0, reset=0, values=[
                ("``0b0``", "CPHA=0 (SPI Mode 0): MOSI launched on falling edge, MISO sampled on rising edge."),
                ("``0b1``", "CPHA=1 (SPI Mode 1): MOSI launched on rising edge, MISO sampled on falling edge."),
            ]),
        ])
        self.comb += self.cpha.eq(self._phase.fields.cpha)


# Patch application -------------------------------------------------------------------------------

def patch_spi_master_cpha(verbose=True):
    """Apply the runtime CPHA patch to ``litex.soc.cores.spi.SPIMaster``.

    Idempotent: calling it multiple times has no additional effect. Must be called
    before any ``SPIMaster`` (or ``SoCCore.add_spi_master``) is instantiated.
    """
    if getattr(_SPIMaster, "_cpha_patched", False):
        return False

    _SPIMaster.__init__ = _spi_master_init_cpha
    _SPIMaster.add_csr  = _spi_master_add_csr_cpha
    _SPIMaster._cpha_patched = True

    # Keep the module-level reference in sync (it is the same class object, but be explicit).
    _spi_master_mod.SPIMaster = _SPIMaster

    if verbose:
        print("[spi_cpha_patch] Patched litex SPIMaster with runtime-selectable CPHA (adds 'cpha' signal + '<name>_phase' CSR).")
    return True


# Self-test (run: python3 tools/spi_cpha_patch.py) -----------------------------------------------

if __name__ == "__main__":
    patch_spi_master_cpha()

    from migen.fhdl.verilog import convert

    # Build a CSR-less instance so cpha stays a top-level input we can inspect/drive.
    dut = _SPIMaster(pads=None, data_width=8, sys_clk_freq=100e6, spi_clk_freq=1e6, with_csr=False)
    assert hasattr(dut, "cpha"), "cpha signal missing after patch"

    verilog = convert(dut).main_source
    assert "cpha" in verilog, "cpha not present in generated Verilog"
    print("[spi_cpha_patch] Verilog generation OK, 'cpha' present.")

    # Functional check: MOSI must update on the rising SCLK edge when cpha=1 and on the
    # falling edge when cpha=0. We detect the update edge by watching pads.mosi transitions
    # relative to pads.clk transitions during an all-ones then alternating transfer.
    def _run(cpha_val):
        dut2 = _SPIMaster(pads=None, data_width=8, sys_clk_freq=8e6, spi_clk_freq=1e6, with_csr=False)
        results = {"clk_edges": [], "mosi_edges": []}

        def bench():
            yield dut2.cpha.eq(cpha_val)
            yield dut2.mosi.eq(0xA5)  # 1010_0101, guarantees MOSI transitions
            yield dut2.length.eq(8)
            yield dut2.cs.eq(1)
            yield
            yield dut2.start.eq(1)
            yield
            yield dut2.start.eq(0)
            prev_clk  = 0
            prev_mosi = 0
            for t in range(400):
                cur_clk  = (yield dut2.pads.clk)
                cur_mosi = (yield dut2.pads.mosi)
                if cur_clk != prev_clk:
                    results["clk_edges"].append((t, "rise" if cur_clk else "fall"))
                if cur_mosi != prev_mosi:
                    results["mosi_edges"].append((t, cur_clk))  # clk level at the MOSI change
                prev_clk, prev_mosi = cur_clk, cur_mosi
                yield
            if (yield dut2.done):
                pass

        run_simulation(dut2, bench())
        return results

    from migen.sim import run_simulation  # noqa: E402  (imported here to keep top clean)

    r0 = _run(0)
    r1 = _run(1)

    # For cpha=0 the MOSI transitions coincide with the clock going low (falling edge just happened
    # -> clk level 0). For cpha=1 they coincide with the clock going high (rising edge -> clk level 1).
    def _summarize(res):
        # Ignore the very first setup transition; look at steady-state MOSI updates during the burst.
        levels = [lvl for (_t, lvl) in res["mosi_edges"]]
        return levels

    lv0 = _summarize(r0)
    lv1 = _summarize(r1)
    print(f"[spi_cpha_patch] cpha=0 MOSI-update clk levels: {lv0}")
    print(f"[spi_cpha_patch] cpha=1 MOSI-update clk levels: {lv1}")

    # Look at the per-bit launch edges only: the final transition is the end-of-transfer /
    # CS-deassertion boundary (clock has stopped toggling), so it is excluded from the check.
    body0 = lv0[:-1] if len(lv0) > 1 else lv0
    body1 = lv1[:-1] if len(lv1) > 1 else lv1

    # cpha=0: data bits launched while clk is low (0) -> falling edge (SPI Mode 0).
    # cpha=1: data bits launched while clk is high (1) -> rising  edge (SPI Mode 1).
    assert body0 and all(l == 0 for l in body0), f"cpha=0 expected MOSI launched on falling edge (clk low), got {lv0}"
    assert body1 and all(l == 1 for l in body1), f"cpha=1 expected MOSI launched on rising edge (clk high), got {lv1}"
    assert body0 != body1, "cpha=0 and cpha=1 must produce different launch edges"
    print("[spi_cpha_patch] Self-test PASSED: MOSI launch edge follows cpha (Mode 0 vs Mode 1).")
