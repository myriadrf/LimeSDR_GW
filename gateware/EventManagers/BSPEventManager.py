#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *

from litex.soc.integration.soc      import *

class BSPEventManager(LiteXModule):
    def __init__(self, width=8):
        """
        width = number of interrupt sources
        """

        # Vector of interrupt sources from FPGA logic
        self.isr_vect = Signal(width)

        # CSR: Software Trigger
        # pulse=True means 'fields.start' goes high for 1 cycle ONLY when you write '1'
        self.sw_trigger = CSRStorage(description="Software Trigger", fields=[
            CSRField("start", size=1, description="Write 1 to trigger", pulse=True)
        ])

        # Event manager
        self.ev = EventManager()

        # Create one event per interrupt source
        for i in range(width):
            setattr(self.ev, f"isr{i}", EventSourceProcess(edge="rising"))

        # Optional SW interrupt
        self.ev.sw = EventSourcePulse()

        self.ev.finalize()

        # ------------------------------------------------------------
        # Trigger logic
        # ------------------------------------------------------------

        for i in range(width):
            self.comb += getattr(self.ev, f"isr{i}").trigger.eq(
                self.isr_vect[i]
            )

        self.comb += self.ev.sw.trigger.eq(self.sw_trigger.fields.start)