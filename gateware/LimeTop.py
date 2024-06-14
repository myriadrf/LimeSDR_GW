#!/usr/bin/env python3

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr              import *
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess
from litex.soc.interconnect                  import axi

from litescope import LiteScopeAnalyzer

from gateware.GpioTop import GpioTop

# Lime Top -----------------------------------------------------------------------------------------

class LimeTop(LiteXModule):
    """Lime Top Level

    This module serves as a potential top-level integration point for Lime Microsystems' design/cores
    within a LiteX target. It features an AXI-Lite MMAP interface (slave) connected to a soft CPU
    and two AXI stream interfaces intended for connection to PCIe DMA. The module also includes an
    example of VHDL integration in LiteX and a LiteScope Analyzer.

    Attributes
    ----------
    mmap : axi.AXILiteInterface
        AXI-Lite Memory Mapped interface (slave) for communication with the soft CPU.

    dma_tx : axi.AXIStreamInterface
        AXI Stream interface for transmitting data to PCIe DMA.

    dma_rx : axi.AXIStreamInterface
        AXI Stream interface for receiving data from PCIe DMA.

    scratch : CSRStorage
        A simple CSR storage register for demonstration purposes.

    gpio : GpioTop
        Instance of the VHDL GPIO module integrated into LiteX.

    analyzer : LiteScopeAnalyzer
        LiteScope logic analyzer instance for signal analysis.
    """
    def __init__(self, platform, sys_clk_freq):
        # AXI MMAP Bus (From CPU).
        self.mmap = axi.AXILiteInterface(address_width=32, data_width=32)

        # AXI Stream Buses (From/To PCIe DMAs)
        self.dma_tx = axi.AXIStreamInterface(data_width=64) # PCIe  -> Logic (or FPGA -> RFIC).
        self.dma_rx = axi.AXIStreamInterface(data_width=64) # Logic -> PCIe  (or RFIC -> FPGA).

        # # #

        # AXI MMAP.
        # ---------
        self.ram = axi.AXILiteSRAM(0x1000)
        self.comb += self.mmap.connect(self.ram.bus)

        # AXI Stream.
        # -----------
        # Perform a simple TX -> RX loopback.
        # This could be done directly with self.dma_rx.connect(self.dma_tx), but is made explicit here.
        self.comb += [
            self.dma_rx.valid.eq(self.dma_tx.valid),
            self.dma_rx.last.eq(self.dma_tx.last),
            self.dma_rx.data.eq(self.dma_tx.data),
            self.dma_tx.ready.eq(self.dma_rx.ready),
        ]

        # CSR Registers Example.
        # ----------------------
        # Adding a simple CSR storage register for demonstration purposes.
        self.scratch = CSRStorage(32, description="Scratch register for testing purposes.")

        # IRQ Example.
        # ------------
        self.ev = EventManager()
        self.ev.irq0 = EventSourceProcess(edge="rising")
        self.ev.irq1 = EventSourceProcess(edge="rising")
        self.ev.finalize()

        # Generate irq0 every 1 seconds.
        self.irq0_timer = WaitTimer(1.0*sys_clk_freq)
        self.comb += self.irq0_timer.wait.eq(~self.irq0_timer.done)
        self.comb += self.ev.irq0.trigger.eq(self.irq0_timer.done)

        # Generate irq1 every 2 seconds.
        self.irq1_timer = WaitTimer(2.0*sys_clk_freq)
        self.comb += self.irq1_timer.wait.eq(~self.irq1_timer.done)
        self.comb += self.ev.irq1.trigger.eq(self.irq1_timer.done)

        # VHDL GPIO example.
        # ------------------
        # Integrate a VHDL GPIO module and connect it to user_led2.
        gpio_top_led = platform.request_all("user_led2")
        self.gpio = GpioTop(platform, gpio_top_led)

        # LiteScope example.
        # ------------------
        # Setup LiteScope Analyzer to capture some of the AXI-Lite MMAP signals.
        analyzer_signals = [
            self.mmap.ar,
            self.mmap.r,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 512,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "lime_top_analyzer.csv"
        )
