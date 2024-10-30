#!/usr/bin/env python3

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr              import *
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess
from litex.soc.interconnect                  import axi

from litex.soc.interconnect import stream

from litescope import LiteScopeAnalyzer

from litex.soc.cores.spi       import SPIMaster

from gateware.GpioTop import GpioTop
from gateware.LimeDFB.lms7002.src.lms7002_top import lms7002_top
from gateware.LimeDFB.rx_path_top.src.rx_path_top import rx_path_top
from gateware.LimeDFB.tx_path_top.src.tx_path_top import tx_path_top

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
    def __init__(self, platform, sys_clk_freq, with_debug=False):
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
        #self.comb += [
        #    self.dma_rx.valid.eq(self.dma_tx.valid),
        #    self.dma_rx.last.eq(self.dma_tx.last),
        #    self.dma_rx.data.eq(self.dma_tx.data),
        #    self.dma_tx.ready.eq(self.dma_rx.ready),
        #]

        # CSR Registers Example.
        # ----------------------
        # Adding a simple CSR storage register for demonstration purposes.
        self.scratch = CSRStorage(32, description="Scratch register for testing purposes.")




        #LMS7002
        lms7002_pads = platform.request("lms7002m")
        self.lms7002 = lms7002_top(platform, lms7002_pads, s_clk_domain="txclk")
        self.comb += self.lms7002.axis_s.areset_n.eq(self.lms7002.tx_en.storage)
        self.comb += self.lms7002.axis_m.areset_n.eq(self.lms7002.tx_en.storage)



        # VHDL GPIO example.
        # ------------------
        # Integrate a VHDL GPIO module and connect it to user_led2.
        gpio_top_led = platform.request_all("user_led2")
        self.gpio = GpioTop(platform, gpio_top_led)


        # RX Path
        self.rx_path = rx_path_top(platform)
        self.comb += self.rx_path.RESET_N.eq(self.lms7002.tx_en.storage)

        # Connect RX path AXIS slave to lms7002 AXIS master
        # The line below is commented to disconnect the RX path from the LMS7002
        # self.comb += self.lms7002.axis_m.connect(self.rx_path.s_axis_iqsmpls)
        self.comb += self.rx_path.s_axis_iqsmpls.areset_n.eq(self.lms7002.tx_en.storage)


        self.comb += [
            self.dma_rx.valid.eq(self.rx_path.m_axis_iqpacket.valid),
            self.dma_rx.last.eq(self.rx_path.m_axis_iqpacket.last),
            self.dma_rx.data.eq(self.rx_path.m_axis_iqpacket.data),
            self.rx_path.m_axis_iqpacket.ready.eq(self.dma_rx.ready),
        ]
        self.comb += self.rx_path.m_axis_iqpacket.areset_n.eq(self.lms7002.tx_en.storage)


        # TX Path
        self.tx_path = tx_path_top(platform, buff_count=2, rx_clk_domain="rxclk", m_clk_domain="txclk")
        #self.comb += self.tx_path.RESET_N.eq(self.lms7002.tx_en.storage)
        self.comb += self.tx_path.RX_SAMPLE_NR.eq(self.rx_path.SMPL_NR_OUT)
        # DMA -> tx_path_top
        self.comb += self.dma_tx.connect(self.tx_path.s_axis_iqpacket, keep={"valid", "ready", "last", "data"},
                                         omit={"areset_n, keep"})
        self.comb += self.tx_path.s_axis_iqpacket.areset_n.eq(self.lms7002.tx_en.storage)
        # tx_path_top -> lms7002_top
        self.comb += self.tx_path.m_axis_iqsample.connect(self.lms7002.axis_s, keep={"valid", "ready", "last", "data"},
                                                          omit={"areset_n, keep"})
        self.comb += self.tx_path.m_axis_iqsample.areset_n.eq(self.lms7002.tx_en.storage)

        # Use pre-generated FFT module by default
        generate_fft = False
        if generate_fft:
            # amlib fft module
            # import required modules
            from amaranth.back import verilog
            from gateware.examples.fft.fixedpointfft import FixedPointFFT
            # from amlib.dsp import FixedPointFFT
            # create fft module
            fft = FixedPointFFT(bitwidth=12, pts=512, verbose=True)
            # generate verilog
            verilog_code = verilog.convert(fft, name="fft",  strip_internal_attrs=False, ports=[fft.in_i,fft.in_q,fft.out_real,fft.out_imag,fft.strobe_in,fft.strobe_out,fft.start,fft.done,fft.wf_start,fft.wf_strobe,fft.wf_real,fft.wf_imag,fft.wr_state,fft.wr_state_valid])
            # write verilog to file
            with open("./gateware/examples/fft/fft.v", "w") as f:
                f.write(verilog_code)
        # import AXIStreamInterface description
        from litex.soc.interconnect.axi import AXIStreamInterface
        # describe layouts for s_axis and m_axis interfaces for fft wrapper
        # definitions copied from rx_path_top to ensure same layout
        s_axis_layout = [("data", max(1, 64))]
        s_axis_layout += [("areset_n", 1)]
        s_axis_layout += [("keep", max(1, 64//8))]
        #
        m_axis_layout = [("data", max(1, 64))]
        m_axis_layout += [("areset_n", 1)]
        m_axis_layout += [("keep", max(1, 64//8))]
        # declare fft interfaces
        self.fft_s_axis = AXIStreamInterface(data_width=64, layout=s_axis_layout, clock_domain=self.lms7002.axis_m.clock_domain)
        self.fft_m_axis = AXIStreamInterface(data_width=64, layout=m_axis_layout, clock_domain=self.lms7002.axis_m.clock_domain)
        # add fft and wrapper to sources
        platform.add_source("./gateware/examples/fft/fft.v")
        platform.add_source("./gateware/examples/fft/fft_wrap.vhd")

        # assign fft wrapper ports to appropriate interfaces
        self.fft_params = dict()
        self.fft_params.update(
            i_CLK = ClockSignal(self.lms7002.axis_m.clock_domain),
            i_RESET_N = self.lms7002.tx_en.storage,
            i_S_AXIS_TVALID = self.fft_s_axis.valid,
            i_S_AXIS_TDATA = self.fft_s_axis.data,
            o_S_AXIS_TREADY = self.fft_s_axis.ready,
            i_S_AXIS_TLAST = self.fft_s_axis.last,
            i_S_AXIS_TKEEP = self.fft_s_axis.keep,
            #
            o_M_AXIS_TDATA = self.fft_m_axis.data,
            o_M_AXIS_TVALID = self.fft_m_axis.valid,
            i_M_AXIS_TREADY = self.fft_m_axis.ready,
            o_M_AXIS_TLAST = self.fft_m_axis.last,
            o_M_AXIS_TKEEP = self.fft_m_axis.keep
        )
        # instantiate fft wrapper
        self.specials += Instance("fft_wrap", **self.fft_params)

        # connect the lms7002 master interface to the fft wrapper slave interface
        self.comb += self.lms7002.axis_m.connect(self.fft_s_axis,omit={"areset_n"})
        # connect the fft wrapper master interface to the rx_path slave interface
        self.comb += self.fft_m_axis.connect(self.rx_path.s_axis_iqsmpls,omit={"areset_n"})


        # LiteScope example.
        # ------------------
        # Setup LiteScope Analyzer to capture some of the AXI-Lite MMAP signals.
        if with_debug:
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


        # IRQ Example.
        # ------------
        self.ev = EventManager()
        self.ev.clk_ctrl_irq = EventSourceProcess()
        self.ev.finalize()

        self.comb += self.ev.clk_ctrl_irq.trigger.eq((self.lms7002.CLK_CTRL.PHCFG_START.re & self.lms7002.CLK_CTRL.PHCFG_START.storage == 1)
                                                     | (self.lms7002.CLK_CTRL.PLLCFG_START.re & self.lms7002.CLK_CTRL.PLLCFG_START.storage == 1)
                                                     | (self.lms7002.CLK_CTRL.PLLRST_START.re & self.lms7002.CLK_CTRL.PLLRST_START.storage == 1) )
        # self.ev.irq0 = EventSourceProcess(edge="rising")
        # self.ev.irq1 = EventSourceProcess(edge="rising")

        #
        # # Generate irq0 every 5 seconds.
        # self.irq0_timer = WaitTimer(5.0*sys_clk_freq)
        # self.comb += self.irq0_timer.wait.eq(~self.irq0_timer.done)
        # self.comb += self.ev.irq0.trigger.eq(self.irq0_timer.done)
        #
        # # Generate irq1 every 5 seconds.
        # self.irq1_timer = WaitTimer(5.0*sys_clk_freq)
        # self.comb += self.irq1_timer.wait.eq(~self.irq1_timer.done)
        # self.comb += self.ev.irq1.trigger.eq(self.irq1_timer.done)
