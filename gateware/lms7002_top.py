#!/usr/bin/env python3

from migen import *

from litex.gen import *

from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *
from litex.soc.cores.clock     import *

from litescope import LiteScopeAnalyzer


class lms7002_top(LiteXModule):
    def __init__(self, platform, lms7002_pads, vendor="XILINX", dev_family="Artix 7", iq_width=12,
                 s_axis_tx_fifo_words=16, m_axis_rx_fifo_words=16, m_clk_domain="sys", s_clk_domain="sys"):
        # Add CSRs
        self.control = CSRStorage(fields=[
            CSRField(name="LMS1_TXNRX1", size=1, offset=12, values=[
                ("``0b0``", "Port 1 TXIQ"),
                ("``0b1``", "Port 1 RXIQ")
            ], reset=0b1),
            CSRField(name="LMS1_TXNRX2", size=1, offset=13, values=[
                ("``0b0``", "Port 2 TXIQ"),
                ("``0b1``", "Port 2 RXIQ")
            ], reset=0b0),
        ])

        self.tx_en = CSRStorage(1,
            description="TX Enable: 0: Disabled, 1: Enabled."
        )
        self.rx_en = CSRStorage(1,
            description="RX Enable: 0: Disabled, 1: Enabled."
        )
        self.trxiq_pulse = CSRStorage(1,
            description="TRXIQ_PULSE mode Enable: 0: Disabled, 1: Enabled."
        )
        self.ddr_en = CSRStorage(1, reset=1,
            description="DDR mode enable: 0: Disabled, 1: Enabled."
        )
        self.mimo_int_en = CSRStorage(1, reset=1,
            description="MIMO mode: 0: Disabled, 1: Enabled."
        )
        self.ch_en = CSRStorage(2, reset=3,
            description="01 - Channel A enabled, 10 - Channel B enabled, 11 - Channels A and B enabled"
        )


        self.lms1_txen = CSRStorage(1, reset=1,
            description="LMS1 TX Enable: 0: Disabled, 1: Enabled."
        )
        self.lms1_rxen = CSRStorage(1, reset=1,
            description="LMS1 TX Enable: 0: Disabled, 1: Enabled."
        )
        self.lms1_txrxen_mux_sel = CSRStorage(1, reset=0,
            description="LMS1 TX Enable: 0: Disabled, 1: Enabled."
        )
        self.lms1_txrxen_inv = CSRStorage(1, reset=0,
            description="LMS1 TX Enable: 0: Disabled, 1: Enabled."
        )
        self.lms1_resetn = CSRStorage(1, reset=1,
            description="LMS1 Reset: 0: Reset active, 1: Reset inactive"
        )
        self.lms1_core_ldo_en = CSRStorage(1, reset=0,
            description="LMS1 internal LDO enable: 0: Disabled, 1: Enabled"
        )
        self.lms1_txnrx1 = CSRStorage(1, reset=1,
            description="LMS1 port1 mode: 0: Port 1 TXIQ, 1: Port 1 RXIQ"
        )
        self.lms2_txnrx2 = CSRStorage(1, reset=0,
            description="LMS1 port2 mode: 0: Port 2 TXIQ, 1: Port 2 RXIQ"
        )

        self.txpll_reset = CSRStorage(1, reset=0,
            description="TX PLL reset: 0: Reset is not active, 1: Reset is Active"
        )

        self.rxpll_reset = CSRStorage(1, reset=0,
            description="TX PLL reset: 0: Reset is not active, 1: Reset is Active"
        )

        self.txpll_status = CSRStatus(1, reset=0,
            description="TX PLL lock status: 0: NO Lock, 1: Locked"
        )

        self.rxpll_status = CSRStatus(1, reset=0,
            description="RX PLL lock status: 0: NO Lock, 1: Locked"
        )





        # Add sources
        platform.add_source("./gateware/LimeDFB/lms7002/src/lms7002_top.vhd")
        platform.add_source("./gateware/LimeDFB/lms7002/src/lms7002_tx.vhd")
        platform.add_source("./gateware/LimeDFB/lms7002/src/lms7002_rx.vhd")
        platform.add_source("./gateware/LimeDFB/lms7002/src/lms7002_ddout.vhd")
        platform.add_source("./gateware/LimeDFB/lms7002/src/lms7002_ddin.vhd")

        platform.add_source("./gateware/LimeDFB/fifo_axis/src/fifo_axis_wrap.vhd")
        platform.add_source("./gateware/LimeDFB/lms7002/src/rx_pll/rx_pll.xci")
        # create misc signals
        self.TX_ACTIVE = Signal()
        self.RX_ACTIVE = Signal()

        # Create streams
        axis_datawidth = 64
        axis_layout = [("data", max(1, axis_datawidth))]
        # adding reset along with data, assuming resets are not global
        axis_layout += [("areset_n", 1)]
        axis_layout += [("keep", max(1, axis_datawidth))]
        self.axis_m = AXIStreamInterface(axis_datawidth, layout=axis_layout, clock_domain=m_clk_domain)
        self.axis_s = AXIStreamInterface(axis_datawidth, layout=axis_layout, clock_domain=s_clk_domain)

        # TX PLL.
        self.cd_txpll_c0 = ClockDomain()
        self.cd_txpll_c1 = ClockDomain()
        self.txpll = txpll = S7PLL(speedgrade=-1)
        self.comb += txpll.reset.eq(self.txpll_reset.storage)
        txpll.register_clkin(lms7002_pads.mclk1, 30.72e6)
        txpll.create_clkout(self.cd_txpll_c0,    30.72e6)
        txpll.create_clkout(self.cd_txpll_c1, 30.72e6, 190)

        self.comb += self.txpll_status.status.eq(txpll.locked)

        # RX PLL.
        self.cd_rxpll_c0 = ClockDomain()
        self.cd_rxpll_c1 = ClockDomain()
        self.rxpll = rxpll = S7PLL(speedgrade=-1)
        self.comb += rxpll.reset.eq(self.rxpll_reset.storage)
        rxpll.register_clkin(lms7002_pads.mclk2, 30.72e6)
        rxpll.create_clkout(self.cd_rxpll_c0,    30.72e6)
        rxpll.create_clkout(self.cd_rxpll_c1, 30.72e6, 120)
        self.comb += self.rxpll_status.status.eq(rxpll.locked)

        self.comb += lms7002_pads.fclk2.eq(ClockSignal("rxpll_c0"))


        # Create params
        self.params_ios = dict()

        # Assign generics
        self.params_ios.update(
            p_g_VENDOR=vendor,
            p_g_DEV_FAMILY=dev_family,
            p_g_IQ_WIDTH=iq_width,
            p_g_S_AXIS_TX_FIFO_WORDS=s_axis_tx_fifo_words,
            p_g_M_AXIS_RX_FIFO_WORDS=m_axis_rx_fifo_words
        )

        # Assign ports
        self.params_ios.update(
            # DIQ1
            i_MCLK1=ClockSignal("txpll_c1"),
            #o_FCLK1=lms7002_pads.fclk1,
            o_DIQ1=lms7002_pads.diq1,
            o_ENABLE_IQSEL1=lms7002_pads.iqsel1,
            o_TXNRX1=lms7002_pads.txnrx1,
            # DIQ2
            i_MCLK2=ClockSignal("rxpll_c1"),
            #o_FCLK2=lms7002_pads.fclk2,
            i_DIQ2=lms7002_pads.diq2,
            i_ENABLE_IQSEL2=lms7002_pads.iqsel2,
            o_TXNRX2=lms7002_pads.txnrx2,
            # Misc LMS
            o_RESET=lms7002_pads.rst_n,
            o_TXEN=lms7002_pads.txen,
            o_RXEN=lms7002_pads.rxen,
            o_CORE_LDO_EN=lms7002_pads.pwrdwn_n,
            # axis_s
            i_s_axis_tx_areset_n=self.axis_s.areset_n,
            i_s_axis_tx_aclk=ClockSignal(s_clk_domain),
            i_s_axis_tx_tvalid=self.axis_s.valid,
            i_s_axis_tx_tdata=self.axis_s.data,
            o_s_axis_tx_tready=self.axis_s.ready,
            i_s_axis_tx_tlast=self.axis_s.last,
            # axis_m
            i_m_axis_rx_areset_n=self.axis_m.areset_n,
            i_m_axis_rx_aclk=ClockSignal(m_clk_domain),
            o_m_axis_rx_tvalid=self.axis_m.valid,
            o_m_axis_rx_tdata=self.axis_m.data,
            o_m_axis_rx_tkeep=self.axis_m.keep,
            i_m_axis_rx_tready=self.axis_m.ready,
            o_m_axis_rx_tlast=self.axis_m.last,
            # misc
            o_TX_ACTIVE=self.TX_ACTIVE,
            o_RX_ACTIVE=self.RX_ACTIVE,
            # interface cfg
            i_CFG_TX_EN=self.tx_en.storage,
            i_CFG_TRXIQ_PULSE=self.trxiq_pulse.storage,
            i_CFG_DDR_EN=self.ddr_en.storage,
            i_CFG_MIMO_INT_EN=self.mimo_int_en.storage,
            i_CFG_CH_EN=self.ch_en.storage,
            i_CFG_LMS_TXEN=self.lms1_txen.storage,
            i_CFG_LMS_TXRXEN_MUX_SEL=self.lms1_txrxen_mux_sel.storage,
            i_CFG_LMS_RXEN=self.lms1_rxen.storage,
            i_CFG_LMS_RESET=self.lms1_resetn.storage,
            i_CFG_LMS_TXRXEN_INV=self.lms1_txrxen_inv.storage,
            i_CFG_LMS_CORE_LDO_EN=self.lms1_core_ldo_en.storage,
            i_CFG_LMS_TXNRX1=self.lms1_txnrx1.storage,
            i_CFG_LMS_TXNRX2=self.lms2_txnrx2.storage
        )

        # Create instance and assign params
        self.specials += Instance("lms7002_top", **self.params_ios)


        # LiteScope example.
        # ------------------
        # Setup LiteScope Analyzer to capture some of the AXI-Lite MMAP signals.
        analyzer_signals = [
            self.tx_en.storage,
            self.axis_m.areset_n,
            self.axis_m.valid,
            self.axis_m.data,
            self.axis_m.keep,
            self.axis_m.ready,
        ]

        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 512,
            clock_domain = m_clk_domain,
            register     = True,
            csr_csv      = "lime_top_lms7002_analyzer.csv"
        )
