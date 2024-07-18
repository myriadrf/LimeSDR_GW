#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *


class lms7002_top(Module):
    def __init__(self, platform, lms7002_pads, vendor="XILINX", dev_family="Artix 7", iq_width=12,
                 s_axis_tx_fifo_words=16, m_axis_rx_fifo_words=16, m_clk_domain="sys", s_clk_domain="sys"):
        # Add CSRs
        self.control = CSRStorage(fields=[
            CSRField(name="TX_EN", size=1, offset=0, values=[
                ("``0b0``", "RX/TX Disabled"),
                ("``0b1``", "RX/TX Enabled")
            ], reset=0b0),
            CSRField(name="TRXIQ_PULSE", size=1, offset=1, values=[
                ("``0b0``", "TRXIQ_pulse mode disabled"),
                ("``0b1``", "TRXIQ_pulse mode enabled")
            ], reset=0b0),
            CSRField(name="DDR_EN", size=1, offset=2, values=[
                ("``0b0``", "SDR mode"),
                ("``0b1``", "DDR mode")
            ], reset=0b1),
            CSRField(name="MIMO_INT_EN", size=1, offset=3, values=[
                ("``0b0``", "MIMO disabled"),
                ("``0b1``", "MIMO enabled")
            ], reset=0b1),
            CSRField(name="CH_EN", size=2, offset=4, values=[
                ("``0b01``", "Channel A enabled"),
                ("``0b10``", "Channel B enabled"),
                ("``0b11``", "Channels A and B enabled")
            ], reset=0b00),
            CSRField(name="LMS1_TXEN", size=1, offset=6, values=[
                ("``0b0``", "LMS_TX disabled"),
                ("``0b1``", "LMS_TX enabled")
            ], reset=0b1),
            CSRField(name="LMS_TXRXEN_MUX_SEL", size=1, offset=7, values=[
                ("''0b0''", "LMS TX/RXEN signal control by TDD signal disabled"),
                ("``0b1``", "LMS TX/RXEN signal control by TDD signal enabled")
            ], reset=0b0),
            CSRField(name="LMS1_RXEN", size=1, offset=8, values=[
                ("``0b0``", "LMS_RX disabled"),
                ("``0b1``", "LMS_TX enabled")
            ], reset=0b1),
            CSRField(name="LMS1_RESET", size=1, offset=9, values=[
                ("``0b0``", "Reset active"),
                ("``0b1``", "Reset inactive")
            ], reset=0b1),
            CSRField(name="LMS_TXRXEN_INV", size=1, offset=10, values=[
                ("``0b0``", "Do not invert TX/RXEN signals"),
                ("``0b1``", "Invert TX/RXEN signals")
            ], reset=0b0),
            CSRField(name="LMS1_CORE_LDO_EN", size=1, offset=11, values=[
                ("``0b0``", "LMS LDO Disabled"),
                ("``0b1``", "LMS LDO Enabled")
            ], reset=0b1),
            CSRField(name="LMS1_TXNRX1", size=1, offset=12, values=[
                ("``0b0``", "Port 1 TXIQ"),
                ("``0b1``", "Port 1 RXIQ")
            ], reset=0b1),
            CSRField(name="LMS1_TXNRX2", size=1, offset=13, values=[
                ("``0b0``", "Port 2 TXIQ"),
                ("``0b1``", "Port 2 RXIQ")
            ], reset=0b0),
        ])

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
        self.axis_m = AXIStreamInterface(axis_datawidth, layout=axis_layout, clock_domain=m_clk_domain)
        self.axis_s = AXIStreamInterface(axis_datawidth, layout=axis_layout, clock_domain=s_clk_domain)

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
            i_MCLK1=lms7002_pads.mclk1,
            o_FCLK1=lms7002_pads.fclk1,
            o_DIQ1=lms7002_pads.diq1,
            o_ENABLE_IQSEL1=lms7002_pads.iqsel1,
            o_TXNRX1=lms7002_pads.txnrx1,
            # DIQ2
            i_MCLK2=lms7002_pads.mclk2,
            o_FCLK2=lms7002_pads.fclk2,
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
            i_m_axis_rx_tready=self.axis_m.ready,
            o_m_axis_rx_tlast=self.axis_m.last,
            # misc
            o_TX_ACTIVE=self.TX_ACTIVE,
            o_RX_ACTIVE=self.RX_ACTIVE,
            # interface cfg
            i_CFG_TX_EN=self.control.fields.TX_EN,
            i_CFG_TRXIQ_PULSE=self.control.fields.TRXIQ_PULSE,
            i_CFG_DDR_EN=self.control.fields.DDR_EN,
            i_CFG_MIMO_INT_EN=self.control.fields.MIMO_INT_EN,
            i_CFG_CH_EN=self.control.fields.CH_EN,
            i_CFG_LMS_TXEN=self.control.fields.LMS1_TXEN,
            i_CFG_LMS_TXRXEN_MUX_SEL=self.control.fields.LMS_TXRXEN_MUX_SEL,
            i_CFG_LMS_RXEN=self.control.fields.LMS1_RXEN,
            i_CFG_LMS_RESET=self.control.fields.LMS1_RESET,
            i_CFG_LMS_TXRXEN_INV=self.control.fields.LMS_TXRXEN_INV,
            i_CFG_LMS_CORE_LDO_EN=self.control.fields.LMS1_CORE_LDO_EN,
            i_CFG_LMS_TXNRX1=self.control.fields.LMS1_TXNRX1,
            i_CFG_LMS_TXNRX2=self.control.fields.LMS1_TXNRX2

        )

        # Create instance and assign params
        self.specials += Instance("lms7002_top", **self.params_ios)
