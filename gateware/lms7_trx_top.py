#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

import os
from shutil import which, copyfile
import subprocess

from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect import stream

from gateware.lms7_trx_files_list import lms7_trx_files, lms7_trx_ips

class LMS7TRXTopWrapper(LiteXModule):
    def __init__(self, platform,
        FTDI_DQ_WIDTH        = 32,    # FTDI Data bus size
        CTRL0_FPGA_RX_SIZE   = 1024,  # Control PC->FPGA, FIFO size in bytes.
        CTRL0_FPGA_RX_RWIDTH = 32,    # Control PC->FPGA, FIFO rd width.
        CTRL0_FPGA_TX_SIZE   = 1024,  # Control FPGA->PC, FIFO size in bytes
        CTRL0_FPGA_TX_WWIDTH = 32,    # Control FPGA->PC, FIFO wr width
        STRM0_FPGA_RX_SIZE   = 4096,  # Stream PC->FPGA, FIFO size in bytes
        STRM0_FPGA_RX_RWIDTH = 128,   # Stream PC->FPGA, rd width
        STRM0_FPGA_TX_SIZE   = 16384, # Stream FPGA->PC, FIFO size in bytes
        STRM0_FPGA_TX_WWIDTH = 64,    # Stream FPGA->PC, wr width
        C_EP02_RDUSEDW_WIDTH = 0,
        C_EP82_WRUSEDW_WIDTH = 0,
        C_EP03_RDUSEDW_WIDTH = 0,
        C_EP83_WRUSEDW_WIDTH = 0,
        ):

        self.platform = platform

        # FT601 FIFO enpoint/ctrl PC<-> FPGA
        self.ctrl_fifo_pc_fpga           = stream.Endpoint([("data", CTRL0_FPGA_RX_RWIDTH), ("empty", 1)])
        self.ctrl_fifo_fpga_pc           = stream.Endpoint([("data", CTRL0_FPGA_TX_WWIDTH), ("full", 1)])
        self.stream_fifo_pc_fpga         = stream.Endpoint([("data", STRM0_FPGA_RX_RWIDTH), ("active", 1), ("empty", 1), ("usedw", C_EP03_RDUSEDW_WIDTH)])
        self.stream_fifo_fpga_pc         = stream.Endpoint([("data", STRM0_FPGA_TX_WWIDTH), ("active", 1), ("full", 1),  ("usedw", C_EP83_WRUSEDW_WIDTH)])

        self.ctrl_fifo_fpga_pc_reset_n   = Signal()
        self.stream_fifo_fpga_pc_reset_n = Signal()
        self.stream_fifo_pc_fpga_reset_n = Signal()
        self.ft_clk                      = Signal()

        # # #

        # Signals
        # -------
        lms_pads          = platform.request("LMS")
        fpga_spi_pads     = platform.request("FPGA_SPI")
        fpga_cfg_spi_pads = platform.request("FPGA_CFG_SPI")
        fpga_i2c_pads     = platform.request("FPGA_I2C")
        rfsw_pads         = platform.request("RFSW")
        revision_pads     = platform.request("revision")
        tx_lb_pads        = platform.request("TX_LB")

        self.cd_osc       = ClockDomain()
        self.cd_lms_tx    = ClockDomain()
        self.cd_lms_rx    = ClockDomain()

        # LMS6 TRX TOP.
        # -------------------------

        self.specials += Instance("lms7_trx_top",
            # General parameters
            p_BOARD                = "LimeSDR-Mini",
            p_DEV_FAMILY           = "MAX 10",
            # LMS7002 related
            p_LMS_DIQ_WIDTH        = 12,
            # FTDI (USB3) related
            p_FTDI_DQ_WIDTH        = FTDI_DQ_WIDTH,        # FTDI Data bus size
            p_CTRL0_FPGA_RX_SIZE   = CTRL0_FPGA_RX_SIZE,   # Control PC->FPGA, FIFO size in bytes.
            p_CTRL0_FPGA_RX_RWIDTH = CTRL0_FPGA_RX_RWIDTH, # Control PC->FPGA, FIFO rd width.
            p_CTRL0_FPGA_TX_SIZE   = CTRL0_FPGA_TX_SIZE,   # Control FPGA->PC, FIFO size in bytes
            p_CTRL0_FPGA_TX_WWIDTH = CTRL0_FPGA_TX_WWIDTH, # Control FPGA->PC, FIFO wr width
            p_STRM0_FPGA_RX_SIZE   = STRM0_FPGA_RX_SIZE,   # Stream PC->FPGA, FIFO size in bytes
            p_STRM0_FPGA_RX_RWIDTH = STRM0_FPGA_RX_RWIDTH, # Stream PC->FPGA, rd width
            p_STRM0_FPGA_TX_SIZE   = STRM0_FPGA_TX_SIZE,   # Stream FPGA->PC, FIFO size in bytes
            p_STRM0_FPGA_TX_WWIDTH = STRM0_FPGA_TX_WWIDTH, # Stream FPGA->PC, wr width
            p_C_EP02_RDUSEDW_WIDTH = C_EP02_RDUSEDW_WIDTH,
            p_C_EP82_WRUSEDW_WIDTH = C_EP82_WRUSEDW_WIDTH,
            p_C_EP03_RDUSEDW_WIDTH = C_EP03_RDUSEDW_WIDTH,
            p_C_EP83_WRUSEDW_WIDTH = C_EP83_WRUSEDW_WIDTH,
            #
            p_TX_N_BUFF            = 4,      # N 4KB buffers in TX interface (2 OR 4)
            p_TX_PCT_SIZE          = 4096,   # TX packet size in bytes
            p_TX_IN_PCT_HDR_SIZE   = 16,
            # Internal configuration memory
            p_FPGACFG_START_ADDR   = 0,
            p_PLLCFG_START_ADDR    = 32,
            p_TSTCFG_START_ADDR    = 96,
            p_PERIPHCFG_START_ADDR = 192,

            # ----------------------------------------------------------------------------
            # External GND pin for reset
            # EXT_GND           : in     std_logic;

            # ----------------------------------------------------------------------------
            # Clock sources
            #    Reference clock, coming from LMK clock buffer.
            i_LMK_CLK               = ClockSignal("sys"),
            o_osc_clk_o             = self.cd_osc.clk,
            o_lms_tx_clk_o          = self.cd_lms_tx.clk,
            o_lms_rx_clk_o          = self.cd_lms_rx.clk,

            # ----------------------------------------------------------------------------
            # LMS7002 Digital
            #    PORT1
            i_LMS_MCLK1             = lms_pads.MCLK1,
            o_LMS_FCLK1             = lms_pads.FCLK1,
            o_LMS_TXNRX1            = lms_pads.TXNRX1,
            o_LMS_ENABLE_IQSEL1     = lms_pads.ENABLE_IQSEL1,
            o_LMS_DIQ1_D            = lms_pads.DIQ1_D,
            #   PORT2
            i_LMS_MCLK2             = lms_pads.MCLK2,
            o_LMS_FCLK2             = lms_pads.FCLK2,
            o_LMS_TXNRX2_or_CLK_SEL = lms_pads.TXNRX2_or_CLK_SEL, #In v2.3 board version this pin is changed to CLK_SEL
            i_LMS_ENABLE_IQSEL2     = lms_pads.ENABLE_IQSEL2,
            i_LMS_DIQ2_D            = lms_pads.DIQ2_D,
            #   MISC
            o_LMS_RESET             = lms_pads.RESET,
            o_LMS_TXEN              = lms_pads.TXEN,
            o_LMS_RXEN              = lms_pads.RXEN,
            o_LMS_CORE_LDO_EN       = lms_pads.CORE_LDO_EN,

            # ----------------------------------------------------------------------------
            #   FTDI (USB3)
            #     Clock source
            i_FT_CLK                 = self.ft_clk,
            # controll endpoint fifo PC->FPGA
            o_EP02_rd        = self.ctrl_fifo_pc_fpga.valid,
            i_EP02_rdata     = self.ctrl_fifo_pc_fpga.data,
            i_EP02_rempty    = self.ctrl_fifo_pc_fpga.empty,
            # controll endpoint fifo FPGA->PC
            o_EP82_aclrn     = self.ctrl_fifo_fpga_pc_reset_n,
            o_EP82_wr        = self.ctrl_fifo_fpga_pc.valid,
            o_EP82_wdata     = self.ctrl_fifo_fpga_pc.data,
            i_EP82_wfull     = self.ctrl_fifo_fpga_pc.full,
            # stream endpoint fifo PC->FPGA
            i_EP03_active    = self.stream_fifo_pc_fpga.active,
            o_EP03_aclrn     = self.stream_fifo_pc_fpga_reset_n,
            o_EP03_rd        = self.stream_fifo_pc_fpga.valid,
            i_EP03_rdata     = self.stream_fifo_pc_fpga.data,
            i_EP03_rempty    = self.stream_fifo_pc_fpga.empty,
            i_EP03_rusedw    = self.stream_fifo_pc_fpga.usedw,
            # stream endpoint fifo FPGA->PC
            i_EP83_active    = self.stream_fifo_fpga_pc.active,
            o_EP83_aclrn     = self.stream_fifo_fpga_pc_reset_n,
            o_EP83_wr        = self.stream_fifo_fpga_pc.valid,
            o_EP83_wdata     = self.stream_fifo_fpga_pc.data,
            i_EP83_wfull     = self.stream_fifo_fpga_pc.full,
            i_EP83_wrusedw   = self.stream_fifo_fpga_pc.usedw,

            # ----------------------------------------------------------------------------
            #  External communication interfaces
            #   FPGA_SPI
            o_FPGA_SPI_SCLK         = fpga_spi_pads.SCLK,
            o_FPGA_SPI_MOSI         = fpga_spi_pads.MOSI,
            i_FPGA_SPI_MISO         = fpga_spi_pads.MISO,
            o_FPGA_SPI_LMS_SS       = fpga_spi_pads.LMS_SS,
            o_FPGA_SPI_DAC_SS       = fpga_spi_pads.DAC_SS,
            #  FPGA_CFG
            i_FPGA_CFG_SPI_MISO     = fpga_cfg_spi_pads.MISO,
            o_FPGA_CFG_SPI_MOSI     = fpga_cfg_spi_pads.MOSI,
            #FPGA_CFG_SPI_SCLK : out    std_logic; -- SCLK pin can be accessed only trough USRMCLK component
            o_FPGA_CFG_SPI_SS_N     = fpga_cfg_spi_pads.SS_N,
            #  FPGA I2C
            io_FPGA_I2C_SCL         = fpga_i2c_pads.SCL,
            io_FPGA_I2C_SDA         = fpga_i2c_pads.SDA,

            # ----------------------------------------------------------------------------
            # General periphery
            #  LEDs
            o_FPGA_LED1_R           = platform.request("FPGA_LED1_R"),
            o_FPGA_LED1_G           = platform.request("FPGA_LED1_G"),
            #  GPIO
            io_FPGA_GPIO            = platform.request("FPGA_GPIO"),
            io_FPGA_EGPIO           = platform.request("FPGA_EGPIO"),
            #  Temperature sensor
            i_LM75_OS               = platform.request("LM75_OS"),
            #  Fan control
            o_FAN_CTRL              = platform.request("FAN_CTRL"),
            #  RF loop back control
            o_RFSW_RX_V1            = rfsw_pads.RX_V1,
            o_RFSW_RX_V2            = rfsw_pads.RX_V2,
            o_RFSW_TX_V1            = rfsw_pads.TX_V1,
            o_RFSW_TX_V2            = rfsw_pads.TX_V2,
            o_TX_LB_AT              = tx_lb_pads.AT,
            o_TX_LB_SH              = tx_lb_pads.SH,
            #  Bill Of material and hardware version
            i_BOM_VER               = revision_pads.BOM_VER,
            i_HW_VER                = revision_pads.HW_VER,
        )

        self.platform.verilog_include_paths.append("LimeSDR-Mini_lms7_trx/mico32_patform/platform1/soc")

        self.platform.toolchain.additional_ldf_commands += [
            "prj_strgy set_value -strategy Strategy1 syn_fix_gated_and_generated_clks=False",
            "prj_strgy set_value -strategy Strategy1 syn_default_enum_encode=Onehot",
            "prj_strgy set_value -strategy Strategy1 syn_export_setting=Yes",
            "prj_strgy set_value -strategy Strategy1 syn_frequency=100",
            "prj_strgy set_value -strategy Strategy1 syn_critical_path_num=3",
            "prj_strgy set_value -strategy Strategy1 syn_pipelining_retiming=None",
            "prj_strgy set_value -strategy Strategy1 syn_push_tristates=False",
            "prj_strgy set_value -strategy Strategy1 syn_res_sharing=False",
            "prj_strgy set_value -strategy Strategy1 syn_vhdl2008=True",
        ]

        self.add_sources()

    def add_sources(self):
        for file in lms7_trx_files:
            self.platform.add_source(file)
        self.platform.add_source("gateware/lms7_trx_top.vhd")
        for file in lms7_trx_ips:
            self.platform.add_ip(file)
        self.platform.add_strategy("LimeSDR-Mini_lms7_trx/proj/user_timing.sty", "user_timing")

