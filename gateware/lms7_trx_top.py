#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems
#
# SPDX-License-Identifier: BSD-2-Clause

import os
import math
from shutil import which, copyfile
import subprocess

from migen import *

from litex.build import tools

from litex.gen import *

from gateware.common import *

from gateware.lms7_trx_files_list import lms7_trx_files, lms7_trx_ips
from gateware.lms7002_top         import DelayControl, SampleCompare
from gateware.general_periph      import ToPeriphCfg, FromPeriphCfg

class LMS7TRXTopWrapper(LiteXModule):
    def __init__(self, platform, lms_pads=None,
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

        assert lms_pads is not None

        self.ft_clk   = Signal()
        self.reset_n  = Signal()

        self.HW_VER   = Signal(4)
        self.BOM_VER  = Signal(3)

        # CFG TOP SPI Interface.
        # ----------------------
        self.cfg_top_mosi = Signal()
        self.cfg_top_sclk = Signal()
        self.cfg_top_ss_n = Signal()
        self.cfg_top_miso = Signal()

        # FT601 FIFO enpoint/ctrl PC<-> FPGA.
        # -----------------------------------
        self.ctrl_fifo   = FIFOInterface(CTRL0_FPGA_RX_RWIDTH, FTDI_DQ_WIDTH)

        # FT601 FIFO enpoint/ctrl Reset.
        # ------------------------------
        self.ctrl_fifo_fpga_pc_reset_n   = Signal()
        self.stream_fifo_pc_fpga_reset_n = Signal()

        # LMS7002 Top Module connections.
        # -------------------------------
        self.delay_control = DelayControl()

        # RXTX.
        # -----
        self.rxtx_smpl_cmp_length = Signal(16)
        self.from_fpgacfg         = FromFPGACfg()
        self.to_tstcfg_from_rxtx  = ToTstCfgFromRXTX()
        self.from_tstcfg          = FromTstCfg()

        # # #

        # Signals.
        # --------
        fpga_cfg_spi_pads = platform.request("FPGA_CFG_SPI")

        # LMS6 TRX TOP.
        # -------------------------
        self.specials += Instance("lms7_trx_top",
            # General parameters
            #p_BOARD                = "LimeSDR-Mini",
            #p_DEV_FAMILY           = "MAX 10",
            # LMS7002 related
            #p_LMS_DIQ_WIDTH        = 12,
            # FTDI (USB3) related
            p_FTDI_DQ_WIDTH        = FTDI_DQ_WIDTH,        # FTDI Data bus size
            p_CTRL0_FPGA_RX_SIZE   = CTRL0_FPGA_RX_SIZE,   # Control PC->FPGA, FIFO size in bytes.
            p_CTRL0_FPGA_RX_RWIDTH = CTRL0_FPGA_RX_RWIDTH, # Control PC->FPGA, FIFO rd width.
            p_CTRL0_FPGA_TX_SIZE   = CTRL0_FPGA_TX_SIZE,   # Control FPGA->PC, FIFO size in bytes
            p_CTRL0_FPGA_TX_WWIDTH = CTRL0_FPGA_TX_WWIDTH, # Control FPGA->PC, FIFO wr width
            # Internal configuration memory
            p_FPGACFG_START_ADDR   = 0,
            p_PLLCFG_START_ADDR    = 32,
            p_TSTCFG_START_ADDR    = 96,
            p_PERIPHCFG_START_ADDR = 192,

            # ----------------------------------------------------------------------------
            # External GND pin for reset
            # EXT_GND           : in     std_logic;
            # Global reset
            i_reset_n               = ~ResetSignal("sys"),

            # ----------------------------------------------------------------------------
            # Clock sources
            #    Reference clock, coming from LMK clock buffer.
            i_osc_clk               = ClockSignal("sys"),

            # ----------------------------------------------------------------------------
            # CFG Top SPI Interface
            i_CFG_TOP_MOSI          = self.cfg_top_mosi,
            i_CFG_TOP_SCLK          = self.cfg_top_sclk,
            i_CFG_TOP_SS_n          = self.cfg_top_ss_n,
            o_CFG_TOP_MISO          = self.cfg_top_miso,

            # ----------------------------------------------------------------------------
            # LMS7002 Digital
            #   MISC
            o_lms_delay_en          = self.delay_control.en,
            o_lms_delay_sel         = self.delay_control.sel,
            o_lms_delay_dir         = self.delay_control.dir,
            o_lms_delay_mode        = self.delay_control.mode,
            i_lms_delay_done        = self.delay_control.done,
            i_lms_delay_error       = self.delay_control.error,

            # ----------------------------------------------------------------------------
            #   FTDI (USB3)
            #     Clock source

            # FIFOs Reset
            o_EP82_aclrn_o   = self.ctrl_fifo_fpga_pc_reset_n,
            o_EP03_aclrn_o   = self.stream_fifo_pc_fpga_reset_n,

            # controll endpoint fifo PC->FPGA
            o_EP02_rd_src    = self.ctrl_fifo.rd,
            i_EP02_rdata_src = self.ctrl_fifo.rdata,
            i_EP02_rempty_src = self.ctrl_fifo.empty,

            # controll endpoint fifo FPGA->PC
            o_EP82_wr_src    = self.ctrl_fifo.wr,
            o_EP82_wdata_src = self.ctrl_fifo.wdata,
            i_EP82_wfull_src = self.ctrl_fifo.full,

            # ----------------------------------------------------------------------------
            #  External communication interfaces
            #   FPGA_SPI
            o_FPGA_SPI_SCLK         = Open(),      #fpga_spi_pads.SCLK,
            o_FPGA_SPI_MOSI         = Open(),      #fpga_spi_pads.MOSI,
            i_FPGA_SPI_MISO         = Constant(0), #fpga_spi_pads.MISO,
            o_FPGA_SPI_LMS_SS       = Open(),      #fpga_spi_pads.LMS_SS,
            o_FPGA_SPI_DAC_SS       = Open(),      #fpga_spi_pads.DAC_SS,
            #  FPGA_CFG
            i_FPGA_CFG_SPI_MISO     = fpga_cfg_spi_pads.MISO,
            o_FPGA_CFG_SPI_MOSI     = fpga_cfg_spi_pads.MOSI,
            #FPGA_CFG_SPI_SCLK : out    std_logic; -- SCLK pin can be accessed only trough USRMCLK component
            o_FPGA_CFG_SPI_SS_N     = fpga_cfg_spi_pads.SS_N,
            #  FPGA I2C
            io_FPGA_I2C_SCL         = Open(),#fpga_i2c_pads.SCL,
            io_FPGA_I2C_SDA         = Open(),#fpga_i2c_pads.SDA,

            # RXTX Top
            o_rxtx_smpl_cmp_length  = self.rxtx_smpl_cmp_length,
            # from_fpgacfg
            o_phase_reg_sel          = self.from_fpgacfg.phase_reg_sel,
            o_clk_ind                = self.from_fpgacfg.clk_ind,
            o_cnt_ind                = self.from_fpgacfg.cnt_ind,
            o_load_phase_reg         = self.from_fpgacfg.load_phase_reg,
            o_drct_clk_en            = self.from_fpgacfg.drct_clk_en,
            o_ch_en                  = self.from_fpgacfg.ch_en,
            o_smpl_width             = self.from_fpgacfg.smpl_width,
            o_mode                   = self.from_fpgacfg.mode,
            o_ddr_en                 = self.from_fpgacfg.ddr_en,
            o_trxiq_pulse            = self.from_fpgacfg.trxiq_pulse,
            o_mimo_int_en            = self.from_fpgacfg.mimo_int_en,
            o_synch_dis              = self.from_fpgacfg.synch_dis,
            o_synch_mode             = self.from_fpgacfg.synch_mode,
            o_smpl_nr_clr            = self.from_fpgacfg.smpl_nr_clr,
            o_txpct_loss_clr         = self.from_fpgacfg.txpct_loss_clr,
            o_rx_en                  = self.from_fpgacfg.rx_en,
            o_tx_en                  = self.from_fpgacfg.tx_en,
            o_rx_ptrn_en             = self.from_fpgacfg.rx_ptrn_en,
            o_tx_ptrn_en             = self.from_fpgacfg.tx_ptrn_en,
            o_tx_cnt_en              = self.from_fpgacfg.tx_cnt_en,
            o_wfm_ch_en              = self.from_fpgacfg.wfm_ch_en,
            o_wfm_play               = self.from_fpgacfg.wfm_play,
            o_wfm_load               = self.from_fpgacfg.wfm_load,
            o_wfm_smpl_width         = self.from_fpgacfg.wfm_smpl_width,
            o_SPI_SS                 = self.from_fpgacfg.SPI_SS,
            o_LMS1_SS                = self.from_fpgacfg.LMS1_SS,
            o_LMS1_RESET             = self.from_fpgacfg.LMS1_RESET,
            o_LMS1_CORE_LDO_EN       = self.from_fpgacfg.LMS1_CORE_LDO_EN,
            o_LMS1_TXNRX1            = self.from_fpgacfg.LMS1_TXNRX1,
            o_LMS1_TXNRX2            = self.from_fpgacfg.LMS1_TXNRX2,
            o_LMS1_TXEN              = self.from_fpgacfg.LMS1_TXEN,
            o_LMS1_RXEN              = self.from_fpgacfg.LMS1_RXEN,
            o_GPIO                   = self.from_fpgacfg.GPIO,
            o_FPGA_LED1_CTRL         = self.from_fpgacfg.FPGA_LED1_CTRL,
            o_FPGA_LED2_CTRL         = self.from_fpgacfg.FPGA_LED2_CTRL,
            o_FX3_LED_CTRL           = self.from_fpgacfg.FX3_LED_CTRL,
            o_CLK_ENA                = self.from_fpgacfg.CLK_ENA,
            o_sync_pulse_period      = self.from_fpgacfg.sync_pulse_period,
            o_sync_size              = self.from_fpgacfg.sync_size,
            o_txant_pre              = self.from_fpgacfg.txant_pre,
            o_txant_post             = self.from_fpgacfg.txant_post,
            # to_tst_cfg_from_rxtx
            i_DDR2_1_STATUS          = self.to_tstcfg_from_rxtx.DDR2_1_STATUS,
            i_DDR2_1_pnf_per_bit     = self.to_tstcfg_from_rxtx.DDR2_1_pnf_per_bit,
            # from_tstcfg
            o_TEST_EN                = self.from_tstcfg.TEST_EN,
            o_TEST_FRC_ERR           = self.from_tstcfg.TEST_FRC_ERR,
            o_TX_TST_I               = self.from_tstcfg.TX_TST_I,
            o_TX_TST_Q               = self.from_tstcfg.TX_TST_Q,

            #  Bill Of material and hardware version
            i_BOM_VER               = self.BOM_VER,
            i_HW_VER                = self.HW_VER,
        )


        #platform.add_period_constraint(lms_pads.MCLK1, 1e9/125e6)
        #platform.add_period_constraint(lms_pads.MCLK2, 1e9/125e6)

        platform.verilog_include_paths.append("LimeSDR-Mini_lms7_trx/mico32_patform/platform1/soc")

        platform.toolchain.additional_ldf_commands += [
            "prj_strgy set_value -strategy Strategy1 syn_fix_gated_and_generated_clks=False",
            "prj_strgy set_value -strategy Strategy1 syn_default_enum_encode=Onehot",
            "prj_strgy set_value -strategy Strategy1 syn_export_setting=Yes",
            "prj_strgy set_value -strategy Strategy1 syn_frequency=100",
            "prj_strgy set_value -strategy Strategy1 syn_critical_path_num=3",
            "prj_strgy set_value -strategy Strategy1 syn_pipelining_retiming=None",
            "prj_strgy set_value -strategy Strategy1 syn_push_tristates=False",
            "prj_strgy set_value -strategy Strategy1 syn_res_sharing=False",
            "prj_strgy set_value -strategy Strategy1 syn_vhdl2008=True",
            "prj_strgy set_value -strategy Strategy1 lse_vhdl2008=True",
            "prj_strgy set_value -strategy Strategy1 lse_frequency=100",
            "prj_strgy set_value -strategy Strategy1 lse_force_gsr=No",
            "prj_strgy set_value -strategy Strategy1 lse_fix_gated_clocks=False",
            "prj_strgy set_value -strategy Strategy1 lse_res_sharing=False",
            "prj_strgy set_value -strategy Strategy1 par_routeing_pass=10",
            "prj_strgy set_value -strategy Strategy1 tmchk_enable_check=False",
        ]

        #mico2_sw = os.path.abspath("LimeSDR-Mini_lms7_trx/mico32_sw/lms7_trx/lms7_trx.mem")
        #eco_cmd = "eco_config memebr -instance {lms7_trx_top/inst0_cpu/inst_cpu/lm32_inst/ebr/genblk1.ram} -init_all no -mem {" + mico2_sw + "} -format hex -init_data static -module {pmi_ram_dpEhnonessen3213819232138192p13822039} -mode {RAM_DP} -depth {8192} -widtha {32} -widthb {32} "
        #platform.toolchain.post_export_commands += [
        #    eco_cmd,
        #    "prj_project save",
        #    "prj_run Export -impl impl -task Bitgen -forceOne",
        #]

        self.add_sources(platform)

    def add_sources(self, platform):
        for file in lms7_trx_files:
            platform.add_source(file)
        platform.add_source("gateware/lms7_trx_top.vhd")
        for file in lms7_trx_ips:
            platform.add_ip(file)
        platform.add_strategy("LimeSDR-Mini_lms7_trx/proj/user_timing.sty", "user_timing")

