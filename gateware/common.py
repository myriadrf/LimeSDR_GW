#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from migen import *

from litex.gen import *

from litex.build.vhd2v_converter import VHD2VConverter

# Configuration Layouts ----------------------------------------------------------------------------

from_fpgacfg_layout = [
    ("phase_reg_sel",     16),
    ("clk_ind",            5),
    ("cnt_ind",            5),
    ("load_phase_reg",     1),
    ("drct_clk_en",       16),
    ("wfm_ch_en",         16),
    ("wfm_smpl_width",     2),
    ("spi_ss",            16),
    ("gpio",              16),
    ("fpga_led1_ctrl",     3),
    ("fpga_led2_ctrl",     3),
    ("fx3_led_ctrl",       3),
    ("clk_ena",            4),
    ("sync_pulse_period", 32),
]

def FromFPGACfg():
    return Record(from_fpgacfg_layout)

# VHD2VConverter Wrapper ---------------------------------------------------------------------------
def add_vhd2v_converter(platform, instance, files=[], force_convert=None):
    force_convert = {True: LiteXContext.platform.vhd2v_force, False: force_convert}[force_convert is None]
    return VHD2VConverter(platform,
        instance      = instance,
        work_package  = "work",
        force_convert = force_convert,
        add_instance  = True,
        files         = files,
    )
