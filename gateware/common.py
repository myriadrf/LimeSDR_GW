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

# VHD2VConverter Wrapper ---------------------------------------------------------------------------
def add_vhd2v_converter(platform, instance, files=[], force_convert=None, add_instance=True, top_entity=None, flatten_source=True):
    force_convert = {True: platform.vhd2v_force, False: force_convert}[force_convert is None]
    return VHD2VConverter(platform,
        instance       = instance,
        top_entity     = top_entity,
        work_package   = "work",
        flatten_source = flatten_source,
        force_convert  = force_convert,
        add_instance   = add_instance,
        files          = files,
    )


# Board ID's
from enum import IntEnum
class BoardIDs(IntEnum):
    LIME_SDR_USB = 0x000E
    LIME_SDR_XTRX = 27
    LIME_SDR_MINI = 0x0011
    SSDR = 32
    HIPER = 31