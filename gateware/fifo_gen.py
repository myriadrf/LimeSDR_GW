#!/usr/bin/env python3

#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import argparse

from migen import *

from litex.gen import *

from litex.build import tools
from litex.build.generic_platform import *
from litex.build.lattice import LatticePlatform

from litex.soc.interconnect import stream


# FIXME/CHECKME:
# - FIFO/CDC/Converter order depending use-case.
# - empty flag generation (currently ~source.valid).
# - rd_en latency.
# - converter order (--reverse).
# - rd_cnt generated in wr clok domain -> See how it's used.

# IOs/Interfaces -----------------------------------------------------------------------------------

def get_ios(input_width, output_width):
    return [
        # Write IOs.
        ("wr_clk",  0, Pins(1)),
        ("wr_rst",  0, Pins(1)),
        ("wr_en",   0, Pins(1)),
        ("wr_data", 0, Pins(input_width)),

        # Read IOs.
        ("rd_clk",  0, Pins(1)),
        ("rd_rst",  0, Pins(1)),
        ("rd_en",   0, Pins(1)),
        ("rd_data", 0, Pins(output_width)),

        # Status.
        ("wr_cnt", 0, Pins(32)), # 32-bit to ensure it's large enough for all cases.
        ("rd_cnt", 0, Pins(32)), # 32-bit to ensure it's large enough for all cases.
        ("empty",  0, Pins(1)),
        ("full",   0, Pins(1)),
    ]

# FIFOGenerator ------------------------------------------------------------------------------------

class FIFOGenerator(LiteXModule):
    def __init__(self, platform, input_width=128, output_width=64, depth=32, reverse=False):
        self.sink   = sink   = stream.Endpoint([("data",  input_width)])
        self.source = source = stream.Endpoint([("data", output_width)])

        # # #
        # Clocking ---------------------------------------------------------------------------------
        platform.add_extension(get_ios(input_width, output_width))
        self.clock_domains.cd_wr = ClockDomain("wr")
        self.clock_domains.cd_rd = ClockDomain("rd")
        self.comb += [
            self.cd_wr.clk.eq(platform.request("wr_clk")),
            self.cd_wr.rst.eq(platform.request("wr_rst")),
            self.cd_rd.clk.eq(platform.request("rd_clk")),
            self.cd_rd.rst.eq(platform.request("rd_rst")),
        ]

        # FIFO -------------------------------------------------------------------------------------

        # FIFO.
        # -----
        self.fifo = ClockDomainsRenamer("wr")(stream.SyncFIFO([("data", input_width)], depth))

        # CDC.
        # ----
        self.cdc = stream.ClockDomainCrossing([("data", input_width)],
            cd_from         = "wr",
            cd_to           = "rd",
            with_common_rst = True,
        )

        # Converter.
        # ----------
        self.conv = ClockDomainsRenamer("rd")(stream.Converter(input_width, output_width, reverse=reverse))

        # Pipeline.
        # ---------
        self.pipeline  = stream.Pipeline(
            self.sink,
            self.fifo,
            self.cdc,
            self.conv, # CHECKME: Order? Should we put it after of before the FIFO?
            self.source,
        )

        # Interface --------------------------------------------------------------------------------

        self.comb += [
            # Write.
            sink.valid.eq(platform.request("wr_en")),
            sink.data.eq(platform.request("wr_data")),

            # Read.
            source.ready.eq(platform.request("rd_en")),  # CHECKME: Latency?.
            platform.request("rd_data").eq(source.data),

            # Status.
            platform.request("full").eq(~sink.ready),
            platform.request("empty").eq(~source.valid), # CHECKME...
            platform.request("wr_cnt").eq(self.fifo.level),
            platform.request("rd_cnt").eq(self.fifo.level), # FIXME: CDC! if used from rd clk domain.
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="FIFO Generator")
    parser.add_argument("--input-width",   default=128, help="Input data width  (default=128).")
    parser.add_argument("--output-width",  default=64,  help="Output data width (default=64).")
    parser.add_argument("--depth",         default=32,  help="Depth (default=32).")
    parser.add_argument("--reverse",       action="store_true", help="Reverse converter ordering.")
    parser.add_argument("--build",         action="store_true", help="Build core")
    args = parser.parse_args()

    # Generate core --------------------------------------------------------------------------------
    input_width  = int(args.input_width)
    output_width = int(args.output_width)
    depth        = int(args.depth)
    platform   = LatticePlatform("", io=[], toolchain="diamond")
    module     = FIFOGenerator(platform,
        input_width  = input_width,
        output_width = output_width,
        depth        = depth,
        reverse      = args.reverse)
    build_name = "fifo_w{}x{}_r{}".format(input_width, depth, output_width)
    if args.build:
        platform.build(module, build_name=build_name, run=False, regular_comb=True)

if __name__ == "__main__":
    main()
