# SPDX-License-Identifier: Apache-2.0
from gateware.fpga_host_descriptor_table import FPGAHostDescriptorTable


class FPGAHostTXDescriptorTable(FPGAHostDescriptorTable):
    """TX table with loading, diagnostic-fetch or fully buffered timed mode.

    Uses the RX validation/copy/queue algorithm and identical CSR offsets.
    Capability bit 31 prohibits execution; bit 30 identifies diagnostic fetch.
    Bit 29 identifies timed mode, whose appended CSRs describe each parent.
    """
    def __init__(self, dma_base, load_only=True, timed=False, streaming=False, **kwargs):
        super().__init__(dma_base, load_only=load_only,
            tx_streaming=streaming,
            capability_flags=((1 << 29) | (int(streaming) << 28)) if timed else (0 if load_only else 1 << 30), **kwargs)
