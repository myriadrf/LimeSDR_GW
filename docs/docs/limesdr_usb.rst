LimeSDR USB
===========

This section provides detailed information about the gateware implemented for the LimeSDR USB board.

Gateware Register Reference
----------------------------
LimeSDR USB exposes registers through two access paths:

- :doc:`Legacy FPGA SPI registers <limesdr-usb/reg_remap/usb_regremap_from_csv>`: legacy host registers used by existing software and previous gateware; planned to be replaced by LiteX CSR.
- :doc:`Native LiteX CSR map <limesdr-usb/litex_doc/index>`: the SoC's dedicated CSR register space generated from LiteX modules.

During the migration phase, the host can continue accessing legacy FPGA SPI register addresses; firmware remaps these FPGA SPI register accesses to native LiteX CSR registers internally. The LiteX CSR map is the forward path for native SoC register access.

.. toctree::
   :maxdepth: 3
   :hidden:

   Legacy FPGA SPI register reference <limesdr-usb/reg_remap/usb_regremap_from_csv>
   Register reference <limesdr-usb/litex_doc/index>
