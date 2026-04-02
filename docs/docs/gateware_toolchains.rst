Gateware Toolchains
===================

This section offers a detailed overview of the FPGA synthesis toolchains used for supported boards
in the LimeSDR_GW project, ensuring reproducibility and helping new developers set up environments.

FPGA Synthesis Toolchains
-------------------------

Different synthesis tools are used based on the FPGA vendor and family:

+-------------------+------------------+-----------------------------+
| Board             | FPGA             | Toolchain                   |
+===================+==================+=============================+
| LimeSDR Mini V1   | Altera MAX10     | Intel Quartus Prime Lite    |
+-------------------+------------------+-----------------------------+
| LimeSDR Mini V2   | Lattice ECP5     | Yosys + nextpnr-ecp5        |
+-------------------+------------------+-----------------------------+
| XTRX              | Xilinx Artix-7   | Xilinx Vivado               |
+-------------------+------------------+-----------------------------+

Notes:

- For **Intel MAX10**, Intel Quartus Prime Lite Edition is sufficient (Quartus Pro is not required).
- For **Lattice ECP5**, the open-source Yosys/nextpnr-ecp5 toolchain is employed.
- For **Xilinx Artix-7**, Xilinx Vivado (WebPACK or Standard edition) is required.

The LiteX build system automatically detects the board and selects the appropriate toolchain,
generating project files, constraints, and build scripts tailored to the vendor.

For CPU firmware compiler setup, see :doc:`firmware_toolchains`.
