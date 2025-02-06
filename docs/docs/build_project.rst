Building the Project
====================

This section explains how to build the gateware and work with the firmware for the supported boards. The build process relies on several mandatory tools (LiteX, SBT, and GHDL) as well as board-specific FPGA toolchains.

Requirements
------------
Before building the project, you **must install** the following tools:

- **LiteX**
  Follow the official installation instructions in the
  `LiteX repository <https://github.com/enjoy-digital/litex>`_.

- **SBT (Scala Build Tool)**
  Installation instructions can be found
  `here <https://www.scala-sbt.org/1.x/docs/Installing-sbt-on-Linux.html#Installing+sbt+on+Linux>`_.

- **GHDL**
  GHDL is required for VHDL-to-Verilog conversion in some targets.
  See the `GHDL repository <https://github.com/ghdl/ghdl>`_ for installation instructions.

Additionally, the required FPGA toolchain depends on the target board:

- **For limesdr_xtrx:**

  **Vivado 2022.1** (or later) is required.
  Download it from `www.xilinx.com <http://www.xilinx.com>`_.
  *Before building, ensure that Vivado’s settings are sourced or that Vivado’s binaries are in your `$PATH`.*

- **For limesdr_mini_v1:**

  **Quartus** is required.
  Download and install the free version of Intel/Altera Quartus.

- **For limesdr_mini_v2:**

  **Diamond** or **Yosys/nextPNR/Trellis** is required (default is trellis).
  Consult your chosen toolchain’s documentation for installation details.

Cloning the Repository
----------------------
To clone the repository and initialize its submodules, run:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   git submodule init
   git submodule update

Build Instructions for Each Board
---------------------------------

LimeSDR XTRX
~~~~~~~~~~~~
To build the gateware for the **limesdr_xtrx** board, use the following command:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build [--with-bios] [--load] [--write] [--cable <cable>]

**Options:**

- ``--with-bios``: Enables LiteX BIOS (requires additional resources).
- ``--load``: Loads the bitstream into SRAM (volatile memory).
- ``--write``: Programs the bitstream into SPI FLASH memory.
- ``--cable``: Specifies the JTAG cable to be used (default is *digilent_hs2*; see ``openFPGALoader --list-cables`` for options).

*Before building, ensure that Vivado is installed and its settings are sourced or its binaries are in your `$PATH`.*

LimeSDR Mini V1
~~~~~~~~~~~~~~~
To build the gateware for the **limesdr_mini_v1** board, use the following command:

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v1 --build [--with-bios] [--with-spi-flash] [--load] [--write] [--cable <cable>]

**Options:**

- ``--with-bios``: Enables LiteX BIOS.
- ``--with-spi-flash``: Enables SPI Flash support.
- ``--load``: Loads the bitstream into SRAM.
- ``--write``: Programs the bitstream into Internal FLASH memory.
- ``--cable``: Specifies the JTAG cable if required (default is *ft2232*; see ``openFPGALoader --list-cables`` for options).

*Before building, ensure that Quartus is installed and configured.*

LimeSDR Mini V2
~~~~~~~~~~~~~~~
To build the gateware for the **limesdr_mini_v2** board, use the following command:

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v2 --build [--with-bios] [--with-spi-flash] [--load] [--write] [--toolchain=TOOLCHAIN] [--cable <cable>]

**Options:**

- ``--toolchain=TOOLCHAIN``: Specify either **diamond** or **trellis** (default is **diamond**).
- ``--with-bios``: Enables LiteX BIOS.
- ``--with-spi-flash``: Enables SPI Flash support (only working with the trellis toolchain).
- ``--load``: Loads the bitstream into SRAM.
- ``--write``: Programs the bitstream into SPI FLASH.
- ``--cable``: Specifies the JTAG cable.

*Before building, ensure that your chosen FPGA toolchain (Diamond or Yosys/nextPNR/Trellis) is installed.*

Firmware Loading via UART
-------------------------
By default, firmware is built when the gateware is compiled and is loaded into SRAM.
Alternatively, firmware can be compiled and loaded through UART:

.. code:: bash

   # Build firmware:
   cd firmware && make clean all && cd ../

   # Load firmware through serial:
   litex_term /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv
