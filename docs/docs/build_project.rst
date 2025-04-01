Building the Project
====================

This section explains how to build the gateware and work with the firmware for the supported boards. The build process relies on several mandatory tools (**LiteX, SBT, and GHDL**) as well as board-specific FPGA toolchains.

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

- **For LimeSDR XTRX:**
  - **Vivado 2022.1** (or later) is required.
  - Download from `Xilinx <http://www.xilinx.com>`_.
  - *Ensure Vivado’s settings are sourced or its binaries are in your ``$PATH`` before building.*

- **For LimeSDR Mini V1:**
  - **Quartus** is required.
  - Download and install the free version of Intel/Altera Quartus.

- **For LimeSDR Mini V2:**
  - **Diamond** or **Yosys/nextPNR/Trellis** is required (default is **Trellis**).
  - Consult your chosen toolchain’s documentation for installation details.

Cloning the Repository
----------------------

To clone the repository and initialize its submodules, run:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   git submodule init
   git submodule update

Build Instructions for Each Board
---------------------------------

Gateware for wanted target can be build and loaded to volatile memory with folowing command:

.. code:: bash

   python3 -m boards.targets.<target> --build --load
   
   
Detailed build instructions for each board can be found below:

.. toctree::
   :maxdepth: 1
   :titlesonly:

   LimeSDR XTRX <./limesdr-xtrx/build_gw>
   LimeSDR Mini V1 <./limesdr-mini-v1/build_gw>
   LimeSDR Mini V2 <./limesdr-mini-v2/build_gw>


Firmware Loading via UART
-------------------------

By default, firmware is built when gateware is compiled and loaded into SRAM.
Alternatively, firmware can be compiled and loaded via UART:

.. code:: bash

   # Build firmware:
   cd firmware && make clean all && cd ../

   # Load firmware through serial:
   litex_term /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv

