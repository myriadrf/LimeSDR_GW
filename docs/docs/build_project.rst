.. _build_the_project:

Building the Project
============================

This section explains how to build the gateware and work with the firmware for the supported boards. The build process relies on several mandatory tools (**LiteX, SBT, and GHDL**) as well as board-specific FPGA toolchains.

Requirements
------------

Before building the project, you **must install** the following tools:

- **LiteX**  
  **2025.08** version tag is required. Follow the official installation instructions in the
  `LiteX repository <https://github.com/enjoy-digital/litex>`_.

- **SBT (Scala Build Tool)**  
  Installation instructions can be found
  `here <https://www.scala-sbt.org/1.x/docs/Installing-sbt-on-Linux.html#Installing+sbt+on+Linux>`_.

- **GHDL**  
  **4.1.0** Version is required for VHDL-to-Verilog conversion in some targets.
  See the `GHDL repository <https://github.com/ghdl/ghdl>`_ for installation instructions.

- **openFPGALoader**
  Universal utility for programming FPGAs. 
  See the `openFPGALoader repository <https://github.com/trabucayre/openFPGALoader>`_ for installation instructions.

Additionally, the required FPGA toolchain depends on the target board:

- **For LimeSDR XTRX:**
   - **Vivado 2022.1** (or later) is required. Download it and install from `Xilinx <http://www.xilinx.com>`_. Free version can be used.
   - Ensure Vivado’s settings are sourced or its binaries are in your ``$PATH`` before building.

- **For LimeSDR Mini V1:**
   - **Quartus 23.1** (or later) is required. Download it and install from `Intel <https://www.intel.com>`_. Free version can be used.

- **For LimeSDR Mini V2:**
   - **Project Trellis (with Yosys and nextpnr)** OpenSource toolchain is required. Clone and install from `GitHub <https://github.com/YosysHQ/prjtrellis>`_.
   - **Diamond 3.14** (or later) is required for bitstream creation. Download it from `Lattice <https://www.latticesemi.com/>`_. Free version can be used.
  
Consult respective toolchain’s documentation for installation details.

.. note::

   While other tool versions might work, it is recommended to use the specified toolchain versions to avoid potential compatibility issues.

Cloning the Repository
----------------------

To clone the repository and initialize its submodules, run:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   git submodule init
   git submodule update

Build/Load/Flash Instructions
-----------------------------

Gateware for wanted target can be build with folowing command:

.. code:: bash

   python3 -m boards.targets.<target> --build


.. note::

   - Ensure required toolchain is installed and configured before building. See `Requirements <https://limesdrgw.myriadrf.org/docs/build_project#requirements>`_ section for respective board.  
   
   - Make sure to run build command from **project root directory**.
   
   
Detailed **build/load/flash** instructions and available options for each board can be found below:

.. toctree::
   :maxdepth: 1
   :titlesonly:

   LimeSDR XTRX <./limesdr-xtrx/build_gw>
   LimeSDR Mini V1 <./limesdr-mini-v1/build_gw>
   LimeSDR Mini V2 <./limesdr-mini-v2/build_gw>

