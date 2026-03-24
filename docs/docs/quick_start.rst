.. _quick_start:

Quick Start
===========

This section explains how to build the gateware and work with the firmware for the supported boards. The build process relies on several mandatory tools (**LiteX, SBT, and GHDL**) as well as board-specific FPGA toolchains.

.. _requirements:

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

- **openFPGALoader v1.1.1**
  Universal utility for programming FPGAs. 
  See the `openFPGALoader repository <https://github.com/trabucayre/openFPGALoader>`_ for installation instructions.

Additionally, the required FPGA toolchain depends on the target board:
  
.. list-table:: Board-specific FPGA toolchain requirements
   :widths: 20 30 20 30
   :header-rows: 1

   * - Board
     - Toolchain
     - Version
     - Notes
   * - LimeSDR XTRX
     - `Vivado <https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html>`_
     - 2022.1
     - Ensure Vivado is in ``$PATH`` or its environment is sourced.
   * - LimeSDR Mini V1
     - `Quartus <https://www.altera.com/products/development-tools/quartus>`_ 
     - 23.1
     - Free edition can be used.
   * - LimeSDR Mini V2
     - `Project Trellis (with Yosys and nextpnr) <https://github.com/YosysHQ/prjtrellis>`_

       `Diamond <https://www.latticesemi.com/latticediamond>`_
     - Trellis - build latest from source; 
       
       Diamond - 3.14 or later;
     - Diamond is required for bitstream generation.
   * - HiperSDR 44xx
     - `Vivado <https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html>`_
     - 2022.2 (exact version required)
     - Requires tactical patch `AR000035576 <https://adaptivesupport.amd.com/s/article/000035576?language=en_US>`_.
   * - sSDR rev2
     - `Vivado <https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html>`_
     - 2022.1
     - Ensure Vivado is in ``$PATH`` or its environment is sourced.
  

Consult respective toolchain’s documentation for installation details.

.. note::

   While other tool versions might work, it is recommended to use the specified toolchain versions to avoid potential compatibility issues.

Cloning the Repository
----------------------

To clone the repository and initialize its submodules, run:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   cd LimeSDR_GW
   git submodule init
   git submodule update

Build/Load/Flash Instructions
-----------------------------

Gateware for wanted target can be build with folowing command:

.. code:: bash

   python3 -m boards.targets.<target> --build


.. note::

   - Ensure required toolchain is installed and configured before building. See :ref:`Requirements <requirements>` section for respective board.  
   
   - Make sure to run build command from **project root directory**.
   
   
Detailed **build/load/flash** instructions and available options for each board can be found below:

.. toctree::
   :maxdepth: 1
   :titlesonly:

   LimeSDR XTRX <./limesdr-xtrx/build_gw>
   LimeSDR Mini V1 <./limesdr-mini-v1/build_gw>
   LimeSDR Mini V2 <./limesdr-mini-v2/build_gw>
   HiperSDR 44xx <./hipersdr-44xx/build_gw>
   sSDR rev2 <./ssdr_rev2/build_gw>
