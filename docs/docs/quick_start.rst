.. _quick_start:

Quick Start
===========

This section describes the prerequisites and basic workflow for building gateware and firmware for
the supported boards. The build process relies on several common tools (**LiteX, SBT, GHDL, and
openFPGALoader**) as well as board-specific FPGA toolchains.

.. _requirements:

Requirements
------------

Before building the project, install the following required tools:

- **LiteX**
  Tag **2025.08** is required. Follow the installation instructions in the
  `LiteX repository <https://github.com/enjoy-digital/litex>`_.
  Ensure **RISC-V toolchain support** is installed together with LiteX.

- **SBT (Scala Build Tool)**  
  Installation instructions are available in the official
  `SBT documentation <https://www.scala-sbt.org/1.x/docs/Installing-sbt-on-Linux.html#Installing+sbt+on+Linux>`_.

- **GHDL**  
  Version **4.1.0** is required for VHDL-to-Verilog conversion on some targets.
  See the `GHDL repository <https://github.com/ghdl/ghdl>`_ for installation instructions.

- **openFPGALoader**  
  Version **1.1.1** is required. This tool is used to program supported FPGA devices.
  See the `openFPGALoader repository <https://github.com/trabucayre/openFPGALoader>`_ for installation instructions.

In addition, each board requires a specific FPGA toolchain:

.. list-table:: Board-specific FPGA toolchains
   :widths: 20 30 20 25
   :header-rows: 1

   * - Board
     - Toolchain
     - Version
     - Notes
   * - LimeSDR XTRX
     - `Vivado <https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html>`_
     - 2022.1
     - Ensure Vivado is available in ``$PATH`` or that its environment has been sourced.
   * - LimeSDR Mini V1
     - `Quartus <https://www.altera.com/products/development-tools/quartus>`_
     - 23.1
     - The free edition can be used.
   * - LimeSDR Mini V2
     - `Project Trellis <https://github.com/YosysHQ/prjtrellis>`_ + Yosys and nextpnr;
  
       `Diamond <https://www.latticesemi.com/latticediamond>`_
     - Trellis: latest from source; 
  
       Diamond: 3.14
     - Diamond is required for final bitstream generation.
   * - HiperSDR-44xx
     - `Vivado <https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html>`_
     - 2022.2
     - Exact version required. Also requires AMD patch
       `AR000035576 <https://adaptivesupport.amd.com/s/article/000035576?language=en_US>`_.
   * - sSDR rev2
     - `Vivado <https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html>`_
     - 2022.1
     - Ensure Vivado is available in ``$PATH`` or that its environment has been sourced.

Consult the respective toolchain documentation for installation details.

.. note::

   Other tool versions may work, but the versions listed above are recommended to avoid
   compatibility issues.

Cloning the Repository
----------------------

To clone the repository and initialize its submodules, run:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   cd LimeSDR_GW
   git submodule init
   git submodule update

Build, Load, and Flash
----------------------

Gateware for a selected target can be built with the following command:

.. code:: bash

   python3 -m boards.targets.<target> --build

Replace ``<target>`` with the appropriate board target module name.

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the toolchain required by your board.

   - Run the build command from the **project root directory**.

Detailed build, load, and flash instructions for each supported board are provided below:

.. toctree::
   :maxdepth: 1
   :titlesonly:

   LimeSDR XTRX <./limesdr-xtrx/build_gw>
   LimeSDR Mini V1 <./limesdr-mini-v1/build_gw>
   LimeSDR Mini V2 <./limesdr-mini-v2/build_gw>
   HiperSDR-44xx <./hipersdr-44xx/build_gw>
   sSDR rev2 <./ssdr_rev2/build_gw>
