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

  - **Vivado 2022.1** (or later) is required.
    Download it from `www.xilinx.com <http://www.xilinx.com>`_.
    *Before building, ensure that Vivado’s settings are sourced or that Vivado’s binaries are in your `$PATH`.*

- **For limesdr_mini_v1:**

  - **Quartus** is required.
    Download and install the free version of Intel/Altera Quartus.

- **For limesdr_mini_v2:**

  - **Diamond** or **Yosys/nextPNR/Trellis** is required (default is Diamond).
    Consult your chosen toolchain’s documentation for installation details.

Cloning the Repository
----------------------
To clone the repository and initialize its submodules, run:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   git submodule init 
   git submodule update

Building and Loading the Gateware
---------------------------------
To build the gateware, use the following command with the appropriate board option:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build --board=limesdr [--cable ft2232] [--load] [--flash]

**Available options:**

- ``--board``: Specifies the board for which the gateware is being built.
- ``--build``: Builds the gateware.
- ``--cable``: Specifies the JTAG cable to be used.
- ``--with-bscan``: Adds JTAG access to the *vexriscv-smp* softcore for debugging.
- ``--load``: Loads the bitstream to SRAM.
- ``--flash``: Loads the bitstream to FLASH memory.

**Available boards for the ``--board`` option are:**

- limesdr

**Note:** The ``--load`` and ``--flash`` options use **JTAG** to communicate with the FPGA. These actions require an external probe (by default, a *digilent_hs2* cable is used). Use the ``--cable`` option to specify a different cable (see ``openFPGALoader --list-cables`` for supported cables).

**Example:**

In this example, we build the gateware for the limesdr_xtrx board and load it using an FT2232 mini module. Running

``openFPGALoader --list-cables``
shows two options for the module:

- ``ft2232`` - FT2232 mini module, using the A interface.
- ``ft2232_b`` - FT2232 mini module, using the B interface.

In this case, we use the A interface.

Before building, ensure that LiteX can locate the appropriate FPGA toolchain by either:
- Sourcing the toolchain’s settings manually.
- Setting the appropriate environment variable (e.g., ``LITEX_ENV_VIVADO`` for Vivado).
- Adding the toolchain’s binaries to your `$PATH`.

To build the project, run:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build --board=limesdr

After building, program the design to the device's RAM by running:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --load --board=limesdr --cable ft2232

Or, to program the design to the device's FLASH, run:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --flash --board=limesdr --cable ft2232

Check the console output to ensure all processes complete successfully.

Building/Loading VexRiscv Firmware through UART
------------------------------------------------
By default, firmware is built when the gateware is compiled and is loaded into SRAM.
Alternatively, firmware can be compiled and loaded through UART:

.. code:: bash

   # Build firmware:
   cd firmware && make clean all && cd ../

   # Load firmware through serial:
   litex_term /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv
