Building the project
====================


Requirements
------------

In order to build broject these tools/software are required:

-  **Litex** - follow offcial install instructions in
   `repository <https://github.com/enjoy-digital/litex>`__
-  **Vivado 2022.1** - free version can be downloaded from
   `www.xilinx.com <http://www.xilinx.com>`__
-  **SBT** - installation instructions can be found `here <https://www.scala-sbt.org/1.x/docs/Installing-sbt-on-Linux.html#Installing+sbt+on+Linux>`__
   

Cloning the Repository
----------------------

To clone this repository and initialize the submodules, run the
following commands:

.. code:: bash

   git clone https://github.com/myriadrf/LimeSDR_GW.git
   git submodule init 
   git submodule update

Building and loading the Gateware
---------------------------------

To build the gateware, use the following command with the appropriate
board option:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build --board=limesdr [--cable ft2232] [--load] [--flash]

**Available options:**

-  ``--board`` to specify the board for which the gateware is being built
-  ``--build`` to build gateware
-  ``--cable`` to specify JTAG cable to be used.
-  ``--with-bscan`` to add JTAG access to the *vexriscv-smp* softcore
   for debugging.
-  ``--load`` load bitstream to SRAM
-  ``--flash`` load bitstream to FLASH memory

**Available boards for --board option are:**

-  limesdr

**Note:** ``--load`` and ``--flash`` use **JTAG** to
communicate with the FPGA. These actions require an external probe, with
a *digilent_hs2* cable used by default. Use ``--cable`` followed by the
cable name to change this (see ``openFPGALoader --list-cables`` for
supported cables).

**Example**

In this example we will build the gateware for limesdr_xtrx board and load it using a ft2232 mini module.

``openFPGALoader --list-cables`` shows two options for the module:

- ``ft2232`` - FT2232 mini module, using the A interface
- ``ft2232_b`` - FT2232 mini module, using the B interface

In this example we will be using the A interface of the FT2232 mini module.
Before attempting to build the gateware, make sure that litex can find Xilinx Vivado tools.
You can do that in one of these ways:

- Source Vivado's settings manually.
- Set LITEX_ENV_VIVADO environment variant to Vivado's settings path.
- Add Vivado toolchain to your $PATH.
  
To build the project run 

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build --board=limesdr

After building the project you can program the design to the device's RAM by running

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --load --board=limesdr --cable ft2232

Or to the device's FLASH by running 

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --flash --board=limesdr --cable ft2232

Make sure to check the command output in the console to check if processes were completed successfully.


Building/loading VexRiscv firmware trough UART
----------------------------------------------

By default firmware should be built when building gateware and compiled
into SRAM. Separately firmware can by compiled and loaded through UART:

.. code:: bash

   # Build firmware:
   cd firmware && make clean all && cd ../

   # Load firmware trough serial
   litex_term  /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv



