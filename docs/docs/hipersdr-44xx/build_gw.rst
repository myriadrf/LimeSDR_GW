HiperSDR 44xx Build Instructions
================================

Before building gateware for the **hipersdr_44xx** target, initialize the private
**AFE79xx** submodule:

.. code:: bash

   git submodule update --init --checkout gateware/AFE79xx

.. note::

   Run this command once after cloning the project repository.

To build gateware for the **hipersdr_44xx** target, first activate the virtual environment and then run the build command:

.. code:: bash

   source .venv/bin/activate
   python3 -m boards.targets.hipersdr_44xx --build

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the board-specific requirements.

   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

**Command:**

.. code:: bash

   python3 -m boards.targets.hipersdr_44xx --build [--load] [--flash] [--gold] [--cable <cable>]

**Options:**

- ``--load``: Load the bitstream into SRAM (volatile memory).
- ``--flash``: Program the bitstream into SPI flash memory.
- ``--gold``: Build or flash the golden image instead of the user image.
- ``--cable <cable>``: Specify the JTAG cable. The default is ``ft2232``.
  Use ``openFPGALoader --list-cables`` to list supported cable names.

User and Golden Bitstreams
--------------------------

- The **user bitstream** is built using the default command shown above.
- The **golden bitstream** is built using the ``--gold`` option:

.. code:: bash

   python3 -m boards.targets.hipersdr_44xx --build --gold

Programming Cables
------------------

JTAG Programming (openFPGALoader)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes how to program the Xilinx FPGA configuration flash memory on the
**HiperSDR 44xx** board through the JTAG interface using **openFPGALoader**.

Tested JTAG programming cables are listed in Table 1.

.. list-table:: Table 1. Tested JTAG Programming Cables
   :header-rows: 1
   :widths: 35 15 50

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     -
     - Xilinx-compatible JTAG programming cable

FT2232H Mini Module JTAG Adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Connect the JTAG jumper wires between the FT2232H Mini Module and the HiperSDR 44xx board as
  shown in Table 2.
* Insert the board into a PCIe slot and power on the host PC.

.. list-table:: Table 2. HiperSDR 44xx Board and FT2232H Mini Module Connections
   :header-rows: 1
   :widths: 50 50

   * - **FT2232H Mini Module**
     - **HiperSDR 44xx**
   * - CN2-12 (AD3)
     - TMS
   * - CN2-10 (AD1)
     - TDI
   * - CN2-9 (AD2)
     - TDO
   * - CN2-7 (AD0)
     - TCK
   * - CN2-2 (GND)
     - GND
   * - CN2-11 (VIO)
     - VIO

.. note::

   Ensure that **CN3-1 (VBUS)** and **CN3-3 (VCC)** are connected with a jumper on the
   FT2232H Mini Module so that the module is powered.

Flashing Instructions
---------------------

- **User bitstream only:**

  .. code:: bash

     python3 -m boards.targets.hipersdr_44xx --flash

- **Golden bitstream only:**

  .. code:: bash

     python3 -m boards.targets.hipersdr_44xx --flash --gold

.. note::

   - When programming an empty SPI flash device, both the golden and user bitstreams must be
     written; otherwise, the FPGA will not boot.
   - After the golden bitstream has been programmed, the user bitstream can be updated
     independently.
   - The required bitstream must be generated before flashing.