LimeSDR XTRX Build Instructions
===============================

To build gateware for the **limesdr_xtrx** target, run:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the board-specific requirements.

   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

**Command:**

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build [--load] [--flash] [--gold] [--cable <cable>]

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

   python3 -m boards.targets.limesdr_xtrx --build --gold

Programming Hardware
--------------------

The required hardware is listed in Table 1.

.. list-table:: Table 1. Required Hardware
   :header-rows: 1
   :widths: 35 15 35

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `PCIe x2 + RF frontend adapter <https://www.crowdsupply.com/lime-micro/limefea-mpcie-carrier-board>`_
     -
     - Mini PCIe to PCIe adapter with JTAG header pins

Supported JTAG programming cables are listed in Table 2.

.. list-table:: Table 2. Tested JTAG Programming Cables
   :header-rows: 1
   :widths: 35 15 35

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     -
     - Xilinx-compatible JTAG programming cable

FT2232H Mini Module Connection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To connect the FT2232H Mini Module:

* Insert the LimeSDR-XTRX v1.2 board into the RF frontend adapter Mini PCIe slot.
* Connect the JTAG signals between the FT2232H Mini Module and the RF frontend adapter as shown in Table 3.
* Select the adapter power-source jumper: USB or PCIe. See the RF frontend adapter documentation for details.
* Connect the USB cable to the FT2232H Mini Module.
* If USB power is selected, connect the micro-USB cable to the RF frontend adapter.
* If PCIe power is selected, insert the RF frontend adapter into a PCIe slot and power on the host PC.

.. list-table:: Table 3. FT2232H Mini Module to RF Frontend Adapter JTAG Connections
   :header-rows: 1
   :widths: 50 50

   * - **FT2232H Mini Module**
     - **PCIe x2 + RF frontend adapter**
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

Flashing Instructions
---------------------

- **User bitstream only:**

  .. code:: bash

     python3 -m boards.targets.limesdr_xtrx --flash

- **Golden bitstream only:**

  .. code:: bash

     python3 -m boards.targets.limesdr_xtrx --flash --gold

.. note::

   - When programming an empty flash device, both the golden and user bitstreams must be written;
     otherwise, the FPGA will not boot.
   - After the golden bitstream has been programmed, the user bitstream can be updated
     independently.
   - The required bitstream must be generated before flashing.