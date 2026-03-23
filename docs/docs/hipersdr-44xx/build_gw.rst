HiperSDR 44xx build instructions 
================================


To build the gateware for **hipersdr_44xx** target, use command:

.. code:: bash

   python3 -m boards.targets.hipersdr_44xx --build
   
.. note::

   - Ensure required toolchain is installed and configured before building. See :ref:`Requirements <requirements>` section for respective board.  
   
   - Make sure to run build command from **project root directory**.
  
Available build options
-----------------------

**Comand:**

.. code:: bash

   python3 -m boards.targets.hipersdr_44xx --build [--load] [--flash] [--gold] [--cable <cable>]

**Options:**

- ``--load``: Loads the bitstream into SRAM (volatile memory).
- ``--flash``: Programs the bitstream into SPI FLASH memory.
- ``--gold``: Build/Flash golden image instead of user.
- ``--cable``: Specifies the JTAG cable (default: *ft2232*). Use ``openFPGALoader --list-cables`` for options.
 

User/Golden Bitstreams
----------------------

- The **User bitstream** is built using the default command above.
- The **Golden bitstream** is built using the ``--gold`` option:
.. code:: bash

   python3 -m boards.targets.hipersdr_44xx --build  --gold


Programming cables
------------------

JTAG programming (openFPGALoader)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes how to program the Xilinx FPGA configuration flash
memory on the **HiperSDR 44xx** board using the JTAG interface and
**openFPGALoader** software.


Tested JTAG programming cables are listed in Table 1.

.. list-table:: Table 1. Tested JTAG programming cables
   :header-rows: 1
   :widths: 35 15 50

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     -
     - Xilinx-compatible JTAG programming cable

FT2232H Mini Module JTAG adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Connect the JTAG jumper wires between the FT2232H Mini Module and the
  HiperSDR 44xx board as shown in Table 2.
* Insert the board into a PCIe slot and power on the PC.

.. list-table:: Table 2. HiperSDR 44xx board and FT2232H Mini Module connections
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

   Make sure **CN3-1 (VBUS)** and **CN3-3 (VCC)** are connected with a
   jumper on the FT2232H Mini Module so that the module is powered.

Flashing Instructions
---------------------
- **User Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.hipersdr_44xx --flash

- **Golden Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.hipersdr_44xx --flash --gold
     
     
.. note::

  - Both User and Gold bitstreams must be programmed when programming an empty FLASH memory otherwise the FPGA will not boot.
  - Once Gold bitstream is programmed into FLASH memory then just User bitstream can be upadated if needed. 
  - User/Gold Bitstreams has to be present/generated first before flashing
