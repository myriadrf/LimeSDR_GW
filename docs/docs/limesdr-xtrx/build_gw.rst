LimeSDR XTRX build instructions 
===============================


To build the gateware for **limesdr_xtrx** target, use command:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build
   
.. note::

   - Ensure required toolchain is installed and configured before building. See :ref:`Requirements <requirements>` section for respective board.  
   
   - Make sure to run build command from **project root directory**.
  
Available build options
-----------------------

**Comand:**

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build [--load] [--flash] [--gold] [--cable <cable>]

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

   python3 -m boards.targets.limesdr_xtrx --build  --gold


Programming cables
------------------

The required hardware is listed in Table 1.

.. list-table:: Table 1. Required hardware
   :header-rows: 1
   :widths: 35 15 35

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `PCIe x2 + RF frontend adapter <https://www.crowdsupply.com/lime-micro/limefea-mpcie-carrier-board>`_
     -
     - Mini PCIe to PCIe adapter with JTAG header pins

Supported JTAG programming cables are listed in Table 2.

.. list-table:: Table 2. Tested JTAG programming cables
   :header-rows: 1
   :widths: 35 15 35

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     -
     - Xilinx-compatible JTAG programming cable

FT2232H Mini Module connection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To connect the FT2232H Mini Module:

* Insert the LimeSDR-XTRX v1.2 into the RF frontend adapter mini PCIe slot.
* Connect the JTAG signals between the FT2232H Mini Module and the RF frontend adapter as shown in Table 3.
* Select the adapter power source jumper: USB or PCIe (check RF frontend adapter hardware for details).
* Connect the USB cable to the FT2232H Mini Module.
* If USB power is selected, connect the micro USB cable to the RF frontend adapter.
* If PCIe power is selected, insert the RF frontend adapter into a PCIe slot and power on the PC.

.. list-table:: Table 3. FT2232H Mini Module to RF frontend adapter JTAG connections
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
- **User Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.llimesdr_xtrx --flash

- **Golden Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.llimesdr_xtrx --flash --gold
     
     
.. note::

   - Both User and Gold bitstreams must be programmed when programming an empty flash otherwise the FPGA will not boot.
   - Once Gold bitstream is programmed into FLASH memory then just User bitstream can be upadated if needed.  
   - User/Gold bitstreams must be generated before flashing.
