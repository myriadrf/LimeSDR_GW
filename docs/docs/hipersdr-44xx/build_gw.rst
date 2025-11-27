HiperSDR 44xx build instructions 
================================


To build the gateware for **hipersdr_44xx** target, use command:

.. code:: bash

   python3 -m boards.targets.hipersdr_44xx --build
   
.. note::

   - Ensure required toolchain is installed and configured before building. See `Requirements <https://limesdrgw.myriadrf.org/docs/build_project#requirements>`_ section for respective board.  
   
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

this section describes how to program the Xilinx FPGA configuration FLASH memory used on the **HiperSDR 44xx** board with JTAG interface and openFGPALoader software.

Used software and hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^


List of used software is given in Table 1.

.. table:: Table 1. Required software and tools

  +------------------------------------------------------------------------------+-------------------------------+---------------------------------------------------+
  | **Tool**                                                                     | **Version**                   | **Comment**                                       |
  +==============================================================================+===============================+===================================================+
  | `openFPGALoader - v0.13.1 <https://github.com/trabucayre/openFPGALoader>`__  | v0.13.1                       | Universal utility for programming FPGAs           |
  +------------------------------------------------------------------------------+-------------------------------+---------------------------------------------------+


List of tested JTAG programming cables is given in Table 2.

.. table:: Table 2. Tested JTAG programming cables

  +------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------+------------------------------------------------------------------------------------+
  | **Hardware**                                                                                                                                   | **Version**                   | **Comment**                                                                        |
  +================================================================================================================================================+===============================+====================================================================================+
  | `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`__                                                                   |                               | Original JTAG programming cable from Xilinx                                        |
  +------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------+------------------------------------------------------------------------------------+


FT2232H Mini Module JTAG adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Connect JTAG jumper cables as shown in Table 3 between JTAG adapter and HiperSDR 44xx board.
* Insert board into PCIe slot and power up PC.


.. table:: Table 3. HiperSDR 44xx board and FT2232H Mini module connections

  +------------------------------------+------------------------------------+
  | **FT2232H Mini Module**            | **HiperSDR 44xx**                  |
  +====================================+====================================+
  | CN2-12 (AD3)                       | TMS                                |
  +------------------------------------+------------------------------------+
  | CN2-10 (AD1)                       | TDI                                |
  +------------------------------------+------------------------------------+
  | CN2-9 (AD2)                        | TDO                                |
  +------------------------------------+------------------------------------+
  | CN2-7 (AD0)                        | TCK                                |
  +------------------------------------+------------------------------------+
  | CN2-2 (GND)                        | GND                                |
  +------------------------------------+------------------------------------+
  | CN2-11 (VIO)                       | VIO                                |
  +------------------------------------+------------------------------------+

   
.. note::

   Make sure that **CN3-1(VBUS)** and **CN3-3(VCC)** pins are connected together with jumper in order to power up FT2232H mini module. 


Flashing Instructions
---------------------
- **User Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.hipersdr_44xx --flash

- **Golden Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.hipersdr_44xx --flash --gold
     
     
.. note::

	User/Gold Bitstreams has to be present/generated first before flashing
