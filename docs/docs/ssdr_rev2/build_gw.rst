sSDR rev2 build instructions 
============================


To build the gateware for **ssdr** target, use command:

.. code:: bash

   python3 -m boards.targets.ssdr --build
   
.. note::

   - Ensure required toolchain is installed and configured before building. See :ref:`Requirements <requirements>` section for respective board.  
   
   - Make sure to run build command from **project root directory**.
  
Available build options
-----------------------

**Comand:**

.. code:: bash

   python3 -m boards.targets.ssdr --build [--load] [--flash] [--gold] [--cable <cable>]

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

   python3 -m boards.targets.ssdr --build  --gold


Programming cables
------------------

FT2232H Mini Module JTAG adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Insert sSDR rev2 into m2m_pcie addapter card. 
* Connect JTAG jumper cables as shown in Table 1 between JTAG cable and m2m_pcie adapter JTAG header.
* Connect USB cable to FT2232H Mini Module.
* Insert m2m_pcie adapter into PCIe slot and power up PC.


.. table:: Table 1. sSDR board (m2m_pcie adapter) and FT2232H Mini module connections

  +------------------------------------+------------------------------------+
  | **FT2232H Mini Module**            | **sSDR + m2m_pcie adapter**        |
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
   
Digilent Hs2 JTAG adapter
^^^^^^^^^^^^^^^^^^^^^^^^^

* Insert sSDR rev2 into m2m_pcie addapter card. 
* Connect JTAG jumper cables as shown in Table 2 between JTAG cable and m2m_pcie adapter JTAG header.
* Connect USB cable to Digilent Hs2 .
* Insert m2m_pcie adapter into PCIe slot and power up PC.

.. table:: Table 2. sSDR board (m2m_pcie adapter) and Digilent Hs2 connections

  +------------------------------------+------------------------------------+
  | **Digilent Hs2**                   | **sSDR + m2m_pcie adapter**        |
  +====================================+====================================+
  | TMS                                | TMS                                |
  +------------------------------------+------------------------------------+
  | TDI                                | TDI                                |
  +------------------------------------+------------------------------------+
  | TDO                                | TDO                                |
  +------------------------------------+------------------------------------+
  | TCK                                | TCK                                |
  +------------------------------------+------------------------------------+
  | GND                                | GND                                |
  +------------------------------------+------------------------------------+
  | VDD                                | VIO                                |
  +------------------------------------+------------------------------------+

Flashing Instructions
---------------------
- **User Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.ssdr --flash

- **Golden Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.ssdr --flash --gold
     
     
.. note::
   
  - Both User and Gold bitstreams must be programmed when programming an empty flash otherwise the FPGA will not boot.
  - Once Gold bitstream is programmed into FLASH memory then just User bitstream can be upadated if needed. 
  - User/Gold Bitstreams has to be present/generated first before flashing.
