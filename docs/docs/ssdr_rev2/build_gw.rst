sSDR rev2 Build Instructions
============================

To build gateware for the **ssdr** target, run:

.. code:: bash

   python3 -m boards.targets.ssdr --build

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the board-specific requirements.

   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

**Command:**

.. code:: bash

   python3 -m boards.targets.ssdr --build [--load] [--flash] [--gold] [--cable <cable>]

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

   python3 -m boards.targets.ssdr --build --gold

Programming Cables
------------------

FT2232H Mini Module JTAG Adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Insert the sSDR rev2 board into the ``m2m_pcie`` adapter card.
* Connect the JTAG jumper wires between the FT2232H Mini Module and the ``m2m_pcie`` adapter
  JTAG header as shown in Table 1.
* Connect the USB cable to the FT2232H Mini Module.
* Insert the ``m2m_pcie`` adapter into a PCIe slot and power on the host PC.

.. table:: Table 1. sSDR Board (m2m_pcie Adapter) and FT2232H Mini Module Connections

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

Digilent HS2 JTAG Adapter
^^^^^^^^^^^^^^^^^^^^^^^^^

* Insert the sSDR rev2 board into the ``m2m_pcie`` adapter card.
* Connect the JTAG jumper wires between the Digilent HS2 and the ``m2m_pcie`` adapter JTAG header
  as shown in Table 2.
* Connect the USB cable to the Digilent HS2.
* Insert the ``m2m_pcie`` adapter into a PCIe slot and power on the host PC.

.. table:: Table 2. sSDR Board (m2m_pcie Adapter) and Digilent HS2 Connections

  +------------------------------------+------------------------------------+
  | **Digilent HS2**                   | **sSDR + m2m_pcie adapter**        |
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

- **User bitstream only:**

  .. code:: bash

     python3 -m boards.targets.ssdr --flash

- **Golden bitstream only:**

  .. code:: bash

     python3 -m boards.targets.ssdr --flash --gold

.. note::

   - When programming an empty SPI flash device, both the golden and user bitstreams must be
     written; otherwise, the FPGA will not boot.
   - After the golden bitstream has been programmed, the user bitstream can be updated
     independently.
   - The required bitstream must be generated before flashing.