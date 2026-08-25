LimeSDR Mini V1 Build Instructions
==================================

To build gateware for the **limesdr_mini_v1** target, first activate the virtual environment and then run the build command:

.. code:: bash

   source .venv/bin/activate
   python3 -m boards.targets.limesdr_mini_v1 --build

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the board-specific requirements.

   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v1 --build [--golden] [--flash] [--with-uartbone] [--cable <cable>]

**Options:**

- ``--golden``: Build the golden bitstream. This image excludes the RX/TX path and LMS7002-related
  modules to reduce size.
- ``--flash``: Program both the golden and user bitstreams. Both bitstreams must be built first.
- ``--with-uartbone``: Enable UARTBone in the design.
- ``--cable <cable>``: Specify the programming cable, if required by the selected flash flow.

.. note::

   As of release 3.0:

   - The golden image must be built at least once before building the user image.
   - The ``--flash`` option works only when ``--golden`` is not used.

User and Golden Bitstreams
--------------------------

The ``--load`` option is not supported, because the user bitstream executes firmware from internal
flash.

To update the LimeSDR Mini V1, use the following sequence:

1. Build the golden bitstream, if it has not already been generated, using the ``--golden`` option.
2. Build the user bitstream.
3. After the build completes, the generated output files include ``.rpd``, ``.pof``, and ``.svf``.

Programming Cables
------------------


The FT2232H Mini Module provides a low-cost JTAG programming interface for LimeSDR Mini v1 and other devices.

.. list-table:: Table 1. Tested JTAG Programming Cables
   :header-rows: 1
   :widths: 35 15 50

   * - **Hardware**
     - **Version**
     - **Comment**
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     -
     - Compatible JTAG programming cable

The FT2232H Mini Module costs approximately $20 from distributors such as Digi-Key and Mouser. JTAG uses four signals: TCK, TMS, TDI, and TDO. This setup uses FT2232H port A (0).

FT2232H Mini Module preparation:

* Connect CN3-1 to CN3-3 to supply VCC from USB VBUS.
* Connect LimeSDR Mini v1 to the FT2232H Mini Module as specified in Table 2 and shown in Figure 1.

.. table:: Table 2. LimeSDR Mini v1 board and FT2232H Mini module connections

  +------------------------------------+---------------------------------+
  | **LimeSDR Mini v1**                | **FT2232H Mini module**         |
  +====================================+=================================+
  | J3-1 (GND)                         | CN2-2 (GND)                     |
  +------------------------------------+---------------------------------+
  | J3-2 (FPGA_JTAG_TCK)               | CN2-7 (AD0)                     |
  +------------------------------------+---------------------------------+
  | J3-3 (FPGA_JTAG_TDO)               | CN2-9 (AD2)                     |
  +------------------------------------+---------------------------------+
  | J3-4 (FPGA_JTAG_TMS)               | CN2-12 (AD3)                    |
  +------------------------------------+---------------------------------+
  | J3-5 (FPGA_JTAG_TDI)               | CN2-10 (AD1)                    |
  +------------------------------------+---------------------------------+
  | J3-6 (VCC3P3)                      | CN2-11 (VIO)                    |
  +------------------------------------+---------------------------------+


Flashing Instructions
---------------------

- **Full flash image (user + golden):**

  .. code:: bash

     python3 -m boards.targets.limesdr_mini_v1 --flash

  Or:

  .. code:: bash

     openFPGALoader -c <cable> LimeSDR-Mini_lms7_trx_HW_1.2.svf
