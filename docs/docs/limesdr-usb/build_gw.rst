LimeSDR USB Build Instructions
==============================

To build gateware for the **limesdr_usb** target, first activate the virtual environment and then run the build command:

.. code:: bash

   source .venv/bin/activate
   python3 -m boards.targets.limesdr_usb --build

.. note::

   - Ensure that the required toolchain (Quartus Prime 23.1) is installed and configured.
   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

**Command:**

.. code:: bash

   python3 -m boards.targets.limesdr_usb --build [options]

**Options:**

- ``--with-bios``: Enable the LiteX BIOS. By default, the CPU executes firmware directly from the integrated ROM.
- ``--with-jtagbone``: Enable JTAGBone (Wishbone-over-JTAG) for gateware debugging and register access via ``litex_server``.
- ``--with-cpu-debug``: Enable spec-compliant RISC-V CPU debug module over a dedicated JTAG tunnel.
- ``--no-ddr``: Disable DDR2 memory modules to free FPGA resources. This is automatically enabled when ``--with-cpu-debug`` is used.
- ``--cable <cable>``: Specify the JTAG cable (default is ``ft2232``).

Programming Cables
------------------

The LimeSDR USB uses an Intel/Altera Cyclone IV FPGA. Tested and supported programming cables include:

.. list-table:: Supported JTAG Programming Cables
   :header-rows: 1
   :widths: 35 65

   * - **Hardware**
     - **Comment**
   * - `Altera USB Blaster <https://www.intel.com/content/www/us/en/docs/programmable/683116/21-3/usb-blaster-download-cable-user-guide.html>`_
     - Standard Altera JTAG programmer.
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     - Used for JTAG communication via OpenOCD or openFPGALoader.

Note: LimeSDR USB's JTAG 10-pin header has a pitch of 0.05" and may require an adapter.

FT2232H Mini Module JTAG Adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Connect the JTAG jumper wires between the FT2232H Mini Module and the LimeSDR USB's
  JTAG header as shown in Table 1.
* Connect the USB cable to the FT2232H Mini Module.
* Connect the USB cable to the LimeSDR USB.

.. table:: Table 1. LimeSDR USB and FT2232H Mini Module Connections

  +------------------------------------+---------------------------------+
  | **LimeSDR USB**                    | **FT2232H Mini module**         |
  +====================================+=================================+
  | J11-2 (GND)                        | CN2-2 (GND)                     |
  +------------------------------------+---------------------------------+
  | J11-1 (FPGA_JTAG_TCK)              | CN2-7 (AD0)                     |
  +------------------------------------+---------------------------------+
  | J11-3 (FPGA_JTAG_TDO)              | CN2-9 (AD2)                     |
  +------------------------------------+---------------------------------+
  | J11-5 (FPGA_JTAG_TMS)              | CN2-12 (AD3)                    |
  +------------------------------------+---------------------------------+
  | J11-9 (FPGA_JTAG_TDI)              | CN2-10 (AD1)                    |
  +------------------------------------+---------------------------------+
  | J11-4 (VCC2P5A_FPGA)               | CN2-11 (VIO)                    |
  +------------------------------------+---------------------------------+

Altera USB Blaster
^^^^^^^^^^^^^^^^^^^^^^^^^

* Connect the Altera USB Blaster to the LimeSDR USB JTAG header (pinouts match 1:1).
* Connect the USB cable to the Altera USB Blaster.
* Connect the USB cable to the LimeSDR USB.

Programming Instructions
------------------------

Loading to SRAM (Volatile)
^^^^^^^^^^^^^^^^^^^^^^^^^^
To load the bitstream into the FPGA's volatile memory (SRAM), use the ``--load`` option:

.. code:: bash

   python3 -m boards.targets.limesdr_usb --load --cable <cable>

This uses the generated ``.svf`` file located in ``bitstream/LimeSDR_USB/``.

Flashing to Non-Volatile Memory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To program the bitstream into the onboard SPI flash memory, use the ``--flash`` option:

.. code:: bash

   python3 -m boards.targets.limesdr_usb --flash --cable <cable>

**SVF Concatenation Process:**
Because the Cyclone IV requires a Serial Flash Loader (SFL) bridge to access the flash memory via JTAG, the build system automatically generates a self-contained flash SVF (``_flash.svf``). This file concatenates:

1. **SFL Bridge**: A pre-built SVF (``sfl_ep4ce40_020f40dd.svf``) that loads the flash-access bridge into the FPGA.
2. **Flash Operations**: The actual erase, program, and verify operations derived from the generated ``.jic`` file.

This self-contained SVF can be used directly with ``openFPGALoader`` or other SVF players without needing a separate bridge-loading step.