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
- ``--with-jtagbone``: Enable JTAGBone (Wishbone-over-JTAG) for gateware debugging and register access via ``litex_server``. Mutually exclusive with ``--with-cpu-debug``.
- ``--with-cpu-debug``: Enable spec-compliant RISC-V CPU debug module over a dedicated JTAG tunnel. Automatically disables DDR2 (implies ``--no-ddr``) to fit within Cyclone IV RAM/logic limits, and is mutually exclusive with ``--with-jtagbone``.
- ``--no-ddr``: Disable DDR2 memory modules and DDR-backed features (e.g. WFM player) to free FPGA logic and RAM resources. Automatically implied when ``--with-cpu-debug`` is used.
- ``--no-ddr-test``: Exclude the DDR2 Built-in Self-Test (BIST) core and pseudorandom pattern generator from the PSS module to save logic resources.
- ``--no-ppsdo``: Exclude the PPS Disciplined Oscillator subsystem from synthesis to reduce Cyclone IV logic element (LE) utilization and routing congestion.
- ``--ep01-size {4096,8192,16384}``: Host-to-FPGA (TX) stream FIFO size in bytes. Default 4096. Larger sizes need the RAM freed by ``--no-ddr`` (see below).
- ``--ep81-size {8192,16384,32768}``: FPGA-to-host (RX) stream FIFO size in bytes. Default 8192. Larger sizes need the RAM freed by ``--no-ddr``.
- ``--tx-buf-size {16384,32768,40960,49152,65536}``: TX packet buffer size in bytes. Default 16384. Larger sizes need the RAM freed by ``--no-ddr``.
- ``--no-soc-json``: Disable automatic export/generation of the SoC module hierarchy JSON file (``soc_structure.json``).
- ``--doc``: Generate LiteX SoC Sphinx documentation and build the HTML output.
- ``--cable <cable>``: Specify the JTAG cable (default is ``ft2232``).

.. note::

   **Build Constraints & Mutual Exclusivity:**

   - ``--with-jtagbone`` and ``--with-cpu-debug`` are mutually exclusive because the Intel Cyclone IV FPGA provides only a single Virtual JTAG (``sld_virtual_jtag``) hardware IP instance.
   - Specifying ``--with-cpu-debug`` automatically disables DDR2 memory (``--no-ddr``) to preserve Cyclone IV EP4CE40 FPGA block RAM (M9K) and logic resources.

Build Configurations
--------------------

**Default build.** ``python3 -m boards.targets.limesdr_usb --build`` includes the full feature
set: the DDR2 memory controller, the WFM (waveform) player with its host-to-FPGA load path, and
the DDR2 self-test core. It reaches every supported sample rate.

**Builds without DDR2.** If the WFM player and DDR2 memory are not needed, ``--no-ddr`` removes
them, together with the DDR2 self-test (it implies ``--no-ddr-test``). This frees about a third of
the FPGA logic and block RAM. The build also packs the remaining RAM more densely. The freed
resources leave room for larger stream buffers and give the fitter more placement and timing
headroom. That headroom may add some stream robustness at the highest sample rates.

**Stream buffer sizes.** ``--ep01-size``, ``--ep81-size`` and ``--tx-buf-size`` set the size of the
USB stream FIFOs and of the TX packet buffer. Larger buffers give more tolerance to host-side
scheduling delays. With a host configured as in the note below, the default sizes already reach
the maximum sample rates. Sizes above the defaults need ``--no-ddr``. The largest tested
combination is ``--no-ddr --ep01-size 8192 --ep81-size 32768 --tx-buf-size 40960``.

Approximate resource use on the EP4CE40:

.. list-table::
   :header-rows: 1
   :widths: 52 16 16 16

   * - Configuration
     - M9K blocks
     - Logic elements
     - DDR2 / WFM player
   * - Default
     - ~87 %
     - ~86 %
     - Yes
   * - ``--no-ddr``
     - ~56 %
     - ~57 %
     - No
   * - ``--no-ddr --ep01-size 8192 --ep81-size 32768 --tx-buf-size 40960``
     - ~96 %
     - ~60 %
     - No

.. note::

   For the highest sample rates (61.44 MSPS SISO, 61.44 MSPS MIMO RX, 30.72 MSPS MIMO TX+RX), give
   the host stream thread real-time scheduling priority (for example an ``rtprio`` limit in
   ``/etc/security/limits.conf``). This applies to every build configuration.

Programming Hardware
--------------------

The LimeSDR USB uses an Intel/Altera Cyclone IV FPGA. Supported programming cables include:

.. list-table:: Supported JTAG Programming Cables
   :header-rows: 1
   :widths: 35 65

   * - **Hardware**
     - **Comment**
   * - `Altera USB Blaster <https://www.intel.com/content/www/us/en/docs/programmable/683116/21-3/usb-blaster-download-cable-user-guide.html>`_
     - Standard Altera JTAG programmer.
   * - `FT2232H Mini Module <https://ftdichip.com/products/ft2232h-mini-module/>`_
     - Used for JTAG communication via OpenOCD or openFPGALoader.

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