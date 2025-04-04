Modifying the Project
=====================

This section describes how to modify the gateware and firmware of the project. It explains where the source files are located and provides an examples of adding a custom modules to the LimeSDR design.

Gateware
--------
The gateware sources are organized into several folders:

- **boards/targets**
  Python LiteX files that contain board-specific top-level gateware descriptions.
- **boards/platforms**
  Python LiteX files that define platform constraints, such as pin locations and I/O standards.
- **gateware**
  HDL source files and module descriptions.
  
To make it easier to understand how to add a custom module, some examples are provided below:
  
.. toctree::
   :maxdepth: 1
   :titlesonly:

   FFT Example <./fft_example>


Firmware
--------
The firmware sources are located in the ``firmware`` folder and are built using the provided ``Makefile``. The gateware project must be built at least once to generate the necessary sources and headers for firmware compilation. When the gateware is built, the firmware is automatically compiled, so manual compilation is not required.

Firmware Loading via UART
-------------------------

By default, firmware is built when gateware is compiled and loaded into SRAM.
Alternatively, firmware can be compiled and loaded via UART:

.. code:: bash

   # Build firmware:
   cd firmware && make clean all && cd ../

   # Load firmware through serial:
   litex_term /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv

Debug Tools
-----------
**Firmware Debug through GDB over JTAG**

To build and load a gateware with a debug interface, run:

.. code-block:: bash

    python3 -m boards.targets.limesdr_xtrx --with-bscan --build --load --flash

Then, load the firmware through serial:

.. code-block:: bash

    litex_term /dev/ttyUSBx --kernel firmware/firmware.bin

Run OpenOCD with one of the configurations:

.. code-block:: bash

    openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl
    # or
    openocd -f ./openocd_xc7_ft2232.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

Finally, connect GDB for debugging:

.. code-block:: bash

    gdb-multiarch -q firmware/firmware.elf -ex "target extended-remote localhost:3333"

For a more user-friendly debugging experience, you can also configure Eclipse IDE. Refer to the guide:
`Using Eclipse to run and debug the software <https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software>`_.
