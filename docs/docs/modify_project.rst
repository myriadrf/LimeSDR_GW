=========================
Modifying the project
=========================

Gateware
----------------------
TBD

Firmware
----------------------

The firmware sources can be found in the ``firmware`` folder. The firmware can be built
using the ``Makefile`` provided in the same folder.

In order to successfully compile, the gateware project needs to be build at least once to generate
required sources and headers.

When building gateware, the firmware gets compiled automatically, it is not required to compile it manually.

Debug tools
----------------------

**Firmware Debug through GDB over JTAG**

To build and load a gateware with a debug interface:

.. code:: bash

   ./limesdr_xtrx.py --with-bscan --build --load --flash

   # Load firmware through serial:
   litex_term /dev/ttyUSBx --kernel firmware/firmware.bin

   # Run OpenOCD with one of the specified configurations:
   openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl
   or
   openocd -f ./openocd_xc7_ft2232.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

   # Connect GDB for debugging:
   gdb-multiarch -q firmware/firmware.elf -ex "target extended-remote localhost:3333"

Note that instead of using GDB directly, Eclipse IDE can be configured
to debug code in a more user-friendly way. Follow this guide to
configure Eclipse IDE: `Using Eclipse to run and debug the
software <https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software>`__