Project Structure
=================

Overview
--------
This page provides a detailed description of the repository organization for the LimeSDR GW project. For a high-level overview of the project, please refer to the *Introduction* page.

Repository Layout
-----------------
The repository is organized into several key directories, each serving a specific purpose:

.. code-block:: bash

   / (root)
   ├── boards/
   │   ├── platforms/      <-- Board-specific I/O constraints and configuration files.
   │   └── targets/        <-- Integration scripts that define the build process (instantiating LiteX cores, peripherals, CPU, and firmware).
   ├── docs/               <-- Documentation (design descriptions, build instructions, modification guidelines, etc.).
   ├── firmware/           <-- Firmware source code for the softcore CPU (drivers and application-specific logic).
   ├── gateware/           <-- FPGA hardware description files (Verilog/VHDL/LiteX) for custom logic and interconnects.
   ├── riscv_jtag_tunneled.tcl  <-- TCL script for automating JTAG debugging.
   ├── README.md           <-- Project overview and documentation links.
   └── .gitignore          <-- Files and directories ignored by version control.

Key Highlights
--------------
- **Boards Directory**:
  - *platforms/* contains the board-specific configuration files.
  - *targets/* houses scripts that integrate hardware cores, peripheral components, the CPU, and firmware as part of the build process.

- **Documentation (docs/)**:
  Contains comprehensive guides covering gateware design, build procedures, and project modifications.

- **Firmware and Gateware**:
  The firmware directory provides the software for the softcore CPU, while the gateware directory holds the RTL files (in Verilog, VHDL, or LiteX format) defining the FPGA logic.

- **Open Source Integration**:
  The project leverages open source tools (such as LiteX, VexRiscv, OpenOCD, GHDL, Yosys, and nextpnr) to ensure a transparent and community-driven development flow.

Additional Files
----------------

**riscv_jtag_tunneled.tcl**
  - A TCL script used for automating JTAG debugging with the RISC-V softcore. It streamlines tasks such as setting breakpoints and managing JTAG connections.

**README.md**
  - Provides an introduction to the project, build instructions, and links to additional documentation.

**.gitignore**
  - Lists files and directories that should be ignored by version control (e.g., temporary files and build outputs).
