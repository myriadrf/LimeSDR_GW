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
   │   ├── platforms/      <-- Board-specific I/O constraints and configuration files (e.g., pin assignments, timing constraints).
   │   └── targets/        <-- Integration scripts that define the build process (instantiating LiteX cores, peripherals, CPU, and firmware). These act as the "top-level" for SoC wiring and flow control.
   ├── docs/               <-- Documentation (design descriptions, build instructions, modification guidelines, etc.).
   ├── firmware/           <-- Firmware source code for the softcore CPU (drivers and application-specific logic).
   ├── gateware/           <-- FPGA hardware description files (Verilog/VHDL/LiteX) for custom logic and interconnects.
   ├── riscv_jtag_tunneled.tcl  <-- TCL script for automating JTAG debugging.
   ├── README.md           <-- Project overview and documentation links.
   └── .gitignore          <-- Files and directories ignored by version control.

Key Highlights
--------------
- **Boards Directory**:
  - *platforms/* contains the board-specific configuration files, such as I/O definitions, clock setups, and vendor-specific constraints.
  - *targets/* houses scripts that integrate hardware cores, peripheral components, the CPU, and firmware as part of the build process. These automate SoC assembly and handle build/flash flows.

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

Challenges and Best Practices
-----------------------------
Adding new boards or features (e.g., custom peripherals) may still require substantial effort and developer experience in areas like timing closure, vendor tools, or HDL integration. For boards closely aligned with existing ones (e.g., minor pin changes), the process is straightforward; otherwise, expect iterative debugging and potential custom developments if unsupported features arise.

Follow naming conventions for files, modules, and signals to maintain consistency. Ensure designs are portable across devices and vendors by using LiteX abstractions where possible.
