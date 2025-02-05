Project Structure
=================

Overview
--------
This FPGA gateware project leverages the **LiteX** framework to build and manage top-level designs for multiple FPGA boards. It integrates custom hardware logic (written in VHDL/Verilog) with pre-designed LiteX components such as **LitePCIe**, **VexRiscV** softcore CPUs, and various communication interfaces (**I2C**, **SPI**, etc.). The design is modular, scalable, and adaptable to support different hardware targets, enabling rapid prototyping, easy maintenance, and future extensibility to other platforms. The project also embraces a comprehensive suite of open source tools.

Repository Structure
--------------------
The repository is organized into distinct directories for board-specific configurations, firmware, gateware, and software. An overview of the key directories and files is provided below:

.. code-block:: bash

   / (root)
   ├── boards/
   │   ├── platforms/
   │   │   ├── fairwaves_xtrx_platform.py
   │   │   ├── limesdr_xtrx_platform.py
   │   │   ├── limesdr_mini_v1_platform.py
   │   │   └── limesdr_mini_v2_platform.py
   │   └── targets/
   │       ├── limesdr_xtrx.py
   │       ├── limesdr_mini_v1.py
   │       └── limesdr_mini_v2.py
   ├── docs/
   ├── firmware/
   ├── gateware/
   ├── riscv_jtag_tunneled.tcl
   ├── README.rst
   └── .gitignore

Directory Details
-----------------

**boards/**
  - **platforms/**: Contains board-specific I/O constraints and configuration files.
  - **targets/**: Contains integration scripts that define the build process for each board, orchestrating the instantiation of LiteX cores, peripheral components, the CPU, and its associated firmware.

**docs/**
  - Contains all project documentation, including design descriptions, user guides, and tutorials. This folder provides detailed instructions on how to build, customize, and deploy the FPGA designs.

**firmware/**
  - Hosts the firmware code that runs on the softcore CPU. It includes drivers and application-specific logic used to initialize and manage the FPGA hardware.

**gateware/**
  - Contains the FPGA hardware description files (Verilog/VHDL/LiteX). This directory defines the custom logic, interconnects, and integration with LiteX components for core processing and peripheral management.

Additional Files
----------------

**riscv_jtag_tunneled.tcl**
  - A TCL script used for automating JTAG debugging with the RISC-V softcore. It streamlines tasks such as setting breakpoints and managing JTAG connections.

**README.md**
  - Provides an introduction to the project, build instructions, and links to additional documentation.

**.gitignore**
  - Lists files and directories that should be ignored by version control (e.g., temporary files and build outputs).

Open Source Tools
-----------------
This project is built using a variety of open source tools:

- **LiteX**: A framework for building FPGA designs and integrating soft cores. [GitHub](https://github.com/enjoy-digital/litex)
- **VexRiscv**: A RISC-V core generator based on SpinalHDL. [GitHub](https://github.com/SpinalHDL/VexRiscv)
- **OpenOCD**: Open On-Chip Debugger for JTAG tunneling and debugging. [OpenOCD Website](https://openocd.org/)
- **OpenFPGALoader**: A tool for loading and flashing FPGA board bitstreams. [GitHub](https://github.com/trabucayre/openFPGALoader)
- **GHDL**: A VHDL simulator (and conversion tool for VHDL-to-Verilog) when using toolchains not directly supporting VHDL (e.g., Yosys/nextpnr). [GitHub](https://github.com/ghdl/ghdl)
- **Yosys**: An open source synthesis suite. [GitHub](https://github.com/YosysHQ/yosys)
- **nextpnr**: A portable FPGA place-and-route tool. [GitHub](https://github.com/YosysHQ/nextpnr)

Board-Specific Toolchain Support
-------------------------------
The synthesis and implementation flow varies by board:

- **Lime SDR Mini V2**:
  Based on an ECP5 FPGA, the design for this board can be fully built using open source tools. The flow uses GHDL to convert VHDL to Verilog, followed by Yosys for synthesis and nextpnr for placement and routing.

- **Lime SDR Mini V1**:
  This board uses GHDL to convert VHDL to Verilog. However, its synthesis and place-and-route flow relies on Altera's Quartus toolchain.

- **Lime SDR XTRX**:
  The XTRX board currently uses Xilinx Vivado for synthesis and place-and-route. Since it is based on an Artix7 FPGA, there is potential to transition to the OpenXC7 toolchain in the future—pending support for the PCIe hardblock required for LitePCIe integration.

Enhancements and Flexibility
----------------------------
- **Multi-Board Support**: Dedicated platform and target files for each board simplify the process of switching between different hardware targets.
- **Modular Architecture**: The clear separation between board-specific configurations and core functionality simplifies maintenance and enhances code reuse.
- **Open Source Integration**: The project leverages open source tools throughout the design flow, promoting community collaboration and transparency.
