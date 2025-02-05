Project Structure
=================

Overview
--------
This FPGA gateware project leverages the **LiteX** framework to build and manage top-level designs for multiple FPGA boards. It integrates custom hardware logic (written in VHDL/Verilog) with pre-designed LiteX components such as **LitePCIe**, **VexriscV** softcore CPUs, and various communication interfaces (**I2C**, **SPI**, etc.). The design is modular, scalable, and adaptable to support different hardware targets and for rapid prototyping, easy maintenance, and future extensibility to other platforms.

Repository Structure
--------------------
The repository is organized into distinct directories for board-specific configurations, firmware, gateware, and software. An of the key directories and files is provided below:

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
   ├── software/
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
