=========================
Project Structure
=========================

Overview
========
This FPGA gateware project leverages the **LiteX** framework to create and manage the top-level design for various FPGA boards. The project integrates both custom application-specific logic, written in VHDL/Verilog, and pre-designed LiteX components such as **LitePCIe**, **VexriscV** softcore CPUs, and communication interfaces (**I2C**, **SPI**). Each FPGA board has its own unique platform and top-level files, allowing for flexibility and scalability across different hardware platforms.

An overview of the folders and files contained in the project repository can be found in this section. The structure is organized to facilitate FPGA gateware development.

Root Directory
==============
The main directories and files are organized as follows:

.. code-block:: bash

   / (root)
   ├── docs/
   ├── firmware/
   ├── gateware/
   ├── software/
   ├── digilent_hs2.cfg
   ├── fairwaves_xtrx_platform.py
   ├── limesdr_xtrx.cfg
   ├── limesdr_xtrx.py
   ├── limesdr_xtrx_platform.py
   ├── riscv_jtag_tunneled.tcl
   ├── README.md
   └── .gitignore

Directory Details
=================

doc/
----
This directory contains project documentation files. These include design descriptions, user manuals, and guides for using the project.

firmware/
---
This directory contains firmware code for softcore CPU in LimeSDR GW project. 

gateware/
---------
The gateware directory contains FPGA-related files, such as Verilog or VHDL code. These files define the hardware logic and structure programmed into the FPGA chip. 

software/
-------------
This folder includes software applications that interface with or control the hardware. Any code or programs to communicate with the FPGA or to control peripherals via drivers.

File Details
============

digilent_hs2.cfg
----------------
A configuration file for the Digilent HS2 JTAG programmer. This file contains settings that enable communication between the programmer and the FPGA for debugging and flashing.


fairwaves_xtrx_platform.py
--------------------------

A Python script that defines platform-specific configurations for the Fairwaves XTRX. It includes specific pin mappings, interfaces, and system setup details for this platform, used in the gateware build process.

limesdr_xtrx.cfg
----------------

A configuration file for the LimeSDR XTRX board and FT2232H based programmer.

limesdr_xtrx.py
---------------

Python file for LiteX project building and configuration.

limesdr_xtrx_platform.py
------------------------

A Python script that defines platform-specific configurations for the LimeSDR XTRX board. 

riscv_jtag_tunneled.tcl
-----------------------

A TCL script that handles tunneling for debugging via JTAG using a RISC-V processor. This script may be used to automate specific tasks in the debugging process, like setting breakpoints or managing JTAG connections.

README.md
---------
The `README.md` file provides an overview of the project, explaining its purpose, how to use it, and links to further documentation.

.gitignore
----------
Specifies which files and directories should be ignored by version control, typically used to exclude build outputs or temporary files.