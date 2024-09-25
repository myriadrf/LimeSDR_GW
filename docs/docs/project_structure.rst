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
   ├── boards/
      ├── targets/
      ├── platforms/
      ├── prog/
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
   ├── README.rst
   └── .gitignore

Directory Details
=================

boards/ 
-------

Contans targets/ subdirectory with project build scripts, platform/ subdirectory with platform-specific I/O constraints, prog/ subdirectory with programming cable configuration files.

doc/
----

This directory contains project documentation files. These include design descriptions, user manuals, and guides for using the project.

firmware/
---------

This directory contains firmware code for softcore CPU in LimeSDR GW project. 

gateware/
---------

The gateware directory contains FPGA-related files, such as Verilog or VHDL code. These files define the hardware logic and structure programmed into the FPGA chip. 

software/
-------------

This folder includes software applications that interface with or control the hardware. Any code or programs to communicate with the FPGA or to control peripherals via drivers.

File Details
============

riscv_jtag_tunneled.tcl
-----------------------

A TCL script that handles tunneling for debugging via JTAG using a RISC-V processor. This script may be used to automate specific tasks in the debugging process, like setting breakpoints or managing JTAG connections.

README.rst
---------
The `README.rst` file provides an overview of the project, explaining its purpose, how to use it, and links to further documentation.

.gitignore
----------
Specifies which files and directories should be ignored by version control, typically used to exclude build outputs or temporary files.
