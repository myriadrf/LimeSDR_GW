.. toctree::
   :maxdepth: 3
   :hidden:

   Introduction <self>
   docs/project_structure
   docs/gw_description
   docs/build_project
   docs/modify_project

Introduction
============

LimeSDR GW project
------------------

The LimeSDR family of boards, renowned for their flexibility and high performance in Software Defined Radio (SDR) applications, rely on sophisticated FPGA gateware to seamlessly integrate various hardware components. This project focuses on developing and customizing FPGA gateware for LimeSDR boards using the `LiteX framework <https://github.com/enjoy-digital/litex>`_, a versatile toolchain for creating and integrating reusable RTL blocks.

Leveraging LiteX as its backbone, the project efficiently connects multiple RTL blocks within the FPGA, enabling tailored design, simulation, and deployment of gateware. This approach supports rapid prototyping, straightforward maintenance, and future extensibility across diverse hardware targets.

Supported Boards
----------------

.. table:: Supported Boards

   +----------------------+---------------------+-----------------------------------------------+
   | **Board**            | **Hardware Version**| **Description**                               |
   +======================+=====================+===============================================+
   | LimeSDR XTRX         | Starting from v1.2  | Small form factor mini PCIe SDR board.        |
   +----------------------+---------------------+-----------------------------------------------+
   | LimeSDR Mini V1      | v1.x                | Small form factor USB SDR board.              |
   +----------------------+---------------------+-----------------------------------------------+
   | LimeSDR Mini V2      | v2.x                | Small form factor USB SDR board.              |
   +----------------------+---------------------+-----------------------------------------------+

Board-Specific Toolchain Support
--------------------------------
Each board uses a toolchain tailored to its FPGA architecture and design constraints:

- **LimeSDR XTRX**: Uses Xilinx Vivado for synthesis and place-and-route, optimizing performance for Artix7 devices. Future support with an open source flow (OpenXC7) is anticipated.
- **LimeSDR Mini V1**: Utilizes GHDL for VHDL-to-Verilog conversion in combination with Altera's Quartus for synthesis and implementation, meeting its specific FPGA requirements.
- **LimeSDR Mini V2**: Fully supports an open source flow using GHDL, Yosys, and nextpnr, demonstrating the potential for a completely open source development process.

Open Source Tools
-----------------

The project leverages a suite of open source tools to ensure flexibility and community-driven development:

.. table:: Open Source Tools

   +----------------------+------------------------------------------------------------+
   | **Tool**             | **Link**                                                   |
   +======================+============================================================+
   | LiteX                | https://github.com/enjoy-digital/litex                     |
   +----------------------+------------------------------------------------------------+
   | VexRiscv             | https://github.com/SpinalHDL/VexRiscv                      |
   +----------------------+------------------------------------------------------------+
   | OpenOCD              | https://openocd.org/                                       |
   +----------------------+------------------------------------------------------------+
   | OpenFPGALoader       | https://github.com/trabucayre/openFPGALoader               |
   +----------------------+------------------------------------------------------------+
   | GHDL                 | https://github.com/ghdl/ghdl                               |
   +----------------------+------------------------------------------------------------+
   | Yosys                | https://github.com/YosysHQ/yosys                           |
   +----------------------+------------------------------------------------------------+
   | nextpnr              | https://github.com/YosysHQ/nextpnr                         |
   +----------------------+------------------------------------------------------------+

These tools enable the project to perform everything from synthesis and simulation to debugging and programming, all while maintaining a commitment to open source development where possible.
