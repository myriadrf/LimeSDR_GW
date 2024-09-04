.. toctree::
   :maxdepth: 3
   :hidden:

   Introduction <self>
   documentation/gw_description

Introduction
========================

LimeSDR_GW project
-------------------------------

The LimeSDR family of boards, known for their flexibility and high performance in Software Defined Radio (SDR) applications, rely on sophisticated FPGA gateware to manage the seamless integration of various hardware components. This project focuses on the development and customization of FPGA gateware for LimeSDR boards using the LiteX framework, a versatile toolchain designed to facilitate the creation and integration of reusable RTL (Register-Transfer Level) blocks. More information about LiteX Framework can be found `here <https://github.com/enjoy-digital/litex>`_ .

LiteX serves as the backbone of this project, providing a high-level platform that simplifies the complex task of connecting RTL blocks within the FPGA. By leveraging LiteX, we can efficiently design, simulate, and deploy gateware that caters to the specific needs of LimeSDR boards, ensuring optimal performance and flexibility across a wide range of applications. The LiteX framework not only supports the integration of standard components but also allows for the customization of interfaces, protocols, and data processing units, making it an ideal choice for developing sophisticated SDR solutions.

This project aims to create a modular, extensible gateware architecture that can be easily adapted to various configurations and use cases within the LimeSDR ecosystem. Through this approach, we seek to enhance the capabilities of LimeSDR boards, enabling them to meet the growing demands of modern wireless communication systems.

Supported boards
-------------------------------

Currently supported LimeSDR Family boards are listed in the Table 1.

.. table:: Table 1. Board components

  +----------------------------------------------+-----------------------+--------------------------------------------------------------------------------+
  | **Board**                                    | **Version**           | **Description**                                                                |
  +----------------------------------------------+-----------------------+--------------------------------------------------------------------------------+
  | LimeSDR-XTRX                                 | Starting from v1.2    | Small form factor mini PCIe expansion SDR board.                               |
  +----------------------------------------------+-----------------------+--------------------------------------------------------------------------------+


