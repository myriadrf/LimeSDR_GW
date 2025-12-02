HiperSDR 44xx
=============

This section provides detailed information about the gateware implemented for the HiperSDR 44xx board.

Main Block Diagram
------------------

The top-level file integrates the following main blocks:

- :ref:`Soft core CPU <soft_core_cpu_module>` – VexRiscv CPU instance.
- :ref:`AFE79xx <rf_transceiver>` – RF Transceiver instance.
- :ref:`LimeTop Module <LimeTop_module>` – Wrapper for blocks handling RF transceiver control and data transfer.
- :ref:`PCIe PHY <pcie_phy_module>` – PCIe block with the physical interface and DMA.
- :ref:`I2C0, I2C1, I2C2, I2C3 <i2c_modules>` – I2C Communication interfaces for controlling onboard peripherals.
- :ref:`SPI0, SPI1, SPI2, <spi_modules>` – SPI Communication interfaces for controlling onboard peripherals.
- :ref:`USSPI Flash <flash_module>` – Module for accessing the FPGA configuration FLASH memory.
  

.. drawio-image:: hipersdr-44xx/images/main_block_diagram.drawio
   :align: center
   :format: svg
   :page-index: 0
   :alt: Main block diagram for HiperSDR 44xx

.. _soft_core_cpu_module:

Soft core CPU Module
^^^^^^^^^^^^^^^^^^^^
The CPU module is a ``vexriscv_smp`` core provided by LiteX. It is specified via the ``cpu_type`` parameter for the ``SoCCore`` class, which serves as the parent class for the top-level gateware design.

The source code for the CPU can be found at:
`LiteX VexRiscv SMP core <https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/cpu/vexriscv_smp/core.py>`_


.. _rf_transceiver:

AFE79xx
^^^^^^^

This module is part of LimeDFB and more details can be found in :external+limedfb:ref:`afe79xx <docs/afe79xx/readme:afe79xx>` description. It interfaces with TI JESD IP to exchange sample data with the AFE7901 chip. It also performs resampling, clock
domain crossing and aligns the data for proper integration with other LimeDFB modules.


.. _LimeTop_module:

LimeTop Module
^^^^^^^^^^^^^^
The **LimeTop Module** serves as a wrapper for the RF transceiver control and data transfer blocks. Its main sub-blocks include:

- :ref:`RX Path Top Module <rx_path_top_module>` – Manages the receive path from the RF transceiver to the FPGA and host, packing IQ samples into packets and generating timestamps.
- :ref:`TX Path Top Module <tx_path_top_module>` – Manages the transmit path from the host through the FPGA to the RF transceiver, unpacking IQ sample packets and handling stream synchronization with timestamps.

.. drawio-image:: hipersdr-44xx/images/limetop_block_diagram.drawio
   :align: center
   :format: svg
   :page-index: 0
   :alt: Lime_top block diagram


.. _rx_path_top_module:

RX Path Top Module
^^^^^^^^^^^^^^^^^^
This module is part of LimeDFB and more details can be found in :external+limedfb:ref:`rx_path_top <docs/rx_path_top/readme:rx_path_top>` description. It handles the receive path from the RF Transceiver to the FPGA and host, including IQ sample packetization and timestamp generation.

.. drawio-image:: ../../gateware/LimeDFB/docs/docs/rx_path_top/block_diagram.drawio
   :align: center
   :format: svg
   :page-index: 0
   :alt: Lime_top block diagram

.. _tx_path_top_module:

TX Path Top Module
^^^^^^^^^^^^^^^^^^
This module is part of LimeDFB and more details can be found in :external+limedfb:ref:`tx_path_top <docs/tx_path_top/readme:tx_path_top>` description. This module manages the transmit path from the host through the FPGA to the RF Transceiver, including unpacking of IQ samples and stream synchronization.

.. _pcie_phy_module:

PCIe PHY Module
^^^^^^^^^^^^^^^
The **PCIe PHY** module is an instantiation of the ``USPPCIEPHY`` class from LitePCIe. It provides the physical layer for the PCIe interface, including DMA support.

The source code for LitePCIe is available at:
`LitePCIe on GitHub <https://github.com/enjoy-digital/litepcie>`_

.. _i2c_modules:

I2C Modules
^^^^^^^^^^^
The **I2C0**, **I2C1**, **I2C2**, **I2C3** modules are instances of the ``I2CMaster`` class provided by LiteX. They are used for controlling onboard peripherals via the I2C protocol.

The source code can be found here:
`I2CMaster in LiteX <https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/bitbang.py>`_

.. _spi_modules:

SPI Modules
^^^^^^^^^^^
The **SPI0**, **SPI1**, **SPI2** modules are an instantiation of the ``SPIMaster`` class from LiteX. It handles SPI communication with the LMS8001, AFE7901 and ADF4002 ICs.

Source code:
`SPIMaster in LiteX <https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/spi/spi_master.py>`_

.. _flash_module:

Flash Module
^^^^^^^^^^^^
The **Flash** module is implemented using the ``USSPIFlash`` class provided by LiteX. It enables access to the FPGA configuration FLASH memory.

Source code:
`USSPIFlash in LiteX <https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/spi_flash.py>`_

Gateware Register Reference
---------------------------
The following documentation is automatically generated from the LiteX SoC definitions. It includes the complete **Control and Status Register (CSR)** map, interrupt vector table, and memory regions for the modules described above (PCIe, I2C, SPI, etc.).

:doc:`hipersdr-44xx/litex_doc/index`
