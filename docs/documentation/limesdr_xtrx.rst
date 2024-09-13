LimeSDR-XTRX
====================

This section provides information about LimeSDR-XTRX gateware


Main block diagram
------------------

Top level file contains these main blocks:
    - **Soft core CPU** - VexRiscv CPU instance
    - **lime_top** - blocks specific to lms7002m transceiver control and data transfer
    - **pcie_phy** - PCIe block with physical interface and DMA
    - **I2C0, I2C1, lms_spi** - communication interfaces to control onboard periphery
    - **flash** - FPGA configuration FLASH memory access

.. figure:: limesdr-xtrx/images/main_block_diagram.svg
  :width: 1000

lime_top
--------


Block **lime_top** is wrapper file for specific lms7002m transceiver control and data transfer blocks. Main blocks are following:
    - :external+limedfb:ref:`lms7002_top <docs/lms7002_top/readme:lms7002_top>` - lms7002 IC phy for sending/receiving digital IQ samples.
    - :external+limedfb:ref:`rx_path_top <docs/rx_path_top/readme:rx_path_top>` - receive path (LMS7002M -> FPGA -> HOST), responsible for packing IQ samples into packets and timestamp generation.
    - :external+limedfb:ref:`tx_path_top <docs/tx_path_top/readme:tx_path_top>` - transmit path (HOST -> FPGA -> LMS7002M), responsible for unpacking received packets into IQ samples and stream synchronization with timestamp.

.. figure:: limesdr-xtrx/images/limetop_block_diagram.svg
  :width: 1000