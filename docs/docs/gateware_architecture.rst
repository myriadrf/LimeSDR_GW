Gateware Architecture
=====================

The LimeSDR_GW architecture is organized into four top-level subsystems:

- :ref:`HIF (Host Interface) <gw_arch_hif>` - host connectivity over PCIe or USB
- :ref:`CPU <gw_arch_cpu>` - embedded control and firmware execution
- :ref:`LimeTOP <gw_arch_limetop>` - RF data path and LMS7002 interface
- :ref:`PSS (Periphery Support Subsytem) <gw_arch_pss>` - board peripherals and support logic

Additional topics:

- :ref:`Interconnect <gw_arch_interconnect>`
- :ref:`External interfaces <gw_arch_external_if>`

The design separates the **control plane** from the **data plane**. Control and status traffic
uses a shared **CSR bus**, while high-throughput RX/TX sample transport uses dedicated
**streaming paths** between the host interface and the RF path.

.. figure:: images/LimeSDR_GW_Architecture.drawio.svg
   :align: center
   :width: 1000
   :alt: LimeSDR_GW top-level architecture

.. note::

   This diagram shows the **target top-level architecture** of LimeSDR_GW. Exact block contents and
   interconnect details may vary slightly between board targets and implementation stages.

.. _gw_arch_domains:

Architectural Domains
---------------------

.. _gw_arch_hif:

HIF - Host Interface Subsystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The **HIF** subsystem is the entry point from the host into the FPGA. The LimeSDR_GW framework supports 
two main high-speed host interfaces: PCI Express (PCIe) and USB 3.0. The interface choice depends on the 
board, required bandwidth, and host compatibility. PCIe offers higher throughput for demanding streaming, 
while USB provides simpler integration and broader accessibility.

Its main responsibilities are:

- host-visible control and status access,
- one or more high-throughput streaming channels,
- forwarding control transactions into the internal CSR bus,
- forwarding interrupts or events toward the host.

HIF provides a consistent system-level host interface, even though the transport implementation
differs between PCIe- and USB-based targets.

.. rubric:: USB Interface (FT601)

Boards such as LimeSDR Mini variants use the FTDI FT601 chip for USB 3.0 connectivity. This is
wrapped in a reusable LiteX USB core with multiple endpoints:

- Control/Status endpoints for configuration, monitoring, and command handling (e.g., GET_INFO,
  LMS_RST).
- Streaming endpoints for RX/TX I/Q data transfer via FIFO.

The core builds on the original LimeSDR USB HDL but fits seamlessly into LiteX's SoC, using CSRs
and Wishbone buses for communication. Firmware manages USB packet processing, FIFO reads/writes,
and host interactions.

.. rubric:: PCIe Interface (LitePCIe)

Boards such as LimeSDR XTRX use PCI Express, based on the open-source `LitePCIe` core (at
https://github.com/enjoy-digital/litepcie). This enables:

- Memory-mapped (MMAP) access via BAR regions for register control and DMA setup.
- Streaming (DMA) for high-speed RX/TX I/Q data.
- Interrupt support.

LitePCIe integrates closely with LiteX's SoC, including tools for auto-generating Linux drivers.
Firmware handles PCIe command processing, and MMAP interactions.

.. _gw_arch_cpu:

CPU - Embedded Control Subsystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The **CPU** subsystem executes firmware and coordinates initialization, configuration, monitoring,
and interrupt handling across the design.

At the top level, it includes:

- ISR handling,
- board-support logic,
- peripheral drivers,
- LMS64 communication/control support.

The CPU communicates with the rest of the architecture primarily through the **CSR bus** and acts
as the central control orchestrator for the platform.

.. _gw_arch_limetop:

LimeTOP - RF Processing Subsystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**LimeTOP** is the main RF-facing subsystem. It connects host-side sample streaming to the external
**LMS7002 digital interface**.

At the top level, it includes:

- the **rxtx** path,
- **lms7002_top**,
- a local **Event Manager**.

Its main functions are:

- RX/TX streaming through dedicated data paths,
- LMS7002 digital interfacing,
- RF-path control and board-specific integration,
- generation of RF-related events and interrupts.

LimeTOP is the main **data-plane** subsystem. The CPU configures it, but the sample stream flows
directly between **HIF** and **LimeTOP**.

.. _gw_arch_pss:

PSS - Peripheral Support Subsystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The **PSS** subsystem groups the lower-speed board-management and peripheral interfaces that sit
outside the main RF streaming path.

It typically includes:

- **GPIO**
- **SPI**
- **I2C**
- **CLK CNTRL**
- a local **Event Manager**

PSS isolates board-specific support logic from the core host and RF data-path logic, which improves
reuse across different targets.

.. _gw_arch_interconnect:

Interconnect
------------

The architecture uses three main interconnect mechanisms:

**CSR bus**
   The shared control/status backbone between **HIF**, **CPU**, **LimeTOP**, and **PSS**. It is
   used for register access, status readback, and peripheral control.

**Streaming paths**
   Dedicated high-bandwidth data paths between **HIF** and **LimeTOP** for RX/TX sample transport.

**IRQ/event paths**
   Event-driven signaling between subsystems. Local **Event Manager** blocks collect subsystem
   events and assert interrupts toward the CPU and, where needed, toward the host interface.

.. _gw_arch_external_if:

External Interfaces
-------------------

At the system boundary, the architecture exposes the following main interfaces:

- **PCIe/USB Host**
- **LMS7002 Digital**
- **GPIO Header**
- **SPI Peripheral**
- **I2C Peripheral**

These interfaces reflect the two main roles of the FPGA design:

- high-speed SDR data transport and processing,
- lower-speed board-management and peripheral control.

Board Implementations
---------------------

The architecture described above is common across supported platforms, while module composition and
integration details vary by board implementation.

For board-specific architecture views and implementation details, see:

- :doc:`limesdr_xtrx`
- :doc:`limesdr_mini_v1`
- :doc:`limesdr_mini_v2`
- :doc:`HiperSDR_44xx`
- :doc:`ssdr_rev2`
