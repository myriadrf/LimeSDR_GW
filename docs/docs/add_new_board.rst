Adding a New Board
==================

This guide is tailored for FPGA and RF engineers looking to extend the **LimeSDR_GW** repository (available on GitHub at myriadrf/LimeSDR_GW) for custom SDR platforms or to add support for new boards. It introduces the **LiteX** framework, the backbone of the repository, which uses Python for hardware system description and integration. Unlike conventional HDL-centric processes, LiteX automates routine SoC tasks, allowing a greater focus on system architecture and RF functionalities.

.. figure:: images/limesdr_family.png
   :width: 600
   :align: center
   :alt: LimeSDR Boards Family

The LimeSDR_GW project brings together various SDR board designs in a portable, unified gateware ecosystem. By integrating LiteX with modular RF processing modules from **LimeDFB**, it simplifies maintenance, expansion, and adaptation across diverse FPGA architectures and hardware configurations. For instance, you might use this guide to incorporate an alternative RF transceiver, switch to a different FPGA vendor for cost savings, or prototype an SDR for applications such as 5G research or satellite communication.

This document outlines the process for adapting new boards, including repository organization, key LiteX concepts, and integration of elements like PCIe or USB interfaces. It assumes familiarity with FPGA workflows (synthesis, timing analysis, simulation) and basic Python proficiency.

If anything remains unclear, please submit an issue on GitHub—contributions and insights are highly encouraged!

.. contents:: Table of Contents
   :depth: 3
   :local:

Introduction
------------

.. _pcie_usb_interfacing:

PCIe and USB Interfacing
------------------------

The LimeSDR_GW framework supports two main high-speed host interfaces: PCI Express (PCIe) and USB 3.0. The interface choice depends on the board, required bandwidth, and host compatibility. PCIe offers higher throughput for demanding streaming, while USB provides simpler integration and broader accessibility.

.. figure:: images/usb_pcie_flows.png
   :align: center
   :width: 800
   :alt: USB PCIe Flows

Overview of Interface Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------------+------------------+-----------------------------+
| Board             | Interface        | Notes                       |
+===================+==================+=============================+
| LimeSDR Mini V1   | USB 3.0          | FT601 with LiteX wrapper    |
+-------------------+------------------+-----------------------------+
| LimeSDR Mini V2   | USB 3.0          | Same as V1, Yosys-compatible|
+-------------------+------------------+-----------------------------+
| XTRX              | PCIe             | Uses LitePCIe core          |
+-------------------+------------------+-----------------------------+

USB Interface (FT601)
^^^^^^^^^^^^^^^^^^^^^

The LimeSDR Mini boards (V1 and V2) use the FTDI FT601 chip for USB 3.0 connectivity. This is wrapped in a reusable LiteX USB core with multiple endpoints:

- Control/Status endpoints for configuration, monitoring, and command handling (e.g., GET_INFO, LMS_RST).
- Streaming endpoints for RX/TX I/Q data transfer via FIFO.

The core builds on the original LimeSDR USB HDL but fits seamlessly into LiteX's SoC, using CSRs and Wishbone buses for communication. Firmware manages USB packet processing, FIFO reads/writes, and host interactions.

**Boards using USB:**

- `LimeSDR Mini V1`
- `LimeSDR Mini V2`

PCIe Interface (LitePCIe)
^^^^^^^^^^^^^^^^^^^^^^^^^

The XTRX board uses PCI Express, based on the open-source `LitePCIe` core (at https://github.com/enjoy-digital/litepcie). This enables:

- Memory-mapped (MMAP) access via BAR regions for register control and DMA setup.
- Streaming (DMA) for high-speed RX/TX I/Q data.
- Interrupt support.

LitePCIe integrates closely with LiteX's SoC, including tools for auto-generating Linux drivers. Firmware handles PCIe command processing, and MMAP interactions.

**Board using PCIe:**

- `LimeSDR XTRX`

Testing and Debugging Interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**For USB (FT601):**

- *Basic Functionality:* Use **LimeSuiteGUI** to detect the board and read/write control/status registers.
- *Streaming Test:* Run **LimeSuite** CLI (e.g., `LimeQuickTest`) or GNU Radio with `gr-limesdr` for RX/TX validation.
- *Firmware Validation:* Check Control/Status endpoints for proper packet handling (e.g., via USB analyzers or logs).

**For PCIe (LitePCIe):**

- *MMAP/DMA Test:* Use **LitePCIe utilities** like `litepcie_util`, `litepcie_dma_test`, and `litepcie_probe` for access and performance checks.

**Generic Debugging Tool:**

- *LiteScope:* Integrate this logic analyzer into the SoC to monitor USB/PCIe signals in the FPGA, useful for FSMs, stalls, or behaviors. For details on adding a host bridge to enable LiteScope communication, see the LiteX Wiki page on `Use Host Bridge to control debug a SoC <https://github.com/enjoy-digital/litex/wiki/Use-Host-Bridge-to-control-debug-a-SoC>`_. For instructions on integrating and using LiteScope itself, see the LiteX Wiki page on `Use LiteScope To Debug A SoC <https://github.com/enjoy-digital/litex/wiki/Use-LiteScope-To-Debug-A-SoC>`_.

Tip: Use LiteX BIOS for initial MMAP register checks before higher-level tools.

Toolchains
----------

This section offers a detailed overview of the FPGA synthesis toolchains used for supported boards in the LimeSDR_GW project, ensuring reproducibility and helping new developers set up environments. It also covers the RISC-V firmware toolchain, which LiteX can install automatically for convenience.

FPGA Synthesis Toolchains
^^^^^^^^^^^^^^^^^^^^^^^^^

Different synthesis tools are used based on the FPGA vendor and family:

+-------------------+------------------+-----------------------------+
| Board             | FPGA             | Toolchain                   |
+===================+==================+=============================+
| LimeSDR Mini V1   | Altera MAX10     | Intel Quartus Prime Lite    |
+-------------------+------------------+-----------------------------+
| LimeSDR Mini V2   | Lattice ECP5     | Yosys + nextpnr-ecp5        |
+-------------------+------------------+-----------------------------+
| XTRX              | Xilinx Artix-7   | Xilinx Vivado               |
+-------------------+------------------+-----------------------------+

Notes:

- For **Intel MAX10**, Intel Quartus Prime Lite Edition is sufficient (Quartus Pro is not required).
- For **Lattice ECP5**, the open-source Yosys/nextpnr-ecp5 toolchain is employed.
- For **Xilinx Artix-7**, Xilinx Vivado (WebPACK or Standard edition) is required.

The LiteX build system automatically detects the board and selects the appropriate toolchain, generating project files, constraints, and build scripts tailored to the vendor.

RISC-V Firmware Toolchains
^^^^^^^^^^^^^^^^^^^^^^^^^^

For soft RISC-V CPUs (e.g., VexRiscv or PicoRV32), a RISC-V toolchain compiles the firmware (BIOS or application). LiteX simplifies installation and management:

The recommended toolchain is **riscv64-unknown-elf-gcc** with newlib (no OS), targeting `rv32im` or `rv32ima` based on CPU features.

Installation:

Use LiteX's setup script for automatic installation:

.. code-block:: bash

    ./litex_setup.py init install --toolchain riscv

LiteX handles firmware compilation, linker scripts, and Board Support Packages (BSPs) for the selected CPU, embedding the binary into the FPGA bitstream. Override defaults with `--riscv-cpu` and `--cpu-variant` options if needed.


Gateware Update Mechanism
--------------------------

.. figure:: images/flash_update.png
   :align: center
   :width: 600
   :alt: Flash Update

This section describes the process for updating the FPGA gateware (bitstream). Updates focus on flashing the bitstream to non-volatile storage (Flash), enabling the FPGA to load it automatically on power-up. The soft CPU firmware handles update commands from the host, with LimeSuite as the primary tool for managing the process on the host side.

Overview
^^^^^^^^

Gateware updates use the host interfaces (USB or PCIe) to transfer bitstream data to the soft CPU firmware, which then manages writing to Flash. This ensures updates are persistent and reduces the need for volatile loads. Key concepts include:

- **Firmware Role**: The soft CPU (e.g., VexRiscv) acts as an intermediary, processing host commands to erase Flash sectors, program pages, and handle data integrity.
- **LimeSuite Communication**: The host tool (LimeSuite) initiates updates, sending commands and bitstream segments over the interface, with firmware responding with statuses (e.g., success or error).
- **Multiboot Support**: Allows multiple bitstream images in Flash (e.g., a reliable "golden" image and an update image), with automatic fallback on failure for recovery.
- **Portability**: LiteX abstracts Flash access (via LiteSPI), making the mechanism consistent across boards and FPGA vendors.

This approach minimizes risks during updates and supports shared firmware across all LimeSDR variants.

Update Process
^^^^^^^^^^^^^^

Updates are typically performed using LimeSuite (e.g., via `LimeUtil --update` or GUI features):

- LimeSuite connects to the board over USB/PCIe and sends update commands along with bitstream data in segments.
- The firmware receives these via control endpoints, validates the data, erases relevant Flash areas, and writes the bitstream.
- Additional handling for non-volatile data like VCTCXO DAC values or serial numbers, stored in specific Flash offsets.

For USB-based boards (Mini V1/V2), this uses FT601; for PCIe (XTRX), it leverages LitePCIe.

Multiboot Across FPGAs
^^^^^^^^^^^^^^^^^^^^^^

Multiboot enables safe updates by supporting multiple images in Flash:

- Store a golden image at the base address and updates at an offset.
- On power-up, the FPGA loads the primary image; if it fails (e.g., due to corruption), it falls back to the golden one automatically.
- Vendor differences (e.g., Intel MAX10 uses CFM partitions, Lattice ECP5 uses configuration registers, Xilinx Artix-7 uses ICAP) are handled transparently, with firmware setting boot flags or addresses.

This provides robustness, especially for remote or field updates.

Testing and Recovery
^^^^^^^^^^^^^^^^^^^^

- Post-update verification via LimeSuite (e.g., check firmware version or run diagnostics like LimeQuickTest).
- If issues arise, multiboot falls back to the golden image; JTAG serves as a last-resort recovery.
- Use tools like LiteScope for debugging Flash interactions during development.

Tutorial: Developing New LimeDFB Blocks
---------------------------------------

While the LimeDFB library provides reusable VHDL components for RF data processing (e.g., RX/TX paths, clock domain crossing, FIFO buffers), you may need to develop new blocks for custom RF functionalities or adaptations (e.g., a new signal path for an alternative transceiver). LimeDFB is designed for modularity, so new blocks can be added with minimal disruption, following VHDL best practices and integrating seamlessly with LiteX.

Steps to Develop a New Block

1. **Create the VHDL Source**:

   - Add your VHDL code in a new subdirectory under LimeDFB (e.g., `custom_path_top/src/custom_path_top.vhd` for a custom processing path).
   - Define clear interfaces (e.g., using AXI-Stream for data flows: `s_axis_iq_tdata`, `s_axis_iq_tvalid`, etc.).
   - Use generic parameters for configurability (e.g., `generic (DATA_WIDTH : integer := 12);`).
   - Ensure vendor-neutrality: Avoid FPGA-specific primitives; use inferred logic where possible.


2. **Add Testbenches for Verification**:

   - Create a testbench in the same subdirectory (e.g., `custom_path_top/testbench/custom_path_top_tb.vhd`).
   - Use VHDL assertions and stimulus generation to verify functionality (e.g., simulate input IQ samples and check outputs).
   - Include clock domain crossing tests if applicable.
   - Run simulations with tools like GHDL or ModelSim to validate before synthesis.

3. **Document the Block**:

   - Update `docs/` with an overview: Describe architecture, interfaces, parameters, and usage.
   - Add WaveDrom waveforms for timing (e.g., JSON for input/output signals).
   - Include a block diagram (e.g., using Draw.io or ASCII art) showing data flow.
   - Reference existing LimeDFB docs for style (e.g., RX/TX path explanations).

4. **Wrap for LiteX Integration**:

   - Create a Python wrapper in `LimeSDR_GW/gateware/LimeDFB_LiteX/` (e.g., `custom_filter_top.py`), following the pattern in :ref:`creating_litex_migen_wrappers`.
   - Expose CSRs for control/status if needed.
   - Instantiate in `LimeTop.py` or your target file.
   - Test in simulation and on hardware.

For more on LimeDFB development, consult https://limedfb.myriadrf.org/.

Tutorial: Adding a Custom Board
-------------------------------

This tutorial provides a step-by-step guide to adding support for a custom SDR board to the LimeSDR_GW repository. It builds on concepts from earlier sections, such as :ref:`understanding_litex_boards_platforms_targets` for platform and target definitions, :ref:`creating_litex_migen_wrappers` for integrating HDL modules, and the repository navigation for file placement. We'll use the LimeSDR Mini V2 (files in ``boards/platforms/limesdr_mini_v2_platform.py`` and ``boards/targets/limesdr_mini_v2.py``) as a template, assuming your custom board has similar features (e.g., LMS7002M transceiver, USB/PCIe interface) but with adjusted pinouts, FPGA device, or peripherals. Adapt as needed for your hardware.

Note: While LiteX simplifies FPGA development and board porting through abstractions, it is not magic—significant differences in hardware (e.g., new FPGA vendors, custom interfaces, or complex peripherals) may still require substantial effort and developer experience in areas like timing closure, vendor tools, or HDL integration. For boards closely aligned with existing ones (e.g., minor pin changes), the process is straightforward; otherwise, expect iterative debugging and potential contributions to LiteX if unsupported features arise.

Prerequisites: Ensure you have the required toolchain installed (see Toolchains section) and basic Python/VHDL knowledge.

1. **Prepare the Repository Structure**

   - Fork/clone the LimeSDR_GW repository.
   - Create a new directory if needed (e.g., ``boards/platforms/custom_board/``), but typically place files directly in ``boards/platforms/`` and ``boards/targets/``.
   - Copy the LimeSDR Mini V2 platform and target files as templates:

     .. code-block:: bash

         cp boards/platforms/limesdr_mini_v2_platform.py boards/platforms/custom_board_platform.py
         cp boards/targets/limesdr_mini_v2.py boards/targets/custom_board.py

2. **Define the Platform (Hardware Constraints)**

   Refer to the Platform = Board + Constraints subsection for details on I/O definitions and timing.

   - Edit ``custom_board_platform.py``:

     - Update the FPGA device (e.g., change ``device="10M16SAU169C8G"`` to match your FPGA, like a different MAX10 variant).
     - Modify ``_io`` list: Adjust pin assignments for clocks, LEDs, I2C/SPI, USB/PCIe, LMS7002M signals, etc., based on your board's schematic. Group signals logically (e.g., ``("LMS", 0, ...)``) and set ``IOStandard``/``Misc`` for electrical specs.
     - Add/customize timing constraints in ``do_finalize()`` (e.g., ``add_period_constraint`` for oscillators/clocks).
     - Configure programming (e.g., ``create_programmer`` for JTAG/USB loader).
     - Add platform commands for synthesis options (e.g., optimization modes, flash settings).

   Example snippet (adapted from Mini V2):
     .. code-block:: python

         _io = [
             # Clk (adjust pin/frequency for your board).
             ("LMK_CLK", 0, Pins("H6"), IOStandard("2.5 V")),
             # ... other I/Os like LMS signals, USB FIFO ...
         ]

         class Platform(AlteraPlatform):  # Change to appropriate vendor class (e.g., LatticePlatform).
             default_clk_name   = "LMK_CLK"
             default_clk_period = 1e9/40e6  # Adjust for your clock freq.
             # ... rest as per template ...

3. **Define the Target (SoC Integration)**

   Refer to the Target = SoC Top-Level + Flow Control subsection for SoC wiring.

   - Edit ``custom_board.py``:

     - Import your new platform: ``from boards.platforms.custom_board_platform import Platform``.
     - Update CRG class: Adjust clock domains/frequencies (e.g., PLL setup if needed).
     - In ``BaseSoC``: Customize CPU type/variant, memory sizes, and integrations (e.g., add I2C/SPI masters, LimeTop for RF paths).
     - Connect peripherals: Wire LimeTop to host interface (e.g., FT601 for USB), add UARTBone if desired.
     - Add optional features: LiteScope for debugging, SPI Flash if supported.
     - Update constants (e.g., FIFO sizes, packet widths) for your board's bandwidth needs.

   Example snippet (adapted from Mini V2):
     .. code-block:: python

         class BaseSoC(SoCCore):
             def __init__(self, sys_clk_freq=77.5e6, ...):
                 platform = Platform()  # Your custom platform.
                 SoCCore.__init__(self, platform, sys_clk_freq, ...)
                 self.crg = _CRG(platform, sys_clk_freq)  # Customize clocks.
                 self.limetop = LimeTop(self, platform, ...)  # Adjust params.
                 # ... connect FT601/PCIe, add wrappers ...

   - For RF integration: If using LimeDFB, wrap new/custom modules (see Creating LiteX/Migen Wrappers) and instantiate in LimeTop.

   - Test: Run a simple build to verify syntax and basic constraints (e.g., ``python3 -m boards.targets.custom_board.py --build``). Check for errors in generated files (e.g., constraints) before proceeding.

4. **Add Build/Flash Logic**

   - In the target file's ``main()``: Update argparse for board-specific options (e.g., toolchain, flash commands).
   - Handle multiboot/golden images if applicable (see Gateware Update Mechanism).
   - Build firmware: Ensure ``firmware/`` Makefile points to your board (e.g., via ``env.mak``).

5. **Verify and Debug**

   Follow the Minimal Verification Design subsection below for a basic SoC test.

   - Build: ``python3 custom_board.py --build --toolchain=your_toolchain``.
   - Load/Flash: Use ``--load`` or ``--flash`` args; verify with LimeSuite (e.g., detect board, run LimeQuickTest).
   - Debug: Add LiteScope probes (e.g., for USB/PCIe signals); use JTAG if issues arise.
   - Test RF: Integrate RX/TX paths, validate with GNU Radio scripts from ``tools/``.

6. **Best Practices and Contribution**

   - Follow naming conventions (see File, Module, and Signal Naming Conventions).
   - Ensure portability (see Developing for Portability Across Devices and Vendors).
   - Document: Update ``docs/`` with your board's build instructions/diagrams.
   - Submit a PR: Include platform/target files, tests, and any new wrappers.

If your board uses a different FPGA/vendor, minimal changes to platform/toolchain suffice due to LiteX abstractions. For issues, open a GitHub issue.