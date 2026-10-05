PPS Disciplined Oscillator (PPSDO)
==================================

Primary Code References
-----------------------
The PPSDO subsystem logic and integration are distributed across the following source files:

* **PPSDO Core Module:** ``gateware/LimePPSDO/src/ppsdo.py``
* **FazyRV Microcontroller RTL:** ``gateware/LimePPSDO/src/FazyRV/``
* **Board Targets:**
    * ``boards/targets/limesdr_xtrx.py``
    * ``boards/targets/limesdr_mini_v2.py``
    * ``boards/targets/limesdr_usb.py``
    * ``boards/targets/hipersdr_44xx.py``

System Overview
---------------
The **PPS Disciplined Oscillator (PPSDO)** subsystem provides autonomous, closed-loop hardware frequency disciplining for LimeSDR platforms equipped with a Voltage-Controlled Temperature-Compensated Crystal Oscillator (VCTCXO) and an external Pulse-Per-Second (PPS) reference signal.

Frequency drift in free-running oscillators can degrade RF synchronization and timing accuracy over time. The PPSDO continuously measures the clock frequency against incoming 1-PPS reference pulses and adjusts the onboard Digital-to-Analog Converter (DAC) to pull the oscillator back to its nominal operating frequency (typically 30.72 MHz) without requiring real-time software intervention on the host CPU.

Key capabilities include:

* **Autonomous Operation:** Embedded soft-core microcontroller (FazyRV) executes the closed-loop disciplining algorithm directly inside the FPGA fabric.
* **Multi-Window Frequency Measurement:** Simultaneously tracks frequency errors across 1-second, 10-second, and 100-second gating intervals for fast acquisition and high-precision steady-state lock.
* **Hardware DAC Steering:** Controls the board's DAC to adjust the VCTCXO frequency tuning voltage.
* **Telemetry & CSR Control:** Exposes real-time frequency error metrics, lock accuracy status, and PPS presence indicators to the host via LiteX CSR registers.

Gateware Architecture & Operating Principles
--------------------------------------------

FazyRV Soft Microcontroller Core
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The PPSDO subsystem instantiates a resource-optimized **FazyRV** 32-bit RISC-V soft processor. The core runs the dedicated disciplining firmware out of embedded FPGA memory and is clocked synchronously from the board's reference clock domain (e.g. 30.72 MHz ``lmk``, ``xo_fpga``, or ``sys``).

Multi-Window Gating & Measurement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Hardware frequency counters measure the number of reference clock cycles between consecutive PPS pulses across three cascaded time windows:

1. **1-Second Interval:** Provides rapid detection of large frequency offsets and quick initial convergence.
2. **10-Second Interval:** Refines frequency estimation and filters out single-pulse PPS jitter.
3. **100-Second Interval:** Delivers fine sub-Hz frequency measurement resolution for optimal long-term stability.

Lock Accuracy States
~~~~~~~~~~~~~~~~~~~~
The PPSDO core reports its lock status via a 2-bit accuracy indicator in the ``status_accuracy`` CSR register:

.. list-table::
   :widths: 15 25 60
   :header-rows: 1

   * - Accuracy Value
     - Lock Level
     - Description
   * - **0**
     - **Unlocked**
     - System is initializing or frequency error exceeds the 1-second tolerance threshold.
   * - **1**
     - **Coarse Lock**
     - Frequency error is within the configured 1-second interval tolerance window.
   * - **2**
     - **Medium Lock**
     - Frequency error is within the configured 10-second interval tolerance window.
   * - **3**
     - **Fine Lock**
     - Frequency error is within the configured 100-second interval tolerance window.

Hardware Plumbing & Board Routing
---------------------------------

PPS input routing, clock domains, and DAC configurations vary across supported board targets:

.. list-table::
   :widths: 22 25 18 15
   :header-rows: 1

   * - Board
     - PPS Input Source
     - Clock Domain
     - DAC Width
   * - **LimeSDR XTRX**
     - Muxed (SYNC header / GNSS chip)
     - ``xo_fpga`` (30.72 MHz)
     - 16-bit
   * - **LimeSDR Mini V2**
     - ``FPGA_GPIO[1]`` (input)
     - ``lmk`` (30.72 MHz)
     - 10-bit
   * - **LimeSDR USB**
     - ``FPGA_GPIO[7]`` (connector **J13**)
     - ``lmk`` (30.72 MHz)
     - 8-bit
   * - **HiperSDR 44xx**
     - Muxed (1PPS input pad / GNSS chip)
     - ``sys`` domain
     - 16-bit

Software Register Reference (LiteX CSR)
---------------------------------------
The PPSDO subsystem provides the following configuration and telemetry registers accessible through the native LiteX CSR bus (typically mapped at base address ``0xf000b000`` or ``0xf000f000`` depending on target layout):

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - CSR Register
     - Type
     - Description
   * - **PPSDO_ENABLE**
     - R/W
     - Subsystem enable (bit 0 = 1 enables the PPSDO core).
   * - **PPSDO_CONFIG_ONE_S_TARGET**
     - R/W
     - Nominal target cycle count for 1-second window (e.g. 30,720,000 for 30.72 MHz).
   * - **PPSDO_CONFIG_ONE_S_TOL**
     - R/W
     - Acceptable cycle error tolerance for 1-second window.
   * - **PPSDO_CONFIG_TEN_S_TARGET**
     - R/W
     - Nominal target cycle count for 10-second window (e.g. 307,200,000).
   * - **PPSDO_CONFIG_TEN_S_TOL**
     - R/W
     - Acceptable cycle error tolerance for 10-second window.
   * - **PPSDO_CONFIG_HUNDRED_S_TARGET**
     - R/W
     - Nominal target cycle count for 100-second window (e.g. 3,072,000,000).
   * - **PPSDO_CONFIG_HUNDRED_S_TOL**
     - R/W
     - Acceptable cycle error tolerance for 100-second window.
   * - **PPSDO_STATUS_ONE_S_ERROR**
     - RO
     - Signed cycle error measured over the most recent 1-second interval.
   * - **PPSDO_STATUS_TEN_S_ERROR**
     - RO
     - Signed cycle error measured over the most recent 10-second interval.
   * - **PPSDO_STATUS_HUNDRED_S_ERROR**
     - RO
     - Signed cycle error measured over the most recent 100-second interval.
   * - **PPSDO_STATUS_DAC_TUNED_VAL**
     - RO
     - Current DAC tune word applied to the VCTCXO.
   * - **PPSDO_STATUS_ACCURACY**
     - RO
     - Current accuracy level (0 = Unlocked, 1 = 1s lock, 2 = 10s lock, 3 = 100s lock).
   * - **PPSDO_STATUS_PPS_ACTIVE**
     - RO
     - Active indicator asserted when PPS pulses are detected.
   * - **PPSDO_STATUS_STATE**
     - RO
     - Internal FSM state of the disciplining core.

Build Options
-------------
On supported board targets, the PPSDO subsystem can be excluded at synthesis time using the ``--no-ppsdo`` command-line option:

.. code-block:: bash

   python3 -m boards.targets.limesdr_usb --build --no-ppsdo

Excluding PPSDO reduces FPGA logic element (LE) utilization and block RAM consumption when PPS disciplining is not required.
