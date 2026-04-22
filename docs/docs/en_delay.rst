Synchronized Stream Start (en_delay)
====================================

.. note::
    This document refers to RX and TX controls multiple times. While the appropriate registers exist for both RX and TX, most boards use RX controls for both stream directions.

Primary Code References
-----------------------
The logic described below is implemented in the following source files:

* **FPGACfg Module:** `gateware/fpgacfg.py`
* **LimeTop Module:** `gateware/LimeTop.py`
* **Board Targets:** `boards/targets/limesdr_xtrx.py`, `boards/targets/ssdr.py`

System Overview
---------------
The ``en_delay`` feature allows synchronized streaming start by delaying the actual internal RX and TX enable signals until a specific trigger condition is met (typically a PPS rising edge).

When this feature is enabled, setting the standard RX or TX enable bits via CSRs does not immediately start the stream. Instead, the gateware waits for the selected delay signal to pulse high before asserting the internal enable signal.

Gateware Implementation
----------------------
The feature is conditionally included in the ``FPGACfg`` module if ``soc_has_timesource`` is set to ``True``.

.. note::
   This feature requires the top-level board target to assign the appropriate signals to ``fpgacfg.tx_en_delay_signal`` and ``fpgacfg.rx_en_delay_signal``.

PPS Signal
-----------------------------
The core trigger for synchronized operations is the ``pps_internal`` signal, which is a board-level signal selected from multiple potential sources via CSRs.

PPS Source Selection
~~~~~~~~~~~~~~~~~~~~
On supported boards, the PPS source is typically controlled via the CSR ``periphcfg.PERIPH_INPUT_SEL_0[0]``.

.. list-table::
   :widths: 20 20 60
   :header-rows: 1

   * - Board
     - Value
     - Selected Source
   * - **LimeSDR-XTRX**
     - **0b01**
     - **SYNC Header** (``synchro_pads.pps_in``)
   * -
     - **Other**
     - **GNSS Chip** (``gps_pads.pps``)
   * - **SSDR**
     - **0b01**
     - **GPIO** (``gpio.GPIO_IN_VAL[2]``)
   * -
     - **Other**
     - **Disabled** (Logic 0)

Internal Distribution
~~~~~~~~~~~~~~~~~~~~~
Once selected, ``pps_internal`` is distributed to several key modules:

* **RX/TX Datapaths:** Typically re-synchronized to the ``lms_rx`` clock domain to generate ``pps_rising`` for timestamping and ``en_delay`` logic.
* **ZDAParser:** Used to gate UTC time validity.
* **PPSDO:** Provides the reference for the frequency-locked loop (FLL).
* **VCTCXO Tamer:** Used for disciplining the board's master oscillator.

Delay Signals
~~~~~~~~~~~~~
The ``en_delay`` feature in ``FPGACfg`` uses two delay signals derived from these sources:

* **Signal 0:** Connected to the synchronized PPS rising edge (``pps_rising``).
* **Signal 1:** Connected to the "PPS and Valid" condition (``pps_rising`` AND ``time_valid``).

Control Modes
~~~~~~~~~~~~~
The delay behavior is controlled via ``rx_delay_mode`` and ``tx_delay_mode`` CSRs in the ``LimeTop`` module.

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Mode Value
     - Description
   * - **0**
     - **No Delay:** Stream starts immediately when enable bit is set.
   * - **1**
     - **Delay until PPS:** Stream starts on the next PPS pulse after enable bit is set.
   * - **2**
     - **Delay until PPS and Valid:** Stream starts on the next PPS pulse where time is valid.

Software Register Reference
---------------------------
The following SPI registers (mapped via firmware LMS64C protocol) are used to configure and monitor the synchronized stream start feature.

.. list-table::
   :widths: 15 15 20 50
   :header-rows: 1

   * - SPI Address
     - Name
     - Valid Values
     - Description
   * - **0x000A**
     - **CONTROL**
     - bit 0: RX_EN
       bit 1: TX_EN
     - Standard stream enable register. When ``en_delay`` is active, these bits arm the synchronization logic.
   * - **0x00CA**
     - **PPS_SEL**
     - 0, 1, 2, ...
     - Selects the source for ``pps_internal``. See "PPS Source Selection" for board-specific values.
   * - **0x0281**
     - **RX_DELAY**
     - 0, 1, 2
     - Controls RX start delay mode:

       * **0**: No Delay
       * **1**: Delay until PPS
       * **2**: Delay until PPS and Valid
   * - **0x0282**
     - **TX_DELAY**
     - 0, 1, 2
     - Controls TX start delay mode:

       * **0**: No Delay
       * **1**: Delay until PPS
       * **2**: Delay until PPS and Valid

Operational Behavior
--------------------
1. **PPS Source Selection:** Configure the PPS source via SPI address **0x00CA** (e.g., set to 1 for SYNC header on XTRX).
2. **Setup Delay Mode:** Software configures the desired delay mode (e.g., Mode 1 for PPS-sync) via SPI addresses **0x0281** (RX) or **0x0282** (TX).
3. **Enable Stream:** Software sets the RX or TX enable bit in the standard control register (Address **0x000A**).
4. **Waiting:** Internal gateware logic holds the actual stream enable signal at ``0``.
5. **Trigger:** Upon the rising edge of the selected trigger signal (e.g., PPS), the internal enable signal is asserted and latched high.
6. **Streaming:** Data streaming begins.
7. **Disable:** Setting the enable bit to ``0`` in register **0x000A** immediately stops the stream and resets the delay logic.
