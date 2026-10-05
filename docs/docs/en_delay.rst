Synchronized Stream Start (en_delay)
====================================

.. note::
    This document refers to RX and TX controls multiple times. While independent registers exist for both RX and TX, most applications utilize RX controls to synchronously gate both stream directions (via ``tx_sync_mode``).

Primary Code References
-----------------------
The synchronized stream start logic is implemented across the following source files:

* **Stream Start Controller:** ``gateware/LimeTop.py`` (``StreamStartController`` class)
* **FPGA Configuration Module:** ``gateware/fpgacfg.py``
* **Board Targets:**
    * ``boards/targets/limesdr_xtrx.py``
    * ``boards/targets/hipersdr_44xx.py``
    * ``boards/targets/ssdr.py``

System Overview
---------------
The **Synchronized Stream Start (en_delay)** feature enables phase-aligned, deterministic streaming across single or multiple SDR devices by delaying internal RX and TX enable assertions until a specified hardware trigger event occurs (such as a PPS rising edge or external synchronization pulse).

When synchronized streaming is armed, software assertions of the standard RX or TX enable bits do not immediately start the datapath. Instead, the gateware holds the internal enable signals low until the configured trigger condition is satisfied, at which point streaming begins synchronously on the exact same clock cycle.

Gateware Architecture & Operating Principles
--------------------------------------------

StreamStartController
~~~~~~~~~~~~~~~~~~~~~
Inside ``LimeTop``, the ``StreamStartController`` module coordinates stream arming, input clock-domain synchronization, rising-edge detection, and output latch generation. It supersedes legacy gating muxes in ``fpgacfg.py``, which now serves solely to supply the raw software enable requests (``rx_en_req`` and ``tx_en_req``) via control register ``0x000A``.

Signal Flow & Functional Stages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The synchronization process operates across four sequential stages:

1. **Host Arming Request:** The host software configures the desired delay mode (registers ``0x0281`` / ``0x0282`` or native LiteX CSRs) and asserts the raw stream enable bits (``rx_en_req`` / ``tx_en_req``) in ``fpgacfg`` register ``0x000A``.
2. **Clock Synchronization & Edge Detection:** External trigger signals (PPS references or SYNC header inputs) are synchronized into the FPGA system clock domain (``sys``). Internal edge detectors flag the arrival of the active rising edge.
3. **Trigger Evaluation & Gating:** The controller evaluates whether the configured trigger condition (immediate, PPS edge, PPS valid, or external trigger) is satisfied. Internal enable latches remain held low until the condition is met.
4. **Synchronous Enable & Datapath Release:** When the trigger condition fires, the controller asserts ``rx_en`` (and ``tx_en`` when ``tx_sync_mode = 1``) in the exact same clock cycle. This releases the LMS7002 / AFE RF datapaths simultaneously, ensuring phase alignment and deterministic sample alignment.

Trigger Timing & Waveform Analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The following digital timing diagram illustrates the cycle-accurate synchronization sequence when armed in **Mode 1** (``START_ON_PPS``) with ``tx_sync_mode = 1``:

1. The host software asserts ``rx_en_req`` (and optionally ``tx_en_req``) via control register ``0x000A``.
2. Stream enables are held low while the controller synchronizes inputs and waits for the trigger condition.
3. Upon detecting ``pps_rising`` (or external trigger), the controller synchronously asserts ``rx_en`` and ``tx_en`` on the exact same clock cycle.
4. Downstream RF datapaths release packetizers simultaneously and the board outputs a synchronization pulse on ``synchro.pps_out`` to cascaded receivers.

.. wavedrom::
   :caption: StreamStartController Cycle-Accurate Start Waveform

   { "signal": [
     { "name": "sys_clk",         "wave": "p..............." },
     { "name": "rx_en_req",       "wave": "01.............." },
     { "name": "pps_input",       "wave": "0...10.........." },
     { "name": "pps_rising",      "wave": "0....10........." },
     { "name": "rx_en (sync)",    "wave": "0.....1........." },
     { "name": "tx_en (sync)",    "wave": "0.....1........." },
     { "name": "rx_stream",       "wave": "x.....3333333333", "data": ["D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9"] },
     { "name": "tx_stream",       "wave": "x.....4444444444", "data": ["D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9"] },
     { "name": "synchro.pps_out", "wave": "0.....1........." }
   ],
   "head": {
     "text": "Arm Request  -->  Trigger Rising Edge  -->  Synchronous Latch  -->  Aligned Streaming",
     "tick": 0
   }}

Start Modes
~~~~~~~~~~~
The controller supports four distinct trigger modes for both RX (``rx_delay_mode``) and TX (``tx_delay_mode``):

.. list-table::
   :widths: 15 25 60
   :header-rows: 1

   * - Mode Value
     - Name
     - Operational Behavior
   * - **0** (``0b00``)
     - **START_IMMEDIATE**
     - Stream starts immediately when the host asserts the enable request bit (no delay).
   * - **1** (``0b01``)
     - **START_ON_PPS**
     - Arming the stream delays start until the next detected rising edge of the PPS reference.
   * - **2** (``0b10``)
     - **START_ON_PPS_VALID**
     - Stream starts on the next PPS rising edge only if the time source indicates valid lock (``pps_valid = 1``).
   * - **3** (``0b11``)
     - **START_ON_EXT_TRIGGER**
     - Stream starts on the rising edge of an external synchronization trigger input (e.g., SYNC header).

TX Synchronization with RX (``tx_sync_mode``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To prevent phase skew between transmit and receive paths, the ``tx_sync_mode`` register controls whether TX starts synchronously with RX:

* ``tx_sync_with_rx = 1`` **(Default):** TX follows RX. Transmit streaming automatically starts in the exact same clock cycle that RX starts, ignoring independent TX delay mode settings.
* ``tx_sync_with_rx = 0``: TX operates completely independently of RX, using ``tx_en_req`` and its own ``tx_delay_mode`` configuration.

Hardware Plumbing & Board-Specific Routing
------------------------------------------

Physical trigger sources, hardware connectors, and synchronization outputs vary across supported board targets. Rather than referencing internal FPGA nets, the table and subsections below document the ultimate physical hardware signal paths and runtime multiplexers:

.. list-table::
   :widths: 16 26 18 20 10 10
   :header-rows: 1

   * - Board
     - Ultimate PPS Source(s)
     - External Trigger (Mode 3)
     - Synchronization Output
     - Supported Modes
   * - **LimeSDR XTRX**
     - Selectable via ``0x00CA`` (``PERIPH_INPUT_SEL_0[0]``):
       
       * Primary: On-board GNSS module 1PPS (``gps_pads.pps``, pin **P3**)
       * Secondary: External SYNC header (``synchro_pads.pps_in``, pin **M3** / GPIO0)
     - *Not routed* (Synchronize via Mode 1/2 using SYNC header PPS)
     - External SYNC header (``synchro_pads.pps_out``, pin **L3** / GPIO1, PPS pass-through)
     - 0, 1, 2
   * - **HiperSDR 44xx**
     - On-board GNSS receiver 1PPS line (pin **P3**; ``pps_valid`` tied to ``1``)
     - External SYNC connector input (``synchro.pps_in``, pin **M3** / GPIO0)
     - External SYNC connector output (``synchro.pps_out``, pin **L3** / GPIO1, driven by ``rx_en``)
     - 0, 1, 2, 3
   * - **sSDR rev2**
     - External GPIO connector PPS input pin (``gpio.GPIO_IN_VAL[6]``)
     - *Not routed*
     - Board GPIO pins
     - 0, 1

Board-Specific Plumbing Details
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

LimeSDR XTRX
^^^^^^^^^^^^
* **PPS Input Multiplexing:** The active PPS reference is dynamically selected by the register ``periphcfg.PERIPH_INPUT_SEL_0[0]`` (SPI register ``0x00CA``). Setting bit 0 to ``0b1`` selects the external multi-pin SYNC header input (pin **M3** / ``PPSI_GPIO0``), while setting bit 0 to ``0b0`` (default) selects the on-board GNSS module 1PPS output (pin **P3**).
* **Time Validity (Mode 2):** The on-board GNSS UART stream is decoded by the gateware NMEA parser (``ZDAParser``). The parser asserts ``pps_valid`` upon receiving valid ``$GPZDA``/``$GNZDA`` time sentences, allowing Mode 2 (``START_ON_PPS_VALID``) operation.
* **Daisy-Chaining / Sync Output:** The external SYNC connector output (pin **L3** / ``PPSO_GPIO2``) is driven by a tri-state buffer (controlled by registers ``0x00C0`` / ``BOARD_GPIO_OVRD[1]`` and ``0x00C4`` / ``BOARD_GPIO_DIR[1]``). When enabled, it outputs the active internal PPS reference, allowing downstream SDRs to be synchronized from a single master reference.

HiperSDR 44xx
^^^^^^^^^^^^^
* **PPS Reference:** Connected directly to the on-board GNSS receiver 1PPS output line (FPGA pin **P3**). Time validity (``pps_valid``) is tied high (``1``).
* **External Stream Trigger (Mode 3):** The external SYNC header input (``synchro.pps_in``, FPGA pin **M3** / GPIO0) is routed directly to ``limetop.ext_stream_trigger``. Asserting a rising edge on this pin immediately triggers stream starts when configured in Mode 3 (``START_ON_EXT_TRIGGER``).
* **Cascade Synchronization Output:** The external SYNC connector output (``synchro.pps_out``, FPGA pin **L3** / GPIO1) is driven directly by ``stream_start_controller.rx_en``. When streaming starts on the master board, pin L3 transitions high, triggering downstream cascaded boards connected via their Mode 3 inputs.

sSDR rev2
^^^^^^^^^
* **PPS Reference:** Connected to the external GPIO connector pin mapped to ``gpio.GPIO_IN_VAL[6]``. Mode 1 (``START_ON_PPS``) gates streaming on rising edges of this external line. Mode 2 and Mode 3 are not active on this target.

Software Register Reference
---------------------------

Native LiteX CSR Registers (``LimeTop``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
When accessing the gateware directly via the LiteX CSR bus, the ``StreamStartController`` exposes the following configuration registers (prefixed with the module hierarchy ``LIMETOP_STREAM_START_CONTROLLER_``):

.. list-table::
   :widths: 40 15 45
   :header-rows: 1

   * - CSR Register
     - Type
     - Description
   * - **LIMETOP_STREAM_START_CONTROLLER_RX_DELAY_MODE**
     - R/W (2-bit)
     - RX stream start mode (0 = Immediate, 1 = On PPS, 2 = On PPS Valid, 3 = On External Trigger).
   * - **LIMETOP_STREAM_START_CONTROLLER_TX_DELAY_MODE**
     - R/W (2-bit)
     - TX stream start mode (0 = Immediate, 1 = On PPS, 2 = On PPS Valid, 3 = On External Trigger). Active only when ``tx_sync_mode`` is set to 0.
   * - **LIMETOP_STREAM_START_CONTROLLER_TX_SYNC_MODE**
     - R/W (1-bit)
     - TX/RX synchronization coupling. Reset default is ``1`` (TX starts synchronously with RX on the same clock cycle). When set to ``0``, TX operates independently.

Legacy SPI Registers (LMS64C Protocol)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
For host software using the LMS64C register protocol, stream synchronization controls are mapped to the following SPI registers:

.. list-table::
   :widths: 15 22 15 48
   :header-rows: 1

   * - SPI Address
     - Register Name
     - Valid Values
     - Description
   * - **0x000A**
     - **CONTROL**
     - bit 0: ``RX_EN``
       bit 1: ``TX_EN``
     - Standard stream enable register in ``fpgacfg``. Asserting these bits asserts ``rx_en_req`` and ``tx_en_req``, arming the ``StreamStartController``.
   * - **0x00CA**
     - **PERIPH_INPUT_SEL_0**
     - 0, 1
     - Selects the active peripheral input source (e.g. PPS selection between on-board GNSS and external SYNC connector on LimeSDR XTRX).
   * - **0x0281**
     - **RX_DELAY**
     - 0, 1, 2, 3
     - Sets the RX start trigger mode (0 = Immediate, 1 = On PPS, 2 = On PPS Valid, 3 = On Ext Trigger). Mapped to ``limetop_stream_start_controller_rx_delay_mode``.
   * - **0x0282**
     - **TX_DELAY**
     - 0, 1, 2, 3
     - Sets the TX start trigger mode (0 = Immediate, 1 = On PPS, 2 = On PPS Valid, 3 = On Ext Trigger). Mapped to ``limetop_stream_start_controller_tx_delay_mode``.

Operational Workflow
--------------------
To perform synchronized multi-channel or multi-device streaming:

1. **Configure PPS/Trigger Source:** If using runtime-selectable sources (e.g. LimeSDR XTRX), configure register **0x00CA** (``PERIPH_INPUT_SEL_0``) to select the on-board GNSS module or external SYNC header input.
2. **Select Start Mode:** Write the desired trigger mode (e.g. Mode 1 for PPS or Mode 3 for External Trigger) to **0x0281** (RX) and optionally **0x0282** (TX).
3. **Verify TX Synchronization Coupling:** By default, ``tx_sync_mode`` is ``1`` (TX starts in lockstep with RX on the same clock cycle). If independent TX gating is needed, configure the ``LIMETOP_STREAM_START_CONTROLLER_TX_SYNC_MODE`` CSR accordingly.
4. **Arm Streaming:** Assert the ``RX_EN`` and ``TX_EN`` bits in control register **0x000A**. The gateware enters the armed state with datapath streaming held at ``0``.
5. **Hardware Trigger Assertion:** Upon the arrival of the next matching trigger edge (PPS rising edge or external trigger), the gateware asserts the internal stream enable latches and streaming begins in deterministic alignment.
6. **Teardown / Stop:** Writing ``0`` to **0x000A** immediately deasserts stream enables and resets the trigger latches.

