UTC Timestamping on LimeSDR‑XTRX
================================

Primary Code References
-----------------------
The logic described below is distributed across the following source files:

* **Board Target:** *boards/targets/limesdr_xtrx.py*
* **NMEA Parser:** *gateware/LimeDFB/general/ZDAParser.py*
* **RX Datapath:** *gateware/LimeDFB/rx_path_top/src/rx_path_top.py*

System Overview
---------------
The timestamping architecture integrates external GPS data with internal FPGA counters to provide accurate packet timing.

External PPS
    Selected in *limesdr_xtrx.py* from either the GPS header or the SYNC header. Distributed internally as the signal *pps_internal*.

GNSS UART
    Feeds the *ZDAParser* module, which extracts UTC date/time. The parser asserts *time_valid* only when a correct ZDA sentence is received (Checksum OK).
    *Note: The parser ignores the Talker ID and matches strictly on the string "ZDA,".*

RX Packet Timestamps
    The RX path injects timestamps into the packet headers using one of two modes:

    * **Classic Mode:** Standard datapath sample counter.
    * **Mixed Mode:** 64‑bit value combining a PPS seconds counter (upper 32 bits) and a cycle counter (lower 32 bits).
        * **[63:32]:** Seconds elapsed since the **RX Stream Start** event.
        * **[31:0]:** RX cycles since the last PPS.

Clock Domain Synchronization
    The PPS signal is re‑synchronized explicitly at each consumer to ensure stability:

    * **ZDAParser:** Re-synced to the default *sys* domain.
    * **RXPathTop:** Re-synced to the *lms_rx* domain.

PPS and UART Plumbing
---------------------
This section details how signals are routed in *limesdr_xtrx.py*.

Hardware Interconnects
~~~~~~~~~~~~~~~~~~~~~~
* **GNSS UART:** A *UARTPHY* instance (baudrate=9600) connects its RX stream directly to *ZDAParser.sink*.
* **Fan-out:** The selected *pps_internal* signal is routed to:
    * *ZDAParser.pps* (Used for validity gating).
    * *RXPathTop.pps* (Used to reset/increment timestamp counters).

PPS Source Selection
~~~~~~~~~~~~~~~~~~~~
The PPS source is controlled via the CSR *periphcfg.PERIPH_INPUT_SEL_0[0]*.

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Value
     - Selected Source
   * - **0b01**
     - **SYNC Header** (*synchro_pads.pps_in*)
   * - **Other**
     - **GPS Header** (*gps_pads.pps*)

Time Readback (Software Mirror)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The main register bank provides "Current Time" CSRs that mirror the parser outputs for software access:

* *main_time_min_sec*: Fields *sec[5:0]*, *min[11:6]*
* *main_time_mon_day_hrs*: Fields *hrs[4:0]*, *day[9:5]*, *mon[13:10]*
* *main_time_yrs*: Field *yrs[11:0]*

UTC Extraction and Validity
---------------------------
The *ZDAParser* module handles NMEA sentence processing.

**Input Format**
    Accepts NMEA ZDA sentences: *$--ZDA,hhmmss.ss,dd,mm,yyyy,xx,xx*CS*

**Extracted Signals**
    *time_hours*, *time_minutes*, *time_seconds*, *time_day*, *time_month*, *time_year*.

**Validity Logic**
    The *time_valid* signal indicates the reliability of the current timestamp.

    * **On PPS Rising Edge:** *time_valid* is **CLEARED**. (The previous UTC time applies to the old second).
    * **On ZDA Receipt:**
        * If Checksum is OK: *time_valid* is **SET**.
        * If Checksum errors: *time_valid* is **CLEARED**.

RX Timestamps and Header Insertion
----------------------------------
*(Logic located in rx_path_top.py)*

Timestamp Configuration
~~~~~~~~~~~~~~~~~~~~~~~
The timestamp mode is controlled by the CSR *lime_top_rxtx_top_rx_path_timestamp_settings.TS_SEL*:

0: Classic Mode
    Header word *PCT_HDR_1* carries the standard datapath sample counter.

1: Mixed Mode
    Header word *PCT_HDR_1* is a 64‑bit concatenation containing both PPS and Clock counts.

    .. list-table::
       :widths: 15 30 55
       :header-rows: 1

       * - Bits
         - Name
         - Description
       * - **[63:32]**
         - *timestamp_pps_counter*
         - Seconds since arbitrary start (increments on PPS rising edge).
       * - **[31:0]**
         - *timestamp_clk_count*
         - RX cycles since the last PPS.

.. note::
   **Synchronization:** PPS re‑sync and counters run in the *lms_rx* domain. The PPS rising edge is detected via a small flip-flop chain. This re-synchronization may introduce quantization effects (see Section Edge Cases and Notes).

Counter Logic
~~~~~~~~~~~~~
The counters operate in the *lms_rx* domain under the following rules:

* **Reset State:** If *TS_SEL == 0* or the internal reset is asserted (*int_clk_rst_n == 0*), both counters are held at 0.
* **Active State:** If *TS_SEL == 1*, the counters update as follows:

    * **On PPS rising edge:** *timestamp_clk_count* resets to 0; *timestamp_pps_counter* increments by 1.
    * **Otherwise:** *timestamp_clk_count* increments every RX clock.

Header Constants
~~~~~~~~~~~~~~~~
* **PCT_HDR_0**: **0x7766554433221100** (Fixed identifier for XTRX designs).
* **PCT_HDR_1**: Dynamic timestamp value (Classic or Mixed format as defined above).

Software CSR Reference
----------------------
Key registers for configuring and reading timestamp data.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Function
     - Registers
   * - **PPS Control**
     - | *periphcfg_PERIPH_INPUT_SEL_0* (Bit 0 selects source)
       | *periphcfg_BOARD_GPIO_OVRD/DIR/VAL* (Controls *synchro.pps_out*)
   * - **RX Mode**
     - | *lime_top_rxtx_top_rx_path_timestamp_settings.TS_SEL*
   * - **Time Snapshots**
     - | **Current Time:** *main_time_min_sec*, *main_time_mon_day_hrs*, *main_time_yrs*
       | **RX Start:** *lime_top_rx_time_min_sec*, *lime_top_rx_time_mon_day_hrs*, *lime_top_rx_time_yrs*
       | **TX Start:** *lime_top_tx_time_min_sec*, *lime_top_tx_time_mon_day_hrs*, *lime_top_tx_time_yrs*

*(Note: RX/TX start times are captured on the rising edge of fpgacfg.rx_en / tx_en)*

Quick Setup Checklist
---------------------
1.  **Verify Build:** Ensure the design is built with *soc_has_timesource=True* (Default for XTRX).
2.  **Select Source:** Write *periphcfg.PERIPH_INPUT_SEL_0[0]*:
    Set to **1** for SYNC Header PPS.
    Set to **0** for GPS Header PPS.
3.  **Enable Mixed Mode:** Write *TS_SEL=1* in *lime_top_rxtx_top_rx_path_timestamp_settings*.
4.  **Parse & Compute:**
     **Capture Base Time (** :math:`T_0` **):** Read the *lime_top_rx_time_...* registers **once** after stream starts.

     **Read Header:** Extract the relative seconds count (*S*) and cycle count (*C*) from the packet header.

     **Calculate Absolute Time:**

      :math:`T_{packet} = T_0 + S + (C / f_{lms\_rx})`

Edge Cases and Notes
--------------------

.. warning::
    **Timestamp Jitter (±1 Cycle)**

    Users may observe a fluctuation of ±1 clock cycle in the reported timestamps. This is an artifact of the input synchronization logic.

    * **Mechanism:** The asynchronous PPS signal is latched into the *lms_rx* domain via a multi-stage Flip-Flop synchronizer.
    * **Effect:** If PPS arrival aligns closely with the clock edge, the synchronizer may resolve the edge one cycle earlier or later between consecutive seconds.

.. wavedrom::

    {
      "signal": [
        { "name": "LMS_RX Clock", "wave": "n.", "period": 4},
        {},
        { "name": "PPS (Scenario A: Early)", "wave": "01......", "node": ".a......" },
        { "name": "Trigger (Scenario A)",   "wave": "0.1.....", "node": "..b......" },
        {},
        { "name": "PPS (Scenario B: Late)",  "wave": "0.1.....", "node": "..c....." },
        { "name": "Trigger (Scenario B)",    "wave": "0.....1.", "node": "......d..." }
      ],
      "edge": [
        "a~>b Latency ~0",
        "c~>d Latency ~1 Cycle"
      ],
      "head": {
        "text": "Quantization Jitter: The 'Missing Cycle' Mechanism",
        "tick": 0
      },
      "config": { "hscale": 2 }
    }

Operational Behaviors
~~~~~~~~~~~~~~~~~~~~~
Counter Free-Run
    If PPS is absent while *TS_SEL=1*, the cycle counter free‑runs and wraps modulo 2^32. The seconds counter **does not** increment.

Mode Switching
    Switching *TS_SEL* asserts counter resets.

    Transition 1→0: Counters held at 0.

    Transition 0→1: Counters resume; seconds counter increments on the *next* PPS.

Clock Domain Logic
    Mixed timestamp counters are clocked in *lms_rx*. Ensure the design configuration matches (they do by default on XTRX).

Appendix: Header Decode Example
-------------------------------
**Scenario:**

Header Value: *PCT_HDR_1 = 0x00003039_000186A0*

Frequency: *f_lms_rx = 30.72 MHz*

**Decoding:**

.. code-block:: text

    1. Extract Fields:
       Seconds = 0x00003039 -> 12345
       Cycles  = 0x000186A0 -> 100000

    2. Calculate Offset:
       Offset  = 100,000 / 30,720,000
               ≈ 3.255 ms (after PPS)

    3. Final Timestamp:
       UTC Time (from CSRs) + 12345s + 3.255 ms