TX Stream Start Buffering
=========================

Purpose
-------
At high sample rates, on rare occasions, the TX path can be starved near the beginning of a stream:
the FPGA starts sending samples to the RF transceiver as soon as the first packet arrives, and if the
host cannot deliver the following data quickly enough, the TX buffer underruns before streaming
has settled.

TX stream start buffering lets the host request that more data is accumulated in the FPGA TX buffer
before the first sample is sent. The extra buffered data acts as a reserve during stream start-up.
This **increases TX latency** by the amount of data buffered, while **decreasing the chance of TX
starvation**. With default settings (both thresholds ``0``) the feature is disabled and behaviour
is unchanged from earlier gateware.

Two thresholds are available:

* ``payload_buff_cnt`` - **amount of sample data** to buffer. This is the recommended control: the
  start delay depends only on the configured amount of data and the sample rate, so it is the same
  on every stream start regardless of the packet size used by the host software.
* ``pct_buff_cnt`` - **number of packets** to buffer. Packet size is chosen by the host software and
  differs between applications and configurations, so the resulting amount of buffered data (and
  latency) depends on the software in use.

Primary Code References
-----------------------

* **TX packet FIFO (gating logic):** ``gateware/LimeDFB/lime_txpct_fifo/src/lime_txpct_fifo.vhd``
* **TX path top (CSRs and clock-domain crossing):** ``gateware/LimeDFB/tx_path_top/src/tx_path_top.py``
* **Legacy SPI register mapping:** ``firmware/bsp/<board>/regremap.c`` (addresses ``0x0200`` / ``0x0201``)

Scope
-----
Buffering applies only to packets that are **not** timestamp-synchronized, i.e. packets whose header
has the timestamp synchronization disable flag (header bit 4, ``sync_dis``) set. Such packets are
normally streamed out automatically as soon as they are available.

Timestamp-synchronized packets (``sync_dis = 0``) are not affected: their start time is already
determined by the packet timestamp, and the host controls latency by choosing the timestamp.

Operating Principle
-------------------
While the TX path is held in reset (stream disabled), the start condition is armed. After the stream
is enabled, the first ``sync_dis`` packet is received completely and then held at the head of the TX
packet FIFO until **one** of the following is true:

1. **Thresholds reached:**

   * the payload RAM holds at least ``payload_buff_cnt`` 128-bit words (if ``payload_buff_cnt`` is
     non-zero), **and**
   * at least ``pct_buff_cnt`` complete packets are queued behind the held packet.

2. **Buffer full:** the TX buffer cannot accept more data, either because all packet slots are in
   use or because the next incoming packet's payload does not fit in the remaining payload RAM.
   Waiting longer cannot buffer more data, so streaming is released instead of stalling.

The payload word count includes the held packet and is updated word by word as data arrives (packet
headers are not counted), so streaming starts as soon as exactly ``payload_buff_cnt`` words are
buffered, independent of how the data is divided into packets. If the first packet alone already
contains more data than ``payload_buff_cnt``, streaming starts as soon as that packet is complete.

Once the first packet is released, buffering is disabled for the rest of the stream: subsequent
packets stream out as soon as they are available, and a later underrun does **not** cause
re-buffering. The condition is re-armed only when the TX path is reset, i.e. when the stream is
stopped and started again.

.. wavedrom::
   :caption: Start buffering with payload_buff_cnt = 12 words

   { "signal": [
     { "name": "clk",                 "wave": "p..........." },
     { "name": "tx stream enable",    "wave": "01.........." },
     { "name": "payload words stored","wave": "=.==.==.==..", "data": ["0", "2", "4", "6", "8", "10", "12"] },
     { "name": "start released",      "wave": "0........1.." },
     { "name": "TX samples out",      "wave": "x........333", "data": ["D0", "D1", "D2"] }
   ],
   "head": { "text": "Output is held until 12 payload words are buffered" }}

.. note::
    If the host stops sending data before the thresholds are reached and the buffer is not full,
    the held data is not transmitted until more data arrives. For short bursts, make sure the burst
    contains at least the configured amount of data, or leave the thresholds at ``0``.

Register Reference
------------------
Both registers default to ``0`` (no buffering). Write them **before** enabling the TX stream; changing
them while streaming is not supported.

Legacy SPI Registers (LMS64C Protocol)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 12 20 10 58
   :header-rows: 1

   * - Address
     - Name
     - Bits
     - Description
   * - ``0x0201``
     - ``payload_buff_cnt``
     - ``[15:0]``
     - Number of 128-bit (16-byte) payload words to buffer before streaming starts
       (``0`` = not used). Includes the data of the held packet; packet headers are not counted.
       Clamped in hardware to the payload RAM size.
   * - ``0x0200``
     - ``pct_buff_cnt``
     - ``[7:0]``
     - Number of complete packets to buffer behind the first packet before streaming starts.
       Clamped in hardware to the number of packet slots minus one.

When both are non-zero, both conditions must be met (or the buffer must be full).

Native LiteX CSR Registers (``LimeTop``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - CSR
     - Equivalent legacy register
   * - ``LIMETOP_RXTX_TOP_TX_PATH_PAYLOAD_BUFF_CNT``
     - ``0x0201``
   * - ``LIMETOP_RXTX_TOP_TX_PATH_PCT_BUFF_CNT``
     - ``0x0200``

Both CSRs are synchronized into the TX path clock domain.

TX Buffer Size per Board
------------------------
The thresholds are limited by the TX buffer of each board:

.. list-table::
   :widths: 30 40 30
   :header-rows: 1

   * - Board
     - Payload RAM
     - Packet slots
   * - LimeSDR XTRX
     - 16 KiB (1024 words)
     - 16
   * - LimeSDR USB
     - 16 KiB (1024 words)
     - 16
   * - sSDR rev2
     - 16 KiB (1024 words)
     - 16
   * - LimeSDR Mini V1
     - 8 KiB (512 words)
     - 16
   * - LimeSDR Mini V2
     - 8 KiB (512 words)
     - 16
   * - HiperSDR 44xx
     - 8 KiB (512 words)
     - 16

Because packets are stored whole, the payload RAM cannot always be filled completely: a packet
is only accepted when all of its payload fits in the remaining space. If ``payload_buff_cnt`` is
set close to the payload RAM size, the threshold may not be reachable with the packet size in use;
streaming then starts on the *buffer full* condition and the amount buffered depends on the packet
size. To keep the start delay deterministic, leave at least one packet's worth of free space, i.e.
keep ``payload_buff_cnt`` at or below the payload RAM size minus the largest packet payload the
host software will send.

Choosing a Value and Latency
----------------------------
The additional start latency introduced by ``payload_buff_cnt`` is the playback time of the buffered
data:

.. math::

   t_{delay} = \frac{\text{payload_buff_cnt} \times 16}{\text{bytes per sample} \times \text{channels} \times f_s}

where *bytes per sample* is the size of one I/Q sample of one channel (4 bytes for 16-bit samples,
3 bytes for 12-bit samples), and *channels* is the number of enabled TX channels.

For example, ``payload_buff_cnt = 256`` (4 KiB) with 16-bit samples on two channels holds 512 samples
per channel; at :math:`f_s = 61.44` MS/s this adds about 8.3 µs of latency.

Recommended approach:

* Leave both registers at ``0`` unless TX underruns are observed at stream start.
* Use ``payload_buff_cnt`` and keep ``pct_buff_cnt`` at ``0``. Start with a small value and increase
  it until start-up underruns no longer occur, staying within the limit described in
  `TX Buffer Size per Board`_.
* Use ``pct_buff_cnt`` only when the packet size of the host software is known and fixed.

Operational Workflow
--------------------

1. Configure the TX stream as usual, with timestamp synchronization disabled for the TX packets.
2. Write ``payload_buff_cnt`` (``0x0201``) and, if needed, ``pct_buff_cnt`` (``0x0200``).
3. Enable the TX stream and start sending data. Output starts once the thresholds are met or the
   buffer is full.
4. To apply new values, stop the stream, write the registers, and start the stream again.
