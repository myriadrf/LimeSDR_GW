.. _stream_packet_structure:

Stream Packet Structure
=======================

This page describes the host stream packet format used at the streaming boundary between the host
interface and **LimeTOP**.

Overview
--------

The stream protocol uses a variable up to  **4096-byte packet**. Each packet contains:

-  **16-byte header** used for packet metadata and stream control,
-  up to **4080-byte payload** used for I/Q sample data.

Header contains receiver and transmitter status flags and packet timestamp. Timestamp is a 64 bit samples 
counter used to synchronize received and transmitted signals. The counter is being incremented with each 
sample after Receiver is enabled. Timestamp can be reset to 0, by using FPGA registers, packets streaming
should be disabled when reseting timestamp. Payload contains RF samples data, the data
format and ordering depends on number of active channels and each sample bit count.


Packet Layout
-------------

At the top level, the packet is split into a smaller header followed by a larger sample payload parts.

.. figure:: images/stream_packet.drawio.svg
   :align: center
   :width: 1000
   :alt: LimeSDR_GW top-level architecture

The exact byte ranges and field definitions are listed in the table below:

.. list-table:: Table 1. Stream packet layout
   :header-rows: 1
   :widths: 18 22 60

   * - Bytes
     - Field
     - Meaning
   * - 0-15
     - Header
     - Packet metadata and stream-control information.
   * - 16-4095
     - Payload
     - I/Q sample payload. The payload size is 4080 bytes.

Stream Packet Field Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The 16-byte packet header is made from two 64-bit words. The first word is the general packet
header, and the second word is the sample counter or timestamp used for stream synchronization.


  .. flat-table:: Table 2. Stream packet field definition
     :header-rows: 1

     * - Byte index
       - Bits
       - Description
     * - :rspan:`3` 0
       - 15-2
       - Reserved
     * - 4
       - **Disable timestamp synchronization for this packet.**
         
         0 – synchronize packet transmitting with timestamp.

         1 – ignore timestamp, transmit as soon as possible. 

         **Note1:** Prior synchronized packets existing in FIFO can delay transmitting of unsynchronized packet.
         
         **Note2:** Gets OR’ed with SPI SYNCH_DIS register value

         **Note3:** For transmit packets only
     * - 3
       - **Tx packet dropped:**
         
         0 - Tx is working normally
         
         1 - Tx received packet with obsolete timestamp
     * - 2-0
       - Reserved
     * - 1-2
       - 31-0  
       - Packet payload size: 

         Payload size specified in bytes
     * - 3-7
       -
       - Reserved
     * - 8-15
       - 64-0
       - **Timestamp:**
         
         64 bit samples counter, stored in Big endian format

         When Receiving: timestamp when the first sample in payload was received

         When Transmitting: timestamp when the first sample in payload should be transmitted
     * - 16-4095
       -
       - **Payload data (IQ samples):**

         Samples format and ordering depends on streaming configuration. Check :ref:`Packet Payload Structure  <pkt_payload_structure>`


.. _pkt_payload_structure:

Packet Payload Structure
^^^^^^^^^^^^^^^^^^^^^^^^

When using **12 bit compressed** samples configuration the packet payload has the following
structure. Bytes are indexed from payload start.

.. flat-table:: Table 3. 12 bit samples data structure
   :header-rows: 1

   * - Byte index
     - Bits
     - Description
   * - 0
     - 7-0
     - ch0_I0 [7:0]
   * - :rspan:`1` 1
     - 7-4
     - ch0_Q0 [3:0]
   * - 3-0
     - ch0_I0 [11:8]
   * - 2
     - 7-0
     - ch0_Q0 [11:4]
   * - 3
     - 7-0
     - ch1_I1 [7:0]
   * - :rspan:`1` 4
     - 7-4
     - ch1_Q1 [3:0]
   * - 3-0
     - ch1_I1 [11:8]
   * - 5
     - 7-0
     - ch1_Q1 [11:4]
   * - ...
     - ...
     - ...

When using **16 bit** samples configuration the packet payload has the following
structure. Bytes are indexed from payload start.


.. flat-table:: Table 4. 16 bit samples data structure
   :header-rows: 1

   * - Byte index
     - Bits
     - Description
   * - 0
     - 7-0
     - ch0_I0 [7:0]
   * - 1
     - 7-0
     - ch0_I0 [15:8]
   * - 2
     - 7-0
     - ch0_Q0[7:0]
   * - 3
     - 7-0
     - ch0_Q0[15:0]
   * - 4
     - 7-0
     - ch1_I1[7:0]
   * - 5
     - 7-0
     - ch1_I1[15:8]
   * - 6
     - 7-0
     - ch1_Q1[7:0]
   * - 7
     - 7-0
     - ch1_Q1[15:8]
   * - ...
     - ...
     - ...

