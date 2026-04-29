HiperSDR 44xx Host Register Reference
=====================================

This page is generated from CSV sources.

Quick Navigation
----------------

.. list-table:: Module overview
   :header-rows: 1
   :widths: 20 16 64

   * - Module
     - Address range
     - Typical use
   * - :ref:`FPGACFG <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000`` - ``0x001F``
     - Board ID/revision, stream mode, RF/TDD controls, LMS enable/reset
   * - :ref:`PLLCFG <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0020`` - ``0x003F``
     - PLL status, divider/multiplier values, phase configuration
   * - :ref:`TSTCFG <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0060`` - ``0x007F``
     - Clock/test control and status counters
   * - :ref:`PERIPHCFG <hipersdr_44xx_regmap_periphcfg>`
     - ``0x00C0`` - ``0x00D3``
     - GPIO and peripheral routing/enable controls
   * - :ref:`MEMCFG <hipersdr_44xx_regmap_memcfg>`
     - ``0xFFE0`` - ``0xFFFF``
     - Reserved memory-mapped window

.. _hipersdr_44xx_regmap_fpgacfg:

FPGACFG Registers (``0x0000`` - ``0x001F``)
-------------------------------------------

.. list-table:: FPGACFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0000 <hipersdr_44xx_reg_0000>`
     - ``0x001B``
     - ``board_id``
     - Board ID
   * - :ref:`0x0001 <hipersdr_44xx_reg_0001>`
     -  
     - ``major_rev``
     - Major revision
   * - :ref:`0x0002 <hipersdr_44xx_reg_0002>`
     -  
     - ``compile_rev``
     - Compile revision
   * - :ref:`0x0003 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0004 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0005 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0006 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0007 <hipersdr_44xx_reg_0007>`
     - ``0x0003``
     - ``ch_en``
     - Channel enable
   * - :ref:`0x0008 <hipersdr_44xx_reg_0008>`
     - ``0x0102``
     - ``stream_ctrl``
     - Sample width and interface mode control.
   * - :ref:`0x0009 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x000A <hipersdr_44xx_reg_000a>`
     - ``0x0000``
     - ``rf_tdd_ctrl``
     - RF switch, TDD, pattern generation, and unified RX/TX enable control.
   * - :ref:`0x000B <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000C <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x000D <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000E <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x000F <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x03FC``
     - \-
     - Reserved.
   * - :ref:`0x0010 <hipersdr_44xx_reg_0010>`
     - ``0x0001``
     - ``txant_pre``
     - Number of samples to delay turning on the internal TDD signal.
   * - :ref:`0x0011 <hipersdr_44xx_reg_0011>`
     - ``0x0001``
     - ``txant_post``
     - Number of samples to delay turning off the internal TDD signal.
   * - :ref:`0x0012 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0xFFFF``
     - \-
     - Reserved.
   * - :ref:`0x0013 <hipersdr_44xx_reg_0013>`
     - ``0x6F6B``
     - ``lms_misc_ctrl``
     - LMS7002 digital-interface and hard-enable control.
   * - :ref:`0x0014 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x0015 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0016 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0017 <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x2340``
     - \-
     - Reserved.
   * - :ref:`0x0018 <hipersdr_44xx_reg_0018>`
     - ``0x0003``
     - ``lms_clk_ctrl``
     - LMS power, clock-source, and reset control.
   * - :ref:`0x0019 <hipersdr_44xx_reg_0019>`
     - ``0x1000``
     - ``rx_packet_size``
     - RX packet size in bytes.
   * - :ref:`0x001A <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001B <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001C <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001D <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x00FF``
     - \-
     - Reserved.
   * - :ref:`0x001E <hipersdr_44xx_regmap_fpgacfg>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x001F <hipersdr_44xx_regmap_fpgacfg>`
     - ``0xD090``
     - \-
     - Reserved.

.. _hipersdr_44xx_regmap_pllcfg:

PLLCFG Registers (``0x0020`` - ``0x003F``)
------------------------------------------

.. list-table:: PLLCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0020 <hipersdr_44xx_reg_0020>`
     - ``0x0000``
     - ``c1 phase``
     - Phase value for PLL output clock 1.
   * - :ref:`0x0021 <hipersdr_44xx_reg_0021>`
     - ``0x0001``
     - ``pll_status``
     - PLL and phase-configuration status.
   * - :ref:`0x0022 <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0023 <hipersdr_44xx_reg_0023>`
     - ``0x0000``
     - ``pll_ctrl``
     - PLL reconfiguration control.
   * - :ref:`0x0024 <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0025 <hipersdr_44xx_regmap_pllcfg>`
     - ``0x01F0``
     - \-
     - Reserved.
   * - :ref:`0x0026 <hipersdr_44xx_reg_0026>`
     - ``0x000A``
     - ``mn_bypass_ctrl``
     - PLL multiplier/divider bypass.
   * - :ref:`0x0027 <hipersdr_44xx_reg_0027>`
     - ``0x0AAA``
     - ``c01_bypass_ctrl``
     - PLL output-divider bypass.
   * - :ref:`0x0028 <hipersdr_44xx_regmap_pllcfg>`
     - ``0xAAAA``
     - \-
     - Reserved.
   * - :ref:`0x0029 <hipersdr_44xx_regmap_pllcfg>`
     - ``0xAAAA``
     - \-
     - Reserved.
   * - :ref:`0x002A <hipersdr_44xx_reg_002a>`
     - ``0x0000``
     - ``n_cnt``
     - PLL divider value.
   * - :ref:`0x002B <hipersdr_44xx_reg_002b>`
     - ``0x0000``
     - ``m_cnt``
     - PLL multiplier value.
   * - :ref:`0x002C <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x002D <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x002E <hipersdr_44xx_reg_002e>`
     - ``0x0000``
     - ``c0_cnt``
     - PLL output 0 divider value.
   * - :ref:`0x002F <hipersdr_44xx_reg_002f>`
     - ``0x0000``
     - ``c1_cnt``
     - PLL output 1 divider value.
   * - :ref:`0x0030 <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0031 <hipersdr_44xx_regmap_pllcfg>` - :ref:`0x003F <hipersdr_44xx_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _hipersdr_44xx_regmap_tstcfg:

TSTCFG Registers (``0x0060`` - ``0x007F``)
------------------------------------------

.. list-table:: TSTCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0060 <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0061 <hipersdr_44xx_reg_0061>`
     - ``0x0000``
     - ``test_en``
     - Test start bits for sys_clk, LMS_TX_CLK, and GNSS.
   * - :ref:`0x0062 <hipersdr_44xx_regmap_tstcfg>` - :ref:`0x0064 <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0065 <hipersdr_44xx_reg_0065>`
     - ``0x0000``
     - ``test_cmplt``
     - Test-complete bits for sys_clk, LMS_TX_CLK, and GNSS.
   * - :ref:`0x0066 <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0067 <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0068 <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0069 <hipersdr_44xx_reg_0069>`
     - ``0x0000``
     - ``sys_clk_cnt``
     - Count of sys_clk cycles
   * - :ref:`0x006A <hipersdr_44xx_regmap_tstcfg>` - :ref:`0x006D <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x006E <hipersdr_44xx_regmap_tstcfg>` - :ref:`0x0071 <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0072 <hipersdr_44xx_reg_0072>`
     - ``0x0000``
     - ``lms_tx_clk_cnt``
     - Low 16 bits of LMS TX clock test counter.
   * - :ref:`0x0073 <hipersdr_44xx_reg_0073>`
     - ``0x0000``
     - ``lms_tx_clk_cnt``
     - Bits 23:16 of LMS TX clock test counter.
   * - :ref:`0x0074 <hipersdr_44xx_regmap_tstcfg>` - :ref:`0x007C <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x007D <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x007E <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x007F <hipersdr_44xx_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _hipersdr_44xx_regmap_periphcfg:

PERIPHCFG Registers (``0x00C0`` - ``0x00D3``)
---------------------------------------------

.. list-table:: PERIPHCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x00C0 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x00C1 <hipersdr_44xx_regmap_periphcfg>` - :ref:`0x00C3 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00C4 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x00C5 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00C6 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x00C7 <hipersdr_44xx_regmap_periphcfg>` - :ref:`0x00C9 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00CA <hipersdr_44xx_reg_00ca>`
     - ``0x0000``
     - ``periph_input_sel``
     - PPS input source select. 0 = GNSS_1PPS, 1 = 1PPSI_GPIO1.
   * - :ref:`0x00CB <hipersdr_44xx_regmap_periphcfg>` - :ref:`0x00D1 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00D2 <hipersdr_44xx_reg_00d2>`
     - ``0x0003``
     - ``periph_en``
     - Peripheral enables: clock-out routing, GNSS reset, GNSS standby.
   * - :ref:`0x00D3 <hipersdr_44xx_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _hipersdr_44xx_regmap_memcfg:

MEMCFG Registers (``0xFFE0`` - ``0xFFFF``)
------------------------------------------

.. list-table:: MEMCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0xFFE0 <hipersdr_44xx_regmap_memcfg>` - :ref:`0xFFFF <hipersdr_44xx_regmap_memcfg>`
     - ``0x0000``
     - \-
     - All MEMCFG addresses in this documented range are reserved.

Register Bitfield Reference
-------------------

Bit fields below are shown from MSB to LSB where applicable.

.. _hipersdr_44xx_reg_0000:

``0x0000`` - board_id
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0000`` | Default: ``0x001B`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_id [7:0]"},
     {"bits": 8, "name": "board_id [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``board_id``
     -  
     - Board ID, read only.

.. _hipersdr_44xx_reg_0001:

``0x0001`` - major_rev
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0001`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "major_rev [7:0]"},
     {"bits": 8, "name": "major_rev [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``major_rev``
     -  
     - Major revision, read only.

.. _hipersdr_44xx_reg_0002:

``0x0002`` - compile_rev
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0002`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "compile_rev [7:0]"},
     {"bits": 8, "name": "compile_rev [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``compile_rev``
     -  
     - Compile revision, read only.

.. _hipersdr_44xx_reg_0007:

``0x0007`` - ch_en
^^^^^^^^^^^^^^^^^^

Address: ``0x0007`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "ch_en [1:0]"},
     {"bits": 6, "name": "Reserved [7:2]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[1:0]``
     - ``ch_en``
     - 01 = A, 10 = B, 11 = A+B
     - Channel selection.

.. _hipersdr_44xx_reg_0008:

``0x0008`` - stream_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0008`` | Default: ``0x0102`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "smpl_width [1:0]"},
     {"bits": 4, "name": "Reserved [5:2]"},
     {"bits": 1, "name": "ddr_en"},
     {"bits": 1, "name": "trxiq_pulse"},
     {"bits": 1, "name": "mimo_int_en"},
     {"bits": 1, "name": "synch_dis"},
     {"bits": 6, "name": "Reserved [15:10]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[9]``
     - ``synch_dis``
     - 0 = enabled, 1 = disabled
     - Packet synchronization using timestamps.
   * - ``[8]``
     - ``mimo_int_en``
     - 0 = disabled, 1 = enabled
     - MIMO mode.
   * - ``[7]``
     - ``trxiq_pulse``
     - 0 = off, 1 = on
     - TRXIQ pulse mode.
   * - ``[6]``
     - ``ddr_en``
     - 0 = SDR, 1 = DDR
     - DIQ interface mode.
   * - ``[1:0]``
     - ``smpl_width``
     - 10 = 12-bit, 00 = 16-bit
     - Sample width selection.

.. _hipersdr_44xx_reg_000a:

``0x000A`` - rf_tdd_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "rx_en"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 2, "name": "rx_rf_sw [3:2]"},
     {"bits": 1, "name": "tx_rf_sw"},
     {"bits": 1, "name": "tdd_manual"},
     {"bits": 1, "name": "tdd_auto_en"},
     {"bits": 1, "name": "tdd_invert"},
     {"bits": 1, "name": "rx_ptrn_en"},
     {"bits": 1, "name": "tx_ptrn_en"},
     {"bits": 1, "name": "tx_cnt_en"},
     {"bits": 1, "name": "rf_sw_auto_en"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11]``
     - ``rf_sw_auto_en``
     - 0/1
     - Control RF switches by internal TDD signal.
   * - ``[10]``
     - ``tx_cnt_en``
     - 0/1
     - Counter test pattern on TX.
   * - ``[9]``
     - ``tx_ptrn_en``
     - 0/1
     - Test pattern on TX.
   * - ``[8]``
     - ``rx_ptrn_en``
     - 0/1
     - Test pattern on RX.
   * - ``[7]``
     - ``tdd_invert``
     - 0/1
     - Invert external TDD signal.
   * - ``[6]``
     - ``tdd_auto_en``
     - 0/1
     - Control external TDD signal by internal TDD signal.
   * - ``[5]``
     - ``tdd_manual``
     - 0/1
     - Manual value of external TDD signal.
   * - ``[4]``
     - ``tx_rf_sw``
     - 0 = TX2, 1 = TX1
     - TX RF switch select.
   * - ``[3:2]``
     - ``rx_rf_sw``
     - 00 = RX_W, 01 = RX_L, 10 = RX_H, 11 = NC
     - RX RF switch select.
   * - ``[0]``
     - ``rx_en``
     - 0/1
     - Unified RX/TX enable.

.. _hipersdr_44xx_reg_0010:

``0x0010`` - txant_pre
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0010`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "txant_pre [7:0]"},
     {"bits": 8, "name": "txant_pre [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``txant_pre``
     -  
     - Number of samples to delay turning on the internal TDD signal.

.. _hipersdr_44xx_reg_0011:

``0x0011`` - txant_post
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0011`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "txant_post [7:0]"},
     {"bits": 8, "name": "txant_post [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``txant_post``
     -  
     - Number of samples to delay turning off the internal TDD signal.

.. _hipersdr_44xx_reg_0013:

``0x0013`` - lms_misc_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0013`` | Default: ``0x6F6B`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "LMS1_RESET"},
     {"bits": 1, "name": "LMS1_CORE_LDO_EN"},
     {"bits": 1, "name": "LMS1_TXNRX1"},
     {"bits": 1, "name": "LMS1_TXNRX2"},
     {"bits": 1, "name": "LMS1_TXEN"},
     {"bits": 1, "name": "LMS1_RXEN"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 8, "name": "LMS_TXRXEN_MUX_SEL [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``LMS_TXRXEN_MUX_SEL``
     -  
     - LMS TX/RX hard-enable source mux selection.
   * - ``[6]``
     - ``LMS1_RXEN``
     - 0=Disabled,1=Enabled
     - LMS1 RX hard enable.
   * - ``[5]``
     - ``LMS1_TXEN``
     - 0=Disabled,1=Enabled
     - LMS1 TX hard enable.
   * - ``[4]``
     - ``LMS1_TXNRX2``
     - 0=TXIQ,1=RXIQ
     - LMS1 port 2 mode selection.
   * - ``[3]``
     - ``LMS1_TXNRX1``
     - 0=TXIQ,1=RXIQ
     - LMS1 port 1 mode selection.
   * - ``[2]``
     - ``LMS1_CORE_LDO_EN``
     - 0=Disabled,1=Enabled
     - LMS1 internal LDO enable.
   * - ``[1]``
     - ``LMS1_RESET``
     - 0=Reset active,1=Inactive
     - LMS1 hardware reset.

.. _hipersdr_44xx_reg_0018:

``0x0018`` - lms_clk_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0018`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "LMS_RST"},
     {"bits": 1, "name": "TCXO_EN"},
     {"bits": 1, "name": "EXT_CLK"},
     {"bits": 1, "name": "CORE_LDO_EN"},
     {"bits": 4, "name": "Reserved [7:4]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[3]``
     - ``CORE_LDO_EN``
     - 0/1
     - LMS internal LDO control.
   * - ``[2]``
     - ``EXT_CLK``
     - 0 = onboard, 1 = external
     - Clock source selection.
   * - ``[1]``
     - ``TCXO_EN``
     - 0/1
     - Onboard clock enable.
   * - ``[0]``
     - ``LMS_RST``
     - 0 = reset active, 1 = reset inactive
     - LMS hardware reset.

.. _hipersdr_44xx_reg_0019:

``0x0019`` - rx_packet_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0019`` | Default: ``0x1000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "RX_PACKET_SIZE [7:0]"},
     {"bits": 8, "name": "RX_PACKET_SIZE [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``RX_PACKET_SIZE``
     -  
     - RX packet size in bytes.

.. _hipersdr_44xx_reg_0020:

``0x0020`` - c1 phase
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0020`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``value``
     -  
     - Phase value for PLL output clock 1.

.. _hipersdr_44xx_reg_0021:

``0x0021`` - pll_status
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0021`` | Default: ``0x0001`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "pllcfg_done"},
     {"bits": 1, "name": "pllcfg_busy"},
     {"bits": 1, "name": "phcfg_done"},
     {"bits": 1, "name": "phcfg_error"},
     {"bits": 4, "name": "Reserved [7:4]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[3]``
     - ``phcfg_error``
     - status
     - Phase configuration error, documented as unused.
   * - ``[2]``
     - ``phcfg_done``
     - status
     - Phase configuration done, read only.
   * - ``[1]``
     - ``pllcfg_busy``
     - status
     - PLL configuration busy, read only.
   * - ``[0]``
     - ``pllcfg_done``
     - status
     - PLL configuration done, read only.

.. _hipersdr_44xx_reg_0023:

``0x0023`` - pll_ctrl
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0023`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "phcfg_start"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 5, "name": "pll_ind [7:3]"},
     {"bits": 6, "name": "Reserved [13:8]"},
     {"bits": 1, "name": "phcfg_mode"},
     {"bits": 1, "name": "Reserved"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[14]``
     - ``phcfg_mode``
     - 0 = manual, 1 = auto
     - Phase configuration mode.
   * - ``[7:3]``
     - ``pll_ind``
     - 0000 = TX PLL, 0001 = RX PLL
     - PLL index.
   * - ``[1]``
     - ``phcfg_start``
     - 0->1 edge
     - Start phase configuration on transition.

.. _hipersdr_44xx_reg_0026:

``0x0026`` - mn_bypass_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0026`` | Default: ``0x000A`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "n_byp"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "m_byp"},
     {"bits": 4, "name": "Reserved [7:4]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[3]``
     - ``m_byp``
     - 0=normal,1=bypass
     - M counter bypass control.
   * - ``[1]``
     - ``n_byp``
     - 0=normal,1=bypass
     - N counter bypass control.

.. _hipersdr_44xx_reg_0027:

``0x0027`` - c01_bypass_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0027`` | Default: ``0x0AAA`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "c0_byp"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "c1_byp"},
     {"bits": 4, "name": "Reserved [7:4]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[3]``
     - ``c1_byp``
     - 0=normal,1=bypass
     - C1 output-divider bypass control.
   * - ``[1]``
     - ``c0_byp``
     - 0=normal,1=bypass
     - C0 output-divider bypass control.

.. _hipersdr_44xx_reg_002a:

``0x002A`` - n_cnt
^^^^^^^^^^^^^^^^^^

Address: ``0x002A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "n_cnt [7:0]"},
     {"bits": 8, "name": "n_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``n_cnt``
     -  
     - PLL divider value.

.. _hipersdr_44xx_reg_002b:

``0x002B`` - m_cnt
^^^^^^^^^^^^^^^^^^

Address: ``0x002B`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "m_cnt [7:0]"},
     {"bits": 8, "name": "m_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``m_cnt``
     -  
     - PLL multiplier value.

.. _hipersdr_44xx_reg_002e:

``0x002E`` - c0_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x002E`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c0_cnt [7:0]"},
     {"bits": 8, "name": "c0_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c0_cnt``
     -  
     - PLL output 0 divider value.

.. _hipersdr_44xx_reg_002f:

``0x002F`` - c1_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x002F`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c1_cnt [7:0]"},
     {"bits": 8, "name": "c1_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c1_cnt``
     -  
     - PLL output 1 divider value.

.. _hipersdr_44xx_reg_0061:

``0x0061`` - test_en
^^^^^^^^^^^^^^^^^^^^

Address: ``0x0061`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "test_en[0]"},
     {"bits": 1, "name": "test_en[1]"},
     {"bits": 1, "name": "test_en[2]"},
     {"bits": 5, "name": "Reserved [7:3]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[2]``
     - ``test_en[2]``
     - 0/1
     - GNSS test start.
   * - ``[1]``
     - ``test_en[1]``
     - 0/1
     - LMS TX clock test start.
   * - ``[0]``
     - ``test_en[0]``
     - 0/1
     - System clock test start.

.. _hipersdr_44xx_reg_0065:

``0x0065`` - test_cmplt
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0065`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "test_cmplt[0]"},
     {"bits": 1, "name": "test_cmplt[1]"},
     {"bits": 1, "name": "test_cmplt[2]"},
     {"bits": 5, "name": "Reserved [7:3]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[2]``
     - ``test_cmplt[2]``
     - status
     - GNSS test complete.
   * - ``[1]``
     - ``test_cmplt[1]``
     - status
     - LMS TX clock test complete.
   * - ``[0]``
     - ``test_cmplt[0]``
     - status
     - System clock test complete.

.. _hipersdr_44xx_reg_0069:

``0x0069`` - sys_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0069`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "sys_clk_cnt [7:0]"},
     {"bits": 8, "name": "sys_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``sys_clk_cnt``
     -  
     - Count of sys_clk cycles

.. _hipersdr_44xx_reg_0072:

``0x0072`` - lms_tx_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0072`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lms_tx_clk_cnt [7:0]"},
     {"bits": 8, "name": "lms_tx_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``lms_tx_clk_cnt``
     -  
     - Low 16 bits of LMS TX clock test counter.

.. _hipersdr_44xx_reg_0073:

``0x0073`` - lms_tx_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0073`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lms_tx_clk_cnt [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - \-
     -  
     -  
   * - ``[7:0]``
     - ``lms_tx_clk_cnt``
     -  
     - Bits 23:16 of LMS TX clock test counter.

.. _hipersdr_44xx_reg_00ca:

``0x00CA`` - periph_input_sel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CA`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "periph_input_sel [7:0]"},
     {"bits": 8, "name": "periph_input_sel [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``periph_input_sel``
     - 0 = GNSS_1PPS, 1 = 1PPSI_GPIO1.
     - PPS input source select.

.. _hipersdr_44xx_reg_00d2:

``0x00D2`` - periph_en
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D2`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 3, "name": "PERIPH_EN [2:0]"},
     {"bits": 5, "name": "Reserved [7:3]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[2:0]``
     - ``PERIPH_EN``
     -  
     - Peripheral enables: clock-out routing, GNSS reset, GNSS standby.
