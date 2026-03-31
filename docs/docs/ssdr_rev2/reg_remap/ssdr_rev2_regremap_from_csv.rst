sSDR rev2 Host Register Reference
=================================

This page is generated from CSV sources.

Quick Navigation
----------------

.. list-table:: Module overview
   :header-rows: 1
   :widths: 20 16 64

   * - Module
     - Address range
     - Typical use
   * - :ref:`FPGACFG <ssdr_rev2_regmap_fpgacfg>`
     - ``0x0000`` - ``0x001F``
     - Board ID/revision, stream mode, RF/TDD controls, LMS enable/reset
   * - :ref:`PLLCFG <ssdr_rev2_regmap_pllcfg>`
     - ``0x0020`` - ``0x003F``
     - PLL status, divider/multiplier values, phase configuration
   * - :ref:`TSTCFG <ssdr_rev2_regmap_tstcfg>`
     - ``0x0060`` - ``0x007F``
     - Clock/test control and status counters
   * - :ref:`PERIPHCFG <ssdr_rev2_regmap_periphcfg>`
     - ``0x00C0`` - ``0x00D3``
     - GPIO and peripheral routing/enable controls
   * - :ref:`MEMCFG <ssdr_rev2_regmap_memcfg>`
     - ``0xFFE0`` - ``0xFFFF``
     - Reserved memory-mapped window

.. _ssdr_rev2_regmap_fpgacfg:

FPGACFG Registers (``0x0000`` - ``0x001F``)
-------------------------------------------

.. list-table:: FPGACFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0000 <ssdr_rev2_reg_0000>`
     - ``0x001B``
     - ``board_id``
     - Board ID, read only.
   * - :ref:`0x0001 <ssdr_rev2_reg_0001>`
     -  
     - ``major_rev``
     - Major revision, read only.
   * - :ref:`0x0002 <ssdr_rev2_reg_0002>`
     -  
     - ``compile_rev``
     - Compile revision, read only.
   * - :ref:`0x0003 <ssdr_rev2_reg_0003>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0004 <ssdr_rev2_reg_0004>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0005 <ssdr_rev2_reg_0005>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0006 <ssdr_rev2_reg_0006>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0007 <ssdr_rev2_reg_0007>`
     - ``0x0003``
     - ``ch_en``
     - Channel enable: 01 = A, 10 = B, 11 = A and B.
   * - :ref:`0x0008 <ssdr_rev2_reg_0008>`
     - ``0x0102``
     - ``synch_dis, mimo_int_en, trxiq_pulse, ddr_en, smpl_width``
     - Sample width and interface mode control.
   * - :ref:`0x0009 <ssdr_rev2_reg_0009>`
     - ``0x0003``
     - ``txpct_loss_clr, smpl_nr_clr``
     - TX packet-loss flag clear and timestamp reset control.
   * - :ref:`0x000A <ssdr_rev2_reg_000a>`
     - ``0x0000``
     - ``rf_sw_auto_en, tx_cnt_en, tx_ptrn_en, rx_ptrn_en, tdd_invert, tdd_auto_en, tdd_manual, tx_rf_sw, rx_rf_sw, rx_en``
     - RF switch, TDD, pattern generation, and unified RX/TX enable control.
   * - :ref:`0x000B <ssdr_rev2_reg_000b>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000C <ssdr_rev2_reg_000c>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x000D <ssdr_rev2_reg_000d>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000E <ssdr_rev2_reg_000e>`
     - ``0x0000``
     - ``RX_PACKET_SAMPLES``
     - RX packet size in samples.
   * - :ref:`0x000F <ssdr_rev2_reg_000f>`
     - ``0x03FC``
     - \-
     - Reserved.
   * - :ref:`0x0010 <ssdr_rev2_reg_0010>`
     - ``0x0001``
     - ``txant_pre``
     - Number of samples to delay turning on the internal TDD signal.
   * - :ref:`0x0011 <ssdr_rev2_reg_0011>`
     - ``0x0001``
     - ``txant_post``
     - Number of samples to delay turning off the internal TDD signal.
   * - :ref:`0x0012 <ssdr_rev2_reg_0012>`
     - ``0xFFFF``
     - \-
     - Reserved.
   * - :ref:`0x0013 <ssdr_rev2_reg_0013>`
     - ``0x6F6B``
     - ``LMS_TXRXEN_MUX_SEL, LMS1_RXEN, LMS1_TXEN, LMS1_TXNRX2, LMS1_TXNRX1, LMS1_CORE_LDO_EN, LMS1_RESET``
     - LMS7002 digital-interface and hard-enable control.
   * - :ref:`0x0014 <ssdr_rev2_reg_0014>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x0015 <ssdr_rev2_reg_0015>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0016 <ssdr_rev2_reg_0016>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0017 <ssdr_rev2_reg_0017>`
     - ``0x2340``
     - \-
     - Reserved.
   * - :ref:`0x0018 <ssdr_rev2_reg_0018>`
     - ``0x0003``
     - ``CORE_LDO_EN, EXT_CLK, TCXO_EN, LMS_RST``
     - LMS power, clock-source, and reset control.
   * - :ref:`0x0019 <ssdr_rev2_reg_0019>`
     - ``0x1000``
     - ``RX_PACKET_SIZE``
     - RX packet size in bytes.
   * - :ref:`0x001A <ssdr_rev2_reg_001a>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001B <ssdr_rev2_reg_001b>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001C <ssdr_rev2_reg_001c>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001D <ssdr_rev2_reg_001d>`
     - ``0x00FF``
     - \-
     - Reserved.
   * - :ref:`0x001E <ssdr_rev2_reg_001e>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x001F <ssdr_rev2_reg_001f>`
     - ``0xD090``
     - \-
     - Reserved.

.. _ssdr_rev2_regmap_pllcfg:

PLLCFG Registers (``0x0020`` - ``0x003F``)
------------------------------------------

.. list-table:: PLLCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0020 <ssdr_rev2_reg_0020>`
     - ``0x0000``
     - ``C1 Phase``
     - Phase value for PLL output clock 1.
   * - :ref:`0x0021 <ssdr_rev2_reg_0021>`
     - ``0x0001``
     - ``phcfg_error, phcfg_done, pllcfg_busy, pllcfg_done``
     - PLL and phase-configuration status.
   * - :ref:`0x0022 <ssdr_rev2_reg_0022>`
     - ``0x0000``
     - ``pllcfg_err, pll_lock``
     - PLL error and TX/RX PLL lock status.
   * - :ref:`0x0023 <ssdr_rev2_reg_0023>`
     - ``0x0000``
     - ``phcfg_mode, pll_ind, phcfg_start``
     - PLL reconfiguration control.
   * - :ref:`0x0024 <ssdr_rev2_reg_0024>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0025 <ssdr_rev2_reg_0025>`
     - ``0x01F0``
     - \-
     - Reserved.
   * - :ref:`0x0026 <ssdr_rev2_reg_0026>`
     - ``0x000A``
     - ``m_byp, n_byp``
     - PLL multiplier/divider bypass.
   * - :ref:`0x0027 <ssdr_rev2_reg_0027>`
     - ``0x0AAA``
     - ``c1_byp, c0_byp``
     - PLL output-divider bypass.
   * - :ref:`0x0028 <ssdr_rev2_reg_0028>`
     - ``0xAAAA``
     - \-
     - Reserved.
   * - :ref:`0x0029 <ssdr_rev2_reg_0029>`
     - ``0xAAAA``
     - \-
     - Reserved.
   * - :ref:`0x002A <ssdr_rev2_reg_002a>`
     - ``0x0000``
     - ``n_cnt``
     - PLL divider value.
   * - :ref:`0x002B <ssdr_rev2_reg_002b>`
     - ``0x0000``
     - ``m_cnt``
     - PLL multiplier value.
   * - :ref:`0x002C <ssdr_rev2_reg_002c>`
     - ``0x0000``
     - ``m_frac(LSB)``
     - PLL multiplier fractional value, LSB.
   * - :ref:`0x002D <ssdr_rev2_reg_002d>`
     - ``0x0000``
     - ``m_frac(MSB)``
     - PLL multiplier fractional value, MSB.
   * - :ref:`0x002E <ssdr_rev2_reg_002e>`
     - ``0x0000``
     - ``c0_cnt``
     - PLL output 0 divider value.
   * - :ref:`0x002F <ssdr_rev2_reg_002f>`
     - ``0x0000``
     - ``c1_cnt``
     - PLL output 1 divider value.
   * - :ref:`0x0030 <ssdr_rev2_reg_0030>`
     - ``0xEFFF``
     - ``auto_phcfg_smpls``
     - Number of samples used during auto phase configuration.
   * - :ref:`0x0031 <ssdr_rev2_regmap_pllcfg>` - :ref:`0x003F <ssdr_rev2_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved. 0x0038 - 0x003F are also reserved.

.. _ssdr_rev2_regmap_tstcfg:

TSTCFG Registers (``0x0060`` - ``0x007F``)
------------------------------------------

.. list-table:: TSTCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0060 <ssdr_rev2_reg_0060>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0061 <ssdr_rev2_reg_0061>`
     - ``0x0000``
     - ``test_en``
     - Test start bits for sys_clk, LMS_TX_CLK, and GNSS.
   * - :ref:`0x0062 <ssdr_rev2_regmap_tstcfg>` - :ref:`0x0064 <ssdr_rev2_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0065 <ssdr_rev2_reg_0065>`
     - ``0x0000``
     - ``test_cmplt``
     - Test-complete bits for sys_clk, LMS_TX_CLK, and GNSS.
   * - :ref:`0x0066 <ssdr_rev2_reg_0066>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0067 <ssdr_rev2_reg_0067>`
     - ``0x0000``
     - ``test_rez``
     - Test result bits; documented as not implemented.
   * - :ref:`0x0068 <ssdr_rev2_reg_0068>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0069 <ssdr_rev2_reg_0069>`
     - ``0x0000``
     - ``sys_clk_cnt``
     - Count of sys_clk cycles; changing values indicate an active clock.
   * - :ref:`0x006A <ssdr_rev2_regmap_tstcfg>` - :ref:`0x006D <ssdr_rev2_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x006E <ssdr_rev2_regmap_tstcfg>` - :ref:`0x0071 <ssdr_rev2_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0072 <ssdr_rev2_reg_0072>`
     - ``0x0000``
     - ``lms_tx_clk_cnt``
     - Low 16 bits of LMS TX clock test counter.
   * - :ref:`0x0073 <ssdr_rev2_reg_0073>`
     - ``0x0000``
     - ``lms_tx_clk_cnt``
     - Bits 23:16 of LMS TX clock test counter.
   * - :ref:`0x0074 <ssdr_rev2_regmap_tstcfg>` - :ref:`0x007C <ssdr_rev2_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x007D <ssdr_rev2_reg_007d>`
     - ``0xAAAA``
     - ``tx_tst_i``
     - TX test value for I channel.
   * - :ref:`0x007E <ssdr_rev2_reg_007e>`
     - ``0x5555``
     - ``tx_tst_q``
     - TX test value for Q channel.
   * - :ref:`0x007F <ssdr_rev2_reg_007f>`
     - ``0x0000``
     - \-
     - Reserved.

.. _ssdr_rev2_regmap_periphcfg:

PERIPHCFG Registers (``0x00C0`` - ``0x00D3``)
---------------------------------------------

.. list-table:: PERIPHCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x00C0 <ssdr_rev2_reg_00c0>`
     - ``0x0002``
     - ``board_gpio_ovrd``
     - GPIO override control. 0 = dedicated function, 1 = user override.
   * - :ref:`0x00C1 <ssdr_rev2_regmap_periphcfg>` - :ref:`0x00C3 <ssdr_rev2_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00C4 <ssdr_rev2_reg_00c4>`
     - ``0x0000``
     - ``board_gpio_dir``
     - Onboard GPIO direction. 0 = input, 1 = output.
   * - :ref:`0x00C5 <ssdr_rev2_reg_00c5>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00C6 <ssdr_rev2_reg_00c6>`
     - ``0x0000``
     - ``board_gpio_val``
     - GPIO output value. 0 = low, 1 = high.
   * - :ref:`0x00C7 <ssdr_rev2_regmap_periphcfg>` - :ref:`0x00C9 <ssdr_rev2_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00CA <ssdr_rev2_reg_00ca>`
     - ``0x0000``
     - ``periph_input_sel``
     - PPS input source select. 0 = GNSS_1PPS, 1 = 1PPSI_GPIO1.
   * - :ref:`0x00CB <ssdr_rev2_regmap_periphcfg>` - :ref:`0x00D1 <ssdr_rev2_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00D2 <ssdr_rev2_reg_00d2>`
     - ``0x0003``
     - ``PERIPH_EN``
     - Peripheral enables: clock-out routing, GNSS reset, GNSS standby.
   * - :ref:`0x00D3 <ssdr_rev2_reg_00d3>`
     - ``0x0000``
     - \-
     - Reserved.

.. _ssdr_rev2_regmap_memcfg:

MEMCFG Registers (``0xFFE0`` - ``0xFFFF``)
------------------------------------------

.. list-table:: MEMCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0xFFE0 <ssdr_rev2_regmap_memcfg>` - :ref:`0xFFFF <ssdr_rev2_regmap_memcfg>`
     - ``0x0000``
     - \-
     - All MEMCFG addresses in this documented range are reserved.

Register Bitfield Reference
-------------------

Bit fields below are shown from MSB to LSB where applicable.

.. _ssdr_rev2_reg_0000:

``0x0000`` - board_id
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0000`` | Default: ``0x001B`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_id [7:0]"},
     {"bits": 8, "name": "board_id [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0001:

``0x0001`` - major_rev
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0001`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "major_rev [7:0]"},
     {"bits": 8, "name": "major_rev [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0002:

``0x0002`` - compile_rev
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0002`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "compile_rev [7:0]"},
     {"bits": 8, "name": "compile_rev [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0003:

``0x0003`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0003`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "reserved [7:0]"},
     {"bits": 8, "name": "reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0004:

``0x0004`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0004`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0005:

``0x0005`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0005`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0006:

``0x0006`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0006`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0007:

``0x0007`` - ch_en
^^^^^^^^^^^^^^^^^^

Address: ``0x0007`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "ch_en [1:0]"},
     {"bits": 6, "name": "Reserved [7:2]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0008:

``0x0008`` - interface and synchronization control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0009:

``0x0009`` - txpct_loss_clr, smpl_nr_clr
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0009`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - TX packet-loss flag clear and timestamp reset control.

.. _ssdr_rev2_reg_000a:

``0x000A`` - RF and TDD control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_000b:

``0x000B`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000B`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_000c:

``0x000C`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000C`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_000d:

``0x000D`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000D`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_000e:

``0x000E`` - RX_PACKET_SAMPLES
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000E`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "RX_PACKET_SAMPLES [7:0]"},
     {"bits": 8, "name": "RX_PACKET_SAMPLES [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``RX_PACKET_SAMPLES``
     -  
     - RX packet size in samples.

.. _ssdr_rev2_reg_000f:

``0x000F`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000F`` | Default: ``0x03FC`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0010:

``0x0010`` - txant_pre
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0010`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "txant_pre [7:0]"},
     {"bits": 8, "name": "txant_pre [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0011:

``0x0011`` - txant_post
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0011`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "txant_post [7:0]"},
     {"bits": 8, "name": "txant_post [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0012:

``0x0012`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0012`` | Default: ``0xFFFF`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0013:

``0x0013`` - LMS_TXRXEN_MUX_SEL, LMS1_RXEN, LMS1_TXEN, LMS1_TXNRX2, LMS1_TXNRX1, LMS1_CORE_LDO_EN, LMS1_RESET
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0013`` | Default: ``0x6F6B`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - LMS7002 digital-interface and hard-enable control.

.. _ssdr_rev2_reg_0014:

``0x0014`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0014`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0015:

``0x0015`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0015`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0016:

``0x0016`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0016`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0017:

``0x0017`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0017`` | Default: ``0x2340`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0018:

``0x0018`` - LMS and clock control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0018`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "LMS_RST"},
     {"bits": 1, "name": "TCXO_EN"},
     {"bits": 1, "name": "EXT_CLK"},
     {"bits": 1, "name": "CORE_LDO_EN"},
     {"bits": 4, "name": "Reserved [7:4]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0019:

``0x0019`` - RX_PACKET_SIZE
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0019`` | Default: ``0x1000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "RX_PACKET_SIZE [7:0]"},
     {"bits": 8, "name": "RX_PACKET_SIZE [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_001a:

``0x001A`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_001b:

``0x001B`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001B`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_001c:

``0x001C`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001C`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_001d:

``0x001D`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001D`` | Default: ``0x00FF`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_001e:

``0x001E`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001E`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_001f:

``0x001F`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001F`` | Default: ``0xD090`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0020:

``0x0020`` - C1 Phase
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0020`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0021:

``0x0021`` - PLL status
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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0022:

``0x0022`` - pllcfg_err, pll_lock
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0022`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - PLL error and TX/RX PLL lock status.

.. _ssdr_rev2_reg_0023:

``0x0023`` - PLL control
^^^^^^^^^^^^^^^^^^^^^^^^

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0024:

``0x0024`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0024`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0025:

``0x0025`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0025`` | Default: ``0x01F0`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0026:

``0x0026`` - m_byp, n_byp
^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0026`` | Default: ``0x000A`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - PLL multiplier/divider bypass.

.. _ssdr_rev2_reg_0027:

``0x0027`` - c1_byp, c0_byp
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0027`` | Default: ``0x0AAA`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - PLL output-divider bypass.

.. _ssdr_rev2_reg_0028:

``0x0028`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0028`` | Default: ``0xAAAA`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0029:

``0x0029`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0029`` | Default: ``0xAAAA`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_002a:

``0x002A`` - n_cnt
^^^^^^^^^^^^^^^^^^

Address: ``0x002A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "n_cnt [7:0]"},
     {"bits": 8, "name": "n_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_002b:

``0x002B`` - m_cnt
^^^^^^^^^^^^^^^^^^

Address: ``0x002B`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "m_cnt [7:0]"},
     {"bits": 8, "name": "m_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_002c:

``0x002C`` - m_frac(LSB)
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x002C`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - PLL multiplier fractional value, LSB.

.. _ssdr_rev2_reg_002d:

``0x002D`` - m_frac(MSB)
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x002D`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "value [7:0]"},
     {"bits": 8, "name": "value [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - PLL multiplier fractional value, MSB.

.. _ssdr_rev2_reg_002e:

``0x002E`` - c0_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x002E`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c0_cnt [7:0]"},
     {"bits": 8, "name": "c0_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_002f:

``0x002F`` - c1_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x002F`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c1_cnt [7:0]"},
     {"bits": 8, "name": "c1_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0030:

``0x0030`` - auto_phcfg_smpls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0030`` | Default: ``0xEFFF`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "auto_phcfg_smpls [7:0]"},
     {"bits": 8, "name": "auto_phcfg_smpls [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``auto_phcfg_smpls``
     -  
     - Number of samples used during auto phase configuration.

.. _ssdr_rev2_reg_0060:

``0x0060`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0060`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0061:

``0x0061`` - test enable
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0061`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "test_en[0]"},
     {"bits": 1, "name": "test_en[1]"},
     {"bits": 1, "name": "test_en[2]"},
     {"bits": 5, "name": "Reserved [7:3]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0065:

``0x0065`` - test complete
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0065`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "sys_clk_test"},
     {"bits": 1, "name": "test_cmplt[1]"},
     {"bits": 1, "name": "test_cmplt[2]"},
     {"bits": 5, "name": "Reserved [7:3]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - ``sys_clk_test``
     - status
     - System clock test complete.

.. _ssdr_rev2_reg_0066:

``0x0066`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0066`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0067:

``0x0067`` - test_rez
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0067`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "test_rez [7:0]"},
     {"bits": 8, "name": "test_rez [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``test_rez``
     -  
     - Test result bits; documented as not implemented.

.. _ssdr_rev2_reg_0068:

``0x0068`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0068`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_0069:

``0x0069`` - sys_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0069`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "sys_clk_cnt [7:0]"},
     {"bits": 8, "name": "sys_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - Count of sys_clk cycles; changing values indicate an active clock.

.. _ssdr_rev2_reg_0072:

``0x0072`` - lms_tx_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0072`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lms_tx_clk_cnt [7:0]"},
     {"bits": 8, "name": "lms_tx_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_0073:

``0x0073`` - lms_tx_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0073`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lms_tx_clk_cnt [7:0]"},
     {"bits": 8, "name": "lms_tx_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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
     - Bits 23:16 of LMS TX clock test counter.

.. _ssdr_rev2_reg_007d:

``0x007D`` - tx_tst_i
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007D`` | Default: ``0xAAAA`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "tx_tst_i [7:0]"},
     {"bits": 8, "name": "tx_tst_i [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``tx_tst_i``
     -  
     - TX test value for I channel.

.. _ssdr_rev2_reg_007e:

``0x007E`` - tx_tst_q
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007E`` | Default: ``0x5555`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "tx_tst_q [7:0]"},
     {"bits": 8, "name": "tx_tst_q [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``tx_tst_q``
     -  
     - TX test value for Q channel.

.. _ssdr_rev2_reg_007f:

``0x007F`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007F`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_00c0:

``0x00C0`` - board_gpio_ovrd
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C0`` | Default: ``0x0002`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "board_gpio_ovrd"},
     {"bits": 7, "name": "Reserved [7:1]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[0]``
     - ``board_gpio_ovrd``
     - 0 = dedicated function, 1 = user override
     - GPIO override control.

.. _ssdr_rev2_reg_00c4:

``0x00C4`` - board_gpio_dir
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C4`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_dir [7:0]"},
     {"bits": 8, "name": "board_gpio_dir [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``board_gpio_dir``
     -  
     - Onboard GPIO direction. 0 = input, 1 = output.

.. _ssdr_rev2_reg_00c5:

``0x00C5`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C5`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.

.. _ssdr_rev2_reg_00c6:

``0x00C6`` - board_gpio_val
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C6`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_val [7:0]"},
     {"bits": 8, "name": "board_gpio_val [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``board_gpio_val``
     -  
     - GPIO output value. 0 = low, 1 = high.

.. _ssdr_rev2_reg_00ca:

``0x00CA`` - periph_input_sel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CA`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "periph_input_sel [7:0]"},
     {"bits": 8, "name": "periph_input_sel [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``periph_input_sel``
     -  
     - PPS input source select. 0 = GNSS_1PPS, 1 = 1PPSI_GPIO1.

.. _ssdr_rev2_reg_00d2:

``0x00D2`` - PERIPH_EN
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D2`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 3, "name": "PERIPH_EN [2:0]"},
     {"bits": 5, "name": "Reserved [7:3]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

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

.. _ssdr_rev2_reg_00d3:

``0x00D3`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D3`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "Reserved [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - \-
     -  
     - Reserved.
