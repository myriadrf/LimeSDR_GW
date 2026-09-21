CA23 Host Register Reference
============================

This page is generated from CSV sources.

Quick Navigation
----------------

.. list-table:: Module overview
   :header-rows: 1
   :widths: 20 16 64

   * - Module
     - Address range
     - Typical use
   * - :ref:`FPGACFG <ca23_regmap_fpgacfg>`
     - ``0x0000`` - ``0x001F``
     - Board ID/revision, stream mode, RF/TDD controls, LMS enable/reset
   * - :ref:`PLLCFG <ca23_regmap_pllcfg>`
     - ``0x0020`` - ``0x003F``
     - PLL status, divider/multiplier values, phase configuration
   * - :ref:`TSTCFG <ca23_regmap_tstcfg>`
     - ``0x0060`` - ``0x007F``
     - Clock/test control and status counters
   * - :ref:`PERIPHCFG <ca23_regmap_periphcfg>`
     - ``0x00C0`` - ``0x00D3``
     - GPIO and peripheral routing/enable controls
   * - :ref:`GNSSCFG <ca23_regmap_gnsscfg>`
     - ``0x0100`` - ``0x011F``
     - GNSS configuration and status (legacy)
   * - :ref:`TIMECFG <ca23_regmap_timecfg>`
     - ``0x0280`` - ``0x029F``
     - GNSS time information and related
   * - :ref:`MEMCFG <ca23_regmap_memcfg>`
     - ``0xFFE0`` - ``0xFFFF``
     - Reserved memory-mapped window

.. _ca23_regmap_fpgacfg:

FPGACFG Registers (``0x0000`` - ``0x001F``)
-------------------------------------------

.. list-table:: FPGACFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0000 <ca23_reg_0000>`
     - ``0x001B``
     - ``board_id``
     - Board ID
   * - :ref:`0x0001 <ca23_reg_0001>`
     -  
     - ``major_rev``
     - Major revision
   * - :ref:`0x0002 <ca23_reg_0002>`
     -  
     - ``compile_rev``
     - Compile revision
   * - :ref:`0x0003 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0004 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0005 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0006 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0007 <ca23_reg_0007>`
     - ``0x0003``
     - ``ch_en``
     - Channel enable
   * - :ref:`0x0008 <ca23_reg_0008>`
     - ``0x0102``
     - ``stream_ctrl``
     - Sample width and interface mode control.
   * - :ref:`0x0009 <ca23_reg_0009>`
     - ``0x0003``
     - ``txpct_ctrl``
     - TX packet-loss flag clear and timestamp reset control.
   * - :ref:`0x000A <ca23_reg_000a>`
     - ``0x0000``
     - ``rf_tdd_ctrl``
     - RF switch, TDD, pattern generation, and unified RX/TX enable control.
   * - :ref:`0x000B <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000C <ca23_regmap_fpgacfg>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x000D <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000E <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x000F <ca23_regmap_fpgacfg>`
     - ``0x03FC``
     - \-
     - Reserved.
   * - :ref:`0x0010 <ca23_reg_0010>`
     - ``0x0001``
     - ``txant_pre``
     - Number of samples to delay turning on the internal TDD signal.
   * - :ref:`0x0011 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0012 <ca23_regmap_fpgacfg>`
     - ``0xFFFF``
     - \-
     - Reserved.
   * - :ref:`0x0013 <ca23_reg_0013>`
     - ``0x6F6B``
     - ``lms_misc_ctrl``
     - LMS7002 digital-interface and hard-enable control.
   * - :ref:`0x0014 <ca23_regmap_fpgacfg>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x0015 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0016 <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0017 <ca23_regmap_fpgacfg>`
     - ``0x2340``
     - \-
     - Reserved.
   * - :ref:`0x0018 <ca23_reg_0018>`
     - ``0x0003``
     - ``lms_clk_ctrl``
     - LMS power, clock-source, and reset control.
   * - :ref:`0x0019 <ca23_reg_0019>`
     - ``0x1000``
     - ``rx_packet_size``
     - RX packet size in bytes.
   * - :ref:`0x001A <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001B <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001C <ca23_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001D <ca23_regmap_fpgacfg>`
     - ``0x00FF``
     - \-
     - Reserved.
   * - :ref:`0x001E <ca23_regmap_fpgacfg>`
     - ``0x0003``
     - \-
     - Reserved.
   * - :ref:`0x001F <ca23_regmap_fpgacfg>`
     - ``0xD090``
     - \-
     - Reserved.

.. _ca23_regmap_pllcfg:

PLLCFG Registers (``0x0020`` - ``0x003F``)
------------------------------------------

.. list-table:: PLLCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0020 <ca23_reg_0020>`
     - ``0x0000``
     - ``c1 phase``
     - Phase value for PLL output clock 1.
   * - :ref:`0x0021 <ca23_reg_0021>`
     - ``0x0001``
     - ``pll_status``
     - PLL and phase-configuration status.
   * - :ref:`0x0022 <ca23_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0023 <ca23_reg_0023>`
     - ``0x0000``
     - ``pll_ctrl``
     - PLL reconfiguration control.
   * - :ref:`0x0024 <ca23_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0025 <ca23_regmap_pllcfg>`
     - ``0x01F0``
     - \-
     - Reserved.
   * - :ref:`0x0026 <ca23_reg_0026>`
     - ``0x000A``
     - ``mn_bypass_ctrl``
     - PLL multiplier/divider bypass.
   * - :ref:`0x0027 <ca23_reg_0027>`
     - ``0x0AAA``
     - ``c01_bypass_ctrl``
     - PLL output-divider bypass.
   * - :ref:`0x0028 <ca23_regmap_pllcfg>`
     - ``0xAAAA``
     - \-
     - Reserved.
   * - :ref:`0x0029 <ca23_regmap_pllcfg>`
     - ``0xAAAA``
     - \-
     - Reserved.
   * - :ref:`0x002A <ca23_reg_002a>`
     - ``0x0000``
     - ``n_cnt``
     - PLL divider value.
   * - :ref:`0x002B <ca23_reg_002b>`
     - ``0x0000``
     - ``m_cnt``
     - PLL multiplier value.
   * - :ref:`0x002C <ca23_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x002D <ca23_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x002E <ca23_reg_002e>`
     - ``0x0000``
     - ``c0_cnt``
     - PLL output 0 divider value.
   * - :ref:`0x002F <ca23_reg_002f>`
     - ``0x0000``
     - ``c1_cnt``
     - PLL output 1 divider value.
   * - :ref:`0x0030 <ca23_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0031 <ca23_regmap_pllcfg>` - :ref:`0x003F <ca23_regmap_pllcfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _ca23_regmap_tstcfg:

TSTCFG Registers (``0x0060`` - ``0x007F``)
------------------------------------------

.. list-table:: TSTCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0060 <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0061 <ca23_reg_0061>`
     - ``0x0000``
     - ``test_en``
     - Test start bits for sys_clk, LMS_TX_CLK, and GNSS.
   * - :ref:`0x0062 <ca23_regmap_tstcfg>` - :ref:`0x0064 <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0065 <ca23_reg_0065>`
     - ``0x0000``
     - ``test_cmplt``
     - Test-complete bits for sys_clk, LMS_TX_CLK, and GNSS.
   * - :ref:`0x0066 <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0067 <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0068 <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0069 <ca23_reg_0069>`
     - ``0x0000``
     - ``sys_clk_cnt``
     - Count of sys_clk cycles
   * - :ref:`0x006A <ca23_regmap_tstcfg>` - :ref:`0x006D <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x006E <ca23_regmap_tstcfg>` - :ref:`0x0071 <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0072 <ca23_reg_0072>`
     - ``0x0000``
     - ``lms_tx_clk_cnt``
     - Low 16 bits of LMS TX clock test counter.
   * - :ref:`0x0073 <ca23_reg_0073>`
     - ``0x0000``
     - ``lms_tx_clk_cnt``
     - Bits 23:16 of LMS TX clock test counter.
   * - :ref:`0x0074 <ca23_regmap_tstcfg>` - :ref:`0x007C <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x007D <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x007E <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x007F <ca23_regmap_tstcfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _ca23_regmap_periphcfg:

PERIPHCFG Registers (``0x00C0`` - ``0x00D3``)
---------------------------------------------

.. list-table:: PERIPHCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0014 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - ``rx1_sw``
     - RX1 MIPI RF switch control
   * - :ref:`0x0015 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - ``trx1_sw``
     - TRX1 MIPI RF switch control
   * - :ref:`0x0016 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - ``trx1_ant_sw``
     - TRX1 antenna MIPI RF switch control
   * - :ref:`0x0017 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - ``rx2_trx2_sw``
     - RX2 and TRX2 MIPI RF switch control
   * - :ref:`0x00C0 <ca23_reg_00c0>`
     - ``0x0000``
     - ``board_gpio_ovrd``
     - Board GPIO override control.
   * - :ref:`0x00C1 <ca23_regmap_periphcfg>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C2 <ca23_reg_00c2>`
     - ``0x0000``
     - ``board_gpio_rd``
     - Board GPIO read value.
   * - :ref:`0x00C3 <ca23_regmap_periphcfg>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C4 <ca23_reg_00c4>`
     - ``0x0000``
     - ``board_gpio_dir``
     - Board GPIO direction control (1=output).
   * - :ref:`0x00C5 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00C6 <ca23_reg_00c6>`
     - ``0x0000``
     - ``board_gpio_val``
     - Board GPIO output value.
   * - :ref:`0x00C7 <ca23_regmap_periphcfg>` - :ref:`0x00C9 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00CA <ca23_reg_00ca>`
     - ``0x0000``
     - ``periph_input_sel``
     - PPS input source select. 0 = GNSS_1PPS, 1 = 1PPSI_GPIO1.
   * - :ref:`0x00CB <ca23_regmap_periphcfg>` - :ref:`0x00D1 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x00D2 <ca23_reg_00d2>`
     - ``0x0003``
     - ``periph_en``
     - Peripheral enables: clock-out routing, GNSS reset, GNSS standby.
   * - :ref:`0x00D3 <ca23_regmap_periphcfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _ca23_regmap_gnsscfg:

GNSSCFG Registers (``0x0100`` - ``0x011F``)
-------------------------------------------

.. list-table:: GNSSCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0100 <ca23_regmap_gnsscfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0101 <ca23_regmap_gnsscfg>`
     - ``0x0000``
     - \-
     - Reserved
   * - :ref:`0x0102 <ca23_reg_0102>`
     - ``0x0000``
     - ``gnss_utc_mm_ss1``
     - UTC of position fix (MM-SS1)
   * - :ref:`0x0103 <ca23_reg_0103>`
     - ``0x0003``
     - ``gnss_utc_hh``
     - UTC of position fix (HH)
   * - :ref:`0x0104 <ca23_reg_0104>`
     - ``0x0000``
     - ``gnss_status``
     - Data valid status
   * - :ref:`0x0105 <ca23_reg_0105>`
     - ``0x0022``
     - ``gnss_lat_ll1_ll0``
     - Latitude (LL1-LL0)
   * - :ref:`0x0106 <ca23_reg_0106>`
     - ``0x0000``
     - ``gnss_lat_ll3_ll2``
     - Latitude (LL3-LL2)
   * - :ref:`0x0107 <ca23_reg_0107>`
     - ``0x0164``
     - ``gnss_lat_n_s``
     - Latitude N/S
   * - :ref:`0x0108 <ca23_reg_0108>`
     - ``0x0000``
     - ``gnss_long_yy1_yy0``
     - Longitude (YY1-YY0)
   * - :ref:`0x0109 <ca23_reg_0109>`
     - ``0x0000``
     - ``gnss_long_yy3_yy2``
     - Longitude (YY3-YY2)
   * - :ref:`0x010A <ca23_reg_010a>`
     - ``0x0000``
     - ``gnss_long_y4``
     - Longitude (Y4)
   * - :ref:`0x010B <ca23_reg_010b>`
     - ``0x0000``
     - ``gnss_long_e_w``
     - Longitude E/W
   * - :ref:`0x010C <ca23_reg_010c>`
     - ``0x0000``
     - ``gnss_speed_xx1_xx0``
     - Speed over ground (XX1-XX0)
   * - :ref:`0x010D <ca23_reg_010d>`
     - ``0x0000``
     - ``gnss_speed_xx2``
     - Speed over ground (XX2)
   * - :ref:`0x010E <ca23_reg_010e>`
     - ``0x0000``
     - ``gnss_course_xx1_xx0``
     - Course over ground (XX1-XX0)
   * - :ref:`0x010F <ca23_reg_010f>`
     - ``0x0000``
     - ``gnss_course_x2``
     - Course over ground (X2)
   * - :ref:`0x0110 <ca23_reg_0110>`
     - ``0x0000``
     - ``gnss_date_mm_yy``
     - Date (MM-YY)
   * - :ref:`0x0111 <ca23_reg_0111>`
     - ``0x0000``
     - ``gnss_date_dd``
     - Date (DD)
   * - :ref:`0x0112 <ca23_regmap_gnsscfg>` - :ref:`0x0113 <ca23_regmap_gnsscfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0114 <ca23_reg_0114>`
     - ``0x0000``
     - ``gnss_fix``
     - Combined fixes status
   * - :ref:`0x0115 <ca23_regmap_gnsscfg>` - :ref:`0x011F <ca23_regmap_gnsscfg>`
     - ``0x0000``
     - \-
     - Reserved.

.. _ca23_regmap_timecfg:

TIMECFG Registers (``0x0280`` - ``0x029F``)
-------------------------------------------

.. list-table:: TIMECFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0280 <ca23_reg_0280>`
     - ``0x0000``
     - ``timestamp_settings``
     - Timestamp Settings
   * - :ref:`0x0281 <ca23_reg_0281>`
     - ``0x0000``
     - ``rx_delay_mode``
     - RX delay mode
   * - :ref:`0x0282 <ca23_reg_0282>`
     - ``0x0000``
     - ``tx_delay_mode``
     - TX delay mode
   * - :ref:`0x0283 <ca23_reg_0283>`
     - ``0x0000``
     - ``main_time_min_sec``
     - Main time (minutes and seconds)
   * - :ref:`0x0284 <ca23_reg_0284>`
     - ``0x0000``
     - ``main_time_mon_day_hrs``
     - Main time (months days and hours)
   * - :ref:`0x0285 <ca23_reg_0285>`
     - ``0x0000``
     - ``main_time_yrs``
     - Main time (years)
   * - :ref:`0x0286 <ca23_reg_0286>`
     - ``0x0000``
     - ``rx_time_min_sec``
     - RX start time (minutes and seconds)
   * - :ref:`0x0287 <ca23_reg_0287>`
     - ``0x0000``
     - ``rx_time_mon_day_hrs``
     - RX start time (months days and hours)
   * - :ref:`0x0288 <ca23_reg_0288>`
     - ``0x0000``
     - ``rx_time_yrs``
     - RX start time (years)
   * - :ref:`0x0289 <ca23_reg_0289>`
     - ``0x0000``
     - ``tx_time_min_sec``
     - TX start time (minutes and seconds)
   * - :ref:`0x028A <ca23_reg_028a>`
     - ``0x0000``
     - ``tx_time_mon_day_hrs``
     - TX start time (months days and hours)
   * - :ref:`0x028B <ca23_reg_028b>`
     - ``0x0000``
     - ``tx_time_yrs``
     - TX start time (years)

.. _ca23_regmap_memcfg:

MEMCFG Registers (``0xFFE0`` - ``0xFFFF``)
------------------------------------------

.. list-table:: MEMCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0xFFE0 <ca23_regmap_memcfg>` - :ref:`0xFFFF <ca23_regmap_memcfg>`
     - ``0x0000``
     - \-
     - All MEMCFG addresses in this documented range are reserved.

Register Bitfield Reference
-------------------

Bit fields below are shown from MSB to LSB where applicable.

.. _ca23_reg_0000:

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

.. _ca23_reg_0001:

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

.. _ca23_reg_0002:

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

.. _ca23_reg_0007:

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

.. _ca23_reg_0008:

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

.. _ca23_reg_0009:

``0x0009`` - txpct_ctrl
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0009`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "smpl_nr_clr"},
     {"bits": 1, "name": "txpct_loss_clr"},
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
   * - ``[1]``
     - ``txpct_loss_clr``
     - 0 - Normal operation (Default) 1 - Rising edge clears flag
     - TX packets dropping flag clear
   * - ``[0]``
     - ``smpl_nr_clr``
     - 0 - Normal operation (Default) 1 - Timestamp is cleared
     - Reset_timestamp

.. _ca23_reg_000a:

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

.. _ca23_reg_0010:

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

.. _ca23_reg_0013:

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

.. _ca23_reg_0018:

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

.. _ca23_reg_0019:

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

.. _ca23_reg_0020:

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

.. _ca23_reg_0021:

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

.. _ca23_reg_0023:

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

.. _ca23_reg_0026:

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

.. _ca23_reg_0027:

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

.. _ca23_reg_002a:

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

.. _ca23_reg_002b:

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

.. _ca23_reg_002e:

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

.. _ca23_reg_002f:

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

.. _ca23_reg_0061:

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

.. _ca23_reg_0065:

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

.. _ca23_reg_0069:

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

.. _ca23_reg_0072:

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

.. _ca23_reg_0073:

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

.. _ca23_reg_00c0:

``0x00C0`` - board_gpio_ovrd
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C0`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "board_gpio_ovrd [3:0]"},
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
   * - ``[3:0]``
     - ``board_gpio_ovrd``
     - 0=Dedicated function,1=Overridden by user
     - GPIO override bits for FPGA_GPIO[3:0].

.. _ca23_reg_00c2:

``0x00C2`` - board_gpio_rd
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C2`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "board_gpio_rd [3:0]"},
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
   * - ``[3:0]``
     - ``board_gpio_rd``
     - 0=Low,1=High
     - GPIO readback bits for FPGA_GPIO[3:0].

.. _ca23_reg_00c4:

``0x00C4`` - board_gpio_dir
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C4`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "board_gpio_dir [3:0]"},
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
   * - ``[3:0]``
     - ``board_gpio_dir``
     - 0=Input,1=Output
     - GPIO direction bits for FPGA_GPIO[3:0] (1=output).

.. _ca23_reg_00c6:

``0x00C6`` - board_gpio_val
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C6`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "board_gpio_val [3:0]"},
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
   * - ``[3:0]``
     - ``board_gpio_val``
     - 0=Low,1=High
     - GPIO output value bits for FPGA_GPIO[3:0].

.. _ca23_reg_00ca:

``0x00CA`` - periph_input_sel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CA`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "periph_input_sel [1:0]"},
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
     - ``periph_input_sel``
     - 0 = GNSS_1PPS, 1 = 1PPSI_GPIO1.
     - PPS input source select.

.. _ca23_reg_00d2:

``0x00D2`` - periph_en
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D2`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "PERIPH_EN[0]"},
     {"bits": 1, "name": "PERIPH_EN[1]"},
     {"bits": 1, "name": "PERIPH_EN[2]"},
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
     - ``PERIPH_EN[2]``
     - 0 – CLK OUT → NC (Default) 1 – CLK OUT → mPCIe SMB_CLK (pin 30)
     - EN_GPIO (mPCIe)
   * - ``[1]``
     - ``PERIPH_EN[1]``
     - 0 – Reset 1 – Normal operation (Default)
     - GNSS_HW_R
   * - ``[0]``
     - ``PERIPH_EN[0]``
     - 0 – Standby mode 1 – Normal mode (Default)
     - GNSS_HW_S

.. _ca23_reg_0102:

``0x0102`` - gnss_utc_mm_ss1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0102`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ss1 [7:0]"},
     {"bits": 8, "name": "mm [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``mm``
     -  
     - UTC of position fix (MM) in BCD format.
   * - ``[7:0]``
     - ``ss1``
     -  
     - UTC of position fix (SS1) in BCD format.

.. _ca23_reg_0103:

``0x0103`` - gnss_utc_hh
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0103`` | Default: ``0x0003`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "hh [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``hh``
     -  
     - UTC of position fix (HH) in BCD format.

.. _ca23_reg_0104:

``0x0104`` - gnss_status
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0104`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "status"},
     {"bits": 7, "name": "Reserved [7:1]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[0]``
     - ``status``
     - 1 = Data valid, 0 = Warning
     - GNSS data valid status.

.. _ca23_reg_0105:

``0x0105`` - gnss_lat_ll1_ll0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0105`` | Default: ``0x0022`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ll0 [7:0]"},
     {"bits": 8, "name": "ll1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``ll1``
     -  
     - Latitude (LL1) in BCD format.
   * - ``[7:0]``
     - ``ll0``
     -  
     - Latitude (LL0) in BCD format.

.. _ca23_reg_0106:

``0x0106`` - gnss_lat_ll3_ll2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0106`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ll2 [7:0]"},
     {"bits": 8, "name": "ll3 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``ll3``
     -  
     - Latitude (LL3) in BCD format.
   * - ``[7:0]``
     - ``ll2``
     -  
     - Latitude (LL2) in BCD format.

.. _ca23_reg_0107:

``0x0107`` - gnss_lat_n_s
^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0107`` | Default: ``0x0164`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "n_s"},
     {"bits": 7, "name": "Reserved [7:1]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[0]``
     - ``n_s``
     - 0 = N, 1 = S
     - Latitude North/South indicator.

.. _ca23_reg_0108:

``0x0108`` - gnss_long_yy1_yy0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0108`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "yy0 [7:0]"},
     {"bits": 8, "name": "yy1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``yy1``
     -  
     - Longitude (YY1) in BCD format.
   * - ``[7:0]``
     - ``yy0``
     -  
     - Longitude (YY0) in BCD format.

.. _ca23_reg_0109:

``0x0109`` - gnss_long_yy3_yy2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0109`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "yy2 [7:0]"},
     {"bits": 8, "name": "yy3 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``yy3``
     -  
     - Longitude (YY3) in BCD format.
   * - ``[7:0]``
     - ``yy2``
     -  
     - Longitude (YY2) in BCD format.

.. _ca23_reg_010a:

``0x010A`` - gnss_long_y4
^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x010A`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "y4 [3:0]"},
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
   * - ``[3:0]``
     - ``y4``
     -  
     - Longitude (Y4) in BCD format.

.. _ca23_reg_010b:

``0x010B`` - gnss_long_e_w
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x010B`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "e_w"},
     {"bits": 7, "name": "Reserved [7:1]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[0]``
     - ``e_w``
     - 0 = E, 1 = W
     - Longitude East/West indicator.

.. _ca23_reg_010c:

``0x010C`` - gnss_speed_xx1_xx0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x010C`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "xx0 [7:0]"},
     {"bits": 8, "name": "xx1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``xx1``
     -  
     - Speed over ground (XX1) in BCD format.
   * - ``[7:0]``
     - ``xx0``
     -  
     - Speed over ground (XX0) in BCD format.

.. _ca23_reg_010d:

``0x010D`` - gnss_speed_xx2
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x010D`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "xx2 [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``xx2``
     -  
     - Speed over ground (XX2) in BCD format.

.. _ca23_reg_010e:

``0x010E`` - gnss_course_xx1_xx0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x010E`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "xx0 [7:0]"},
     {"bits": 8, "name": "xx1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``xx1``
     -  
     - Course over ground (XX1) in BCD format.
   * - ``[7:0]``
     - ``xx0``
     -  
     - Course over ground (XX0) in BCD format.

.. _ca23_reg_010f:

``0x010F`` - gnss_course_x2
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x010F`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "x2 [3:0]"},
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
   * - ``[3:0]``
     - ``x2``
     -  
     - Course over ground (X2) in BCD format.

.. _ca23_reg_0110:

``0x0110`` - gnss_date_mm_yy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0110`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "yy [7:0]"},
     {"bits": 8, "name": "mm [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``mm``
     -  
     - Date (MM) in BCD format.
   * - ``[7:0]``
     - ``yy``
     -  
     - Date (YY) in BCD format.

.. _ca23_reg_0111:

``0x0111`` - gnss_date_dd
^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0111`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "dd [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``dd``
     -  
     - Date (DD) in BCD format.

.. _ca23_reg_0114:

``0x0114`` - gnss_fix
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0114`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "glgsa_fix [3:0]"},
     {"bits": 4, "name": "gpgsa_fix [7:4]"},
     {"bits": 4, "name": "gbgsa_fix [11:8]"},
     {"bits": 4, "name": "gagsa_fix [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:12]``
     - ``gagsa_fix``
     - 1=No fix, 2=2D, 3=3D
     - Galileo fix status.
   * - ``[11:8]``
     - ``gbgsa_fix``
     - 1=No fix, 2=2D, 3=3D
     - BeiDou fix status.
   * - ``[7:4]``
     - ``gpgsa_fix``
     - 1=No fix, 2=2D, 3=3D
     - GPS fix status.
   * - ``[3:0]``
     - ``glgsa_fix``
     - 1=No fix, 2=2D, 3=3D
     - GLONASS fix status.

.. _ca23_reg_0280:

``0x0280`` - timestamp_settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0280`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "TS_SEL"},
     {"bits": 7, "name": "Reserved [7:1]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[0]``
     - ``TS_SEL``
     - 0: Classic, 1: Mixed
     - Timestamp selector

.. _ca23_reg_0281:

``0x0281`` - rx_delay_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0281`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "rx_del_sel [1:0]"},
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
     - ``rx_del_sel``
     - 0: No Delay, 1: Delay until PPS, 2: Delay until PPS and Valid
     - RX enable signal delay mode

.. _ca23_reg_0282:

``0x0282`` - tx_delay_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0282`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "tx_del_sel [1:0]"},
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
     - ``tx_del_sel``
     - 0: No Delay, 1: Delay until PPS, 2: Delay until PPS and Valid
     - TX enable signal delay mode

.. _ca23_reg_0283:

``0x0283`` - main_time_min_sec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0283`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 6, "name": "sec [5:0]"},
     {"bits": 2, "name": "min [7:6]"},
     {"bits": 4, "name": "min [11:8]"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11:6]``
     - ``min``
     -  
     - Current time minutes
   * - ``[5:0]``
     - ``sec``
     -  
     - Current time seconds

.. _ca23_reg_0284:

``0x0284`` - main_time_mon_day_hrs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0284`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 5, "name": "hrs [4:0]"},
     {"bits": 3, "name": "day [7:5]"},
     {"bits": 2, "name": "day [9:8]"},
     {"bits": 4, "name": "mon [13:10]"},
     {"bits": 2, "name": "Reserved [15:14]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[13:10]``
     - ``mon``
     -  
     - Current time months
   * - ``[9:5]``
     - ``day``
     -  
     - Current time days
   * - ``[4:0]``
     - ``hrs``
     -  
     - Current time hours

.. _ca23_reg_0285:

``0x0285`` - main_time_yrs
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0285`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "yrs [7:0]"},
     {"bits": 4, "name": "yrs [11:8]"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11:0]``
     - ``yrs``
     -  
     - Current time years

.. _ca23_reg_0286:

``0x0286`` - rx_time_min_sec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0286`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 6, "name": "sec [5:0]"},
     {"bits": 2, "name": "min [7:6]"},
     {"bits": 4, "name": "min [11:8]"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11:6]``
     - ``min``
     -  
     - RX stream start time minutes
   * - ``[5:0]``
     - ``sec``
     -  
     - RX stream start time seconds

.. _ca23_reg_0287:

``0x0287`` - rx_time_mon_day_hrs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0287`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 5, "name": "hrs [4:0]"},
     {"bits": 3, "name": "day [7:5]"},
     {"bits": 2, "name": "day [9:8]"},
     {"bits": 4, "name": "mon [13:10]"},
     {"bits": 2, "name": "Reserved [15:14]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[13:10]``
     - ``mon``
     -  
     - RX stream start time months
   * - ``[9:5]``
     - ``day``
     -  
     - RX stream start time days
   * - ``[4:0]``
     - ``hrs``
     -  
     - RX stream start time hours

.. _ca23_reg_0288:

``0x0288`` - rx_time_yrs
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0288`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "yrs [7:0]"},
     {"bits": 4, "name": "yrs [11:8]"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11:0]``
     - ``yrs``
     -  
     - RX stream start time years

.. _ca23_reg_0289:

``0x0289`` - tx_time_min_sec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0289`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 6, "name": "sec [5:0]"},
     {"bits": 2, "name": "min [7:6]"},
     {"bits": 4, "name": "min [11:8]"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11:6]``
     - ``min``
     -  
     - TX stream start time minutes
   * - ``[5:0]``
     - ``sec``
     -  
     - TX stream start time seconds

.. _ca23_reg_028a:

``0x028A`` - tx_time_mon_day_hrs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x028A`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 5, "name": "hrs [4:0]"},
     {"bits": 3, "name": "day [7:5]"},
     {"bits": 2, "name": "day [9:8]"},
     {"bits": 4, "name": "mon [13:10]"},
     {"bits": 2, "name": "Reserved [15:14]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[13:10]``
     - ``mon``
     -  
     - TX stream start time months
   * - ``[9:5]``
     - ``day``
     -  
     - TX stream start time days
   * - ``[4:0]``
     - ``hrs``
     -  
     - TX stream start time hours

.. _ca23_reg_028b:

``0x028B`` - tx_time_yrs
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x028B`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "yrs [7:0]"},
     {"bits": 4, "name": "yrs [11:8]"},
     {"bits": 4, "name": "Reserved [15:12]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[11:0]``
     - ``yrs``
     -  
     - TX stream start time years
