LimeSDR Mini V1 Host Register Reference
=======================================

This page is generated from CSV sources.

Quick Navigation
----------------

.. list-table:: Module overview
   :header-rows: 1
   :widths: 20 16 64

   * - Module
     - Address range
     - Typical use
   * - :ref:`FPGACFG <mini_v1_regmap_fpgacfg>`
     - ``0x0000`` - ``0x001F``
     - Board ID, gateware revision, interface and LMS control
   * - :ref:`PLLCFG <mini_v1_regmap_pllcfg>`
     - ``0x0020`` - ``0x003F``
     - PLL status/control, divider and counter configuration
   * - :ref:`TSTCFG <mini_v1_regmap_tstcfg>`
     - ``0x0060`` - ``0x007F``
     - Built-in test control, status, and counters
   * - :ref:`PERIPHCFG <mini_v1_regmap_periphcfg>`
     - ``0x00C0`` - ``0x00DF``
     - GPIO and peripheral override/readback control

.. _mini_v1_regmap_fpgacfg:

FPGACFG Registers (``0x0000`` - ``0x001F``)
-------------------------------------------

.. list-table:: FPGACFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0000 <mini_v1_reg_0000>`
     - ``0x0011``
     - ``board_id``
     - Board identification number (LimeSDR Mini default 0x0011).
   * - :ref:`0x0001 <mini_v1_reg_0001>`
     -  
     - ``gw_ver``
     - Gateware version number.
   * - :ref:`0x0002 <mini_v1_reg_0002>`
     -  
     - ``gw_rev``
     - Gateware revision number.
   * - :ref:`0x0003 <mini_v1_reg_0003>`
     -  
     - ``bom_ver, hw_ver``
     - Board BOM version and hardware version.
   * - :ref:`0x0004 <mini_v1_reg_0004>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0005 <mini_v1_reg_0005>`
     - ``0x0000``
     - ``drct_clk_en``
     - Clock source selection for RX/TX interfaces.
   * - :ref:`0x0006 <mini_v1_reg_0006>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0007 <mini_v1_reg_0007>`
     - ``0x0303``
     - ``ch_en``
     - RX/TX MIMO channel enable control.
   * - :ref:`0x0008 <mini_v1_reg_0008>`
     - ``0x0102``
     - ``dlb_en, synch_dis, mimo_int_en, triq_pulse, ddr_en, mode, smpl_width``
     - DIQ interface control.
   * - :ref:`0x0009 <mini_v1_reg_0009>`
     - ``0x0003``
     - ``txpct_loss_clr, smpl_nr_clr``
     - Packet control: TX packet-loss clear and timestamp reset.
   * - :ref:`0x000A <mini_v1_reg_000a>`
     - ``0x0000``
     - ``tx_ptrn_en, rx_ptrn_en, tx_en, rx_en``
     - RX/TX module control.
   * - :ref:`0x000B <mini_v1_reg_000b>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x000C <mini_v1_reg_000c>`
     - ``0x0003``
     - ``wfm_ch_en``
     - WFM player channel enable control.
   * - :ref:`0x000D <mini_v1_reg_000d>`
     - ``0x0001``
     - ``wfm_load, wfm_play``
     - WFM player load/play control.
   * - :ref:`0x000E <mini_v1_reg_000e>`
     - ``0x0002``
     - ``wfm_smpl_width``
     - WFM player sample width control.
   * - :ref:`0x000F <mini_v1_reg_000f>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0010 <mini_v1_reg_0010>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0011 <mini_v1_reg_0011>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0012 <mini_v1_reg_0012>`
     - ``0xFFFF``
     - ``spi_ss``
     - Controlled SPI slave-select enable bits.
   * - :ref:`0x0013 <mini_v1_reg_0013>`
     - ``0x6F6F``
     - ``lms_misc``
     - LMS7002 MISC pin control.
   * - :ref:`0x0014 <mini_v1_reg_0014>`
     - ``0x0000``
     - \-
     - Reserved for lms3_4.
   * - :ref:`0x0015 <mini_v1_reg_0015>`
     - ``0x0000``
     - \-
     - Reserved for lms5_6.
   * - :ref:`0x0016 <mini_v1_reg_0016>`
     - ``0x0000``
     - \-
     - Reserved for lms7_8.
   * - :ref:`0x0017 <mini_v1_reg_0017>`
     - ``0x0000``
     - ``gpio``
     - GPIO for external periphery.
   * - :ref:`0x0018 <mini_v1_reg_0018>`
     - ``0x0001``
     - ``dev_ctrl0``
     - Device control bit 0 (not used).
   * - :ref:`0x0019 <mini_v1_reg_0019>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x001A <mini_v1_reg_001a>`
     - ``0x0000``
     - ``fpga_led_ctrl``
     - Onboard FPGA LED override/control.
   * - :ref:`0x001B <mini_v1_reg_001b>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001C <mini_v1_reg_001c>`
     - ``0x0000``
     - ``fx3_led_ctrl``
     - Onboard FX3 LED override/control.
   * - :ref:`0x001D <mini_v1_reg_001d>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001E <mini_v1_reg_001e>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x001F <mini_v1_reg_001f>`
     - ``0x0000``
     - \-
     - Reserved.

.. _mini_v1_regmap_pllcfg:

PLLCFG Registers (``0x0020`` - ``0x003F``)
------------------------------------------

.. list-table:: PLLCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0020 <mini_v1_reg_0020>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0021 <mini_v1_reg_0021>`
     - ``0x0001``
     - ``auto_phcfg_err, auto_phcfg_done, busy, done``
     - PLL configuration status.
   * - :ref:`0x0022 <mini_v1_reg_0022>`
     - ``0x0000``
     - ``pll_lock``
     - RX/TX PLL lock status.
   * - :ref:`0x0023 <mini_v1_reg_0023>`
     - ``0x0000``
     - ``phcfg_mode, phcfg_updn, cnt_ind, pll_ind, pllrst_start, phcfg_start, pllcfg_start``
     - PLL control.
   * - :ref:`0x0024 <mini_v1_reg_0024>`
     - ``0x0000``
     - ``cnt_phase``
     - Counter phase value.
   * - :ref:`0x0025 <mini_v1_reg_0025>`
     - ``0x01F0``
     - ``pllcfg_bs, chp_curr, pllcfg_vcodiv, pllcfg_lf_res, pllcfg_lf_cap``
     - PLL reconfiguration settings.
   * - :ref:`0x0026 <mini_v1_reg_0026>`
     - ``0x0001``
     - ``m_odddiv, m_byp, n_odddiv, n_byp``
     - M/N counter bypass and odd-division control.
   * - :ref:`0x0027 <mini_v1_reg_0027>`
     - ``0x555A``
     - ``c0..c7_byp_odddiv``
     - Counter bypass and odd-division control bits for C0..C7.
   * - :ref:`0x0028 <mini_v1_reg_0028>`
     - ``0x5555``
     - ``c8..c15_byp_odddiv``
     - Counter bypass and odd-division control bits for C8..C15.
   * - :ref:`0x0029 <mini_v1_reg_0029>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x002A <mini_v1_reg_002a>`
     - ``0x0000``
     - ``n_hcnt, n_lcnt``
     - N counter values.
   * - :ref:`0x002B <mini_v1_reg_002b>`
     - ``0x0000``
     - ``m_hcnt, m_lcnt``
     - M counter values.
   * - :ref:`0x002C <mini_v1_reg_002c>`
     - ``0x0000``
     - ``m_frac_l``
     - M fractional counter values [15:0].
   * - :ref:`0x002D <mini_v1_reg_002d>`
     - ``0x0000``
     - ``m_frac_h``
     - M fractional counter values [31:16].
   * - :ref:`0x002E <mini_v1_reg_002e>`
     - ``0x0000``
     - ``c0_hcnt, c0_lcnt``
     - C0 counter values.
   * - :ref:`0x002F <mini_v1_reg_002f>`
     - ``0x0000``
     - ``c1_hcnt, c1_lcnt``
     - C1 counter values.
   * - :ref:`0x0030 <mini_v1_reg_0030>`
     - ``0x0000``
     - ``c2_hcnt, c2_lcnt``
     - C2 counter values.
   * - :ref:`0x0031 <mini_v1_reg_0031>`
     - ``0x0000``
     - ``c3_hcnt, c3_lcnt``
     - C3 counter values.
   * - :ref:`0x0032 <mini_v1_reg_0032>`
     - ``0x0000``
     - ``c4_hcnt, c4_lcnt``
     - C4 counter values.
   * - :ref:`0x0033 <mini_v1_reg_0033>`
     - ``0x0000``
     - ``c5_hcnt, c5_lcnt``
     - C5 counter values.
   * - :ref:`0x0034 <mini_v1_reg_0034>`
     - ``0x0000``
     - ``c6_hcnt, c6_lcnt``
     - C6 counter values.
   * - :ref:`0x0035 <mini_v1_reg_0035>`
     - ``0x0000``
     - ``c7_hcnt, c7_lcnt``
     - C7 counter values.
   * - :ref:`0x0036 <mini_v1_reg_0036>`
     - ``0x0000``
     - ``c8_hcnt, c8_lcnt``
     - C8 counter values.
   * - :ref:`0x0037 <mini_v1_reg_0037>`
     - ``0x0000``
     - ``c9_hcnt, c9_lcnt``
     - C9 counter values.
   * - :ref:`0x0038 <mini_v1_reg_0038>`
     -  
     - \-
     - Reserved for C10..C15 counter values.
   * - :ref:`0x0039 <mini_v1_reg_0039>`
     -  
     - \-
     - Reserved for C10..C15 counter values.
   * - :ref:`0x003A <mini_v1_reg_003a>`
     -  
     - \-
     - Reserved for C10..C15 counter values.
   * - :ref:`0x003B <mini_v1_reg_003b>`
     -  
     - \-
     - Reserved for C10..C15 counter values.
   * - :ref:`0x003C <mini_v1_reg_003c>`
     -  
     - \-
     - Reserved for C10..C15 counter values.
   * - :ref:`0x003D <mini_v1_reg_003d>`
     -  
     - \-
     - Reserved for C10..C15 counter values.
   * - :ref:`0x003E <mini_v1_reg_003e>`
     - ``0x0FFF``
     - ``auto_phcfg_smpls``
     - Samples to compare in auto phase-shift mode.
   * - :ref:`0x003F <mini_v1_reg_003f>`
     - ``0x0002``
     - ``auto_phcfg_step``
     - Step size for auto phase.

.. _mini_v1_regmap_tstcfg:

TSTCFG Registers (``0x0060`` - ``0x007F``)
------------------------------------------

.. list-table:: TSTCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0060 <mini_v1_reg_0060>`
     - ``0x00F0``
     - ``spi_sign_rezult, spi_sign``
     - SPI signature and test register.
   * - :ref:`0x0061 <mini_v1_reg_0061>`
     - ``0x0000``
     - ``test_en``
     - Test enable controls.
   * - :ref:`0x0062 <mini_v1_reg_0062>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0063 <mini_v1_reg_0063>`
     - ``0x0000``
     - ``test_frc_err``
     - Error insertion controls for tests.
   * - :ref:`0x0064 <mini_v1_reg_0064>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0065 <mini_v1_reg_0065>`
     - ``0x0000``
     - ``test_cmplt``
     - Test completion status bits.
   * - :ref:`0x0066 <mini_v1_reg_0066>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0067 <mini_v1_reg_0067>`
     - ``0x0000``
     - ``test_rez``
     - Test result bits.
   * - :ref:`0x0068 <mini_v1_reg_0068>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0069 <mini_v1_reg_0069>`
     -  
     - ``fx3_clk_cnt``
     - FX3 PCLK counter value.
   * - :ref:`0x006A <mini_v1_reg_006a>`
     -  
     - ``si5351c_clk0_cnt``
     - Si5351C CLK0 counter value.
   * - :ref:`0x006B <mini_v1_reg_006b>`
     -  
     - ``si5351c_clk1_cnt``
     - Si5351C CLK1 counter value.
   * - :ref:`0x006C <mini_v1_reg_006c>`
     -  
     - ``si5351c_clk2_cnt``
     - Si5351C CLK2 counter value.
   * - :ref:`0x006D <mini_v1_reg_006d>`
     -  
     - ``si5351c_clk3_cnt``
     - Si5351C CLK3 counter value.
   * - :ref:`0x006E <mini_v1_reg_006e>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x006F <mini_v1_reg_006f>`
     -  
     - ``si5351c_clk5_cnt``
     - Si5351C CLK5 counter value.
   * - :ref:`0x0070 <mini_v1_reg_0070>`
     -  
     - ``si5351c_clk6_cnt``
     - Si5351C CLK6 counter value.
   * - :ref:`0x0071 <mini_v1_reg_0071>`
     -  
     - ``si5351c_clk7_cnt``
     - Si5351C CLK7 counter value.
   * - :ref:`0x0072 <mini_v1_reg_0072>`
     -  
     - ``lmk_clk_cnt_l``
     - LMK clock counter low word.
   * - :ref:`0x0073 <mini_v1_reg_0073>`
     -  
     - ``lmk_clk_cnt_h``
     - LMK clock counter high word.
   * - :ref:`0x0074 <mini_v1_reg_0074>`
     -  
     - ``adf_cnt``
     - ADF transition count value.
   * - :ref:`0x0075 <mini_v1_reg_0075>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0076 <mini_v1_reg_0076>`
     -  
     - ``ddr2_1_tst_detail1``
     - DDR2_1 detailed test result flags.
   * - :ref:`0x0077 <mini_v1_reg_0077>`
     -  
     - ``ddr2_1_pnf_per_bit_l``
     - DDR2_1 data [15:0] pass/fail per bit.
   * - :ref:`0x0078 <mini_v1_reg_0078>`
     -  
     - ``ddr2_1_pnf_per_bit_h``
     - DDR2_1 data [31:16] pass/fail per bit.
   * - :ref:`0x0079 <mini_v1_reg_0079>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x007A <mini_v1_reg_007a>`
     -  
     - ``ddr2_2_tst_detail1``
     - DDR2_2 detailed test result flags.
   * - :ref:`0x007B <mini_v1_reg_007b>`
     -  
     - ``ddr2_2_pnf_per_bit_l``
     - DDR2_2 data [15:0] pass/fail per bit.
   * - :ref:`0x007C <mini_v1_reg_007c>`
     -  
     - ``ddr2_2_pnf_per_bit_h``
     - DDR2_2 data [31:16] pass/fail per bit.
   * - :ref:`0x007D <mini_v1_reg_007d>`
     - ``0xAAAA``
     - ``tx_tst_i``
     - TX test pattern I sample value.
   * - :ref:`0x007E <mini_v1_reg_007e>`
     - ``0x5555``
     - ``tx_tst_q``
     - TX test pattern Q sample value.
   * - :ref:`0x007F <mini_v1_reg_007f>`
     -  
     - \-
     - Reserved.

.. _mini_v1_regmap_periphcfg:

PERIPHCFG Registers (``0x00C0`` - ``0x00DF``)
---------------------------------------------

.. list-table:: PERIPHCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x00C0 <mini_v1_reg_00c0>`
     - ``0xFFFF``
     - ``board_gpio_ovrd``
     - Board GPIO override control.
   * - :ref:`0x00C1 <mini_v1_reg_00c1>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C2 <mini_v1_reg_00c2>`
     - ``0x0000``
     - ``board_gpio_rd``
     - Board GPIO read value.
   * - :ref:`0x00C3 <mini_v1_reg_00c3>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C4 <mini_v1_reg_00c4>`
     - ``0x0000``
     - ``board_gpio_dir``
     - Board GPIO direction control.
   * - :ref:`0x00C5 <mini_v1_reg_00c5>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C6 <mini_v1_reg_00c6>`
     - ``0x0000``
     - ``board_gpio_val``
     - Board GPIO output value control.
   * - :ref:`0x00C7 <mini_v1_reg_00c7>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C8 <mini_v1_reg_00c8>`
     - ``0x0000``
     - ``periph_input_rd_0``
     - Peripheral input readback 0 (not used).
   * - :ref:`0x00C9 <mini_v1_reg_00c9>`
     - ``0x0000``
     - ``periph_input_rd_1``
     - Peripheral input readback 1 (not used).
   * - :ref:`0x00CA <mini_v1_reg_00ca>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00CB <mini_v1_reg_00cb>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00CC <mini_v1_reg_00cc>`
     - ``0x0000``
     - ``periph_output_ovrd_0``
     - Peripheral output override 0 (fan).
   * - :ref:`0x00CD <mini_v1_reg_00cd>`
     - ``0x0000``
     - ``periph_output_val_0``
     - Peripheral output value 0 (fan).
   * - :ref:`0x00CE <mini_v1_reg_00ce>`
     - ``0x0000``
     - ``periph_output_ovrd_1``
     - Peripheral output override 1 (not used).
   * - :ref:`0x00CF <mini_v1_reg_00cf>`
     - ``0x0000``
     - ``periph_output_val_1``
     - Peripheral output value 1 (not used).
   * - :ref:`0x00D0 <mini_v1_reg_00d0>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D1 <mini_v1_reg_00d1>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D2 <mini_v1_reg_00d2>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D3 <mini_v1_reg_00d3>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D4 <mini_v1_reg_00d4>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D5 <mini_v1_reg_00d5>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D6 <mini_v1_reg_00d6>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D7 <mini_v1_reg_00d7>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D8 <mini_v1_reg_00d8>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D9 <mini_v1_reg_00d9>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00DA <mini_v1_reg_00da>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00DB <mini_v1_reg_00db>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00DC <mini_v1_reg_00dc>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00DD <mini_v1_reg_00dd>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00DE <mini_v1_reg_00de>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00DF <mini_v1_reg_00df>`
     -  
     - \-
     - Reserved.

Register Bitfield Reference
-------------------

Bit fields below are shown from MSB to LSB where applicable.

.. _mini_v1_reg_0000:

``0x0000`` - board_id
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0000`` | Default: ``0x0011`` | Access: R/W

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
     - Board identification number (LimeSDR Mini default 0x0011).

.. _mini_v1_reg_0001:

``0x0001`` - gw_ver
^^^^^^^^^^^^^^^^^^^

Address: ``0x0001`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "gw_ver [7:0]"},
     {"bits": 8, "name": "gw_ver [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``gw_ver``
     -  
     - Gateware version number.

.. _mini_v1_reg_0002:

``0x0002`` - gw_rev
^^^^^^^^^^^^^^^^^^^

Address: ``0x0002`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "gw_rev [7:0]"},
     {"bits": 8, "name": "gw_rev [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``gw_rev``
     -  
     - Gateware revision number.

.. _mini_v1_reg_0003:

``0x0003`` - hw_ver and bom_ver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0003`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "hw_ver [3:0]"},
     {"bits": 3, "name": "bom_ver [6:4]"},
     {"bits": 1, "name": "Reserved [7:7]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[6:4]``
     - ``bom_ver``
     -  
     - Bill of material version.
   * - ``[3:0]``
     - ``hw_ver``
     -  
     - Hardware version.

.. _mini_v1_reg_0004:

``0x0004`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0004`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0005:

``0x0005`` - drct_clk_en
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0005`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "drct_clk_en_tx"},
     {"bits": 1, "name": "drct_clk_en_rx"},
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
   * - ``[1]``
     - ``drct_clk_en_rx``
     - 0=PLL,1=Direct
     - RX clock source selection.
   * - ``[0]``
     - ``drct_clk_en_tx``
     - 0=PLL,1=Direct
     - TX clock source selection.

.. _mini_v1_reg_0006:

``0x0006`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0006`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0007:

``0x0007`` - ch_en
^^^^^^^^^^^^^^^^^^

Address: ``0x0007`` | Default: ``0x0303`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "ch_en_rx0"},
     {"bits": 1, "name": "ch_en_rx1"},
     {"bits": 6, "name": "Reserved [7:2]"},
     {"bits": 1, "name": "ch_en_tx0"},
     {"bits": 1, "name": "ch_en_tx1"},
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
     - ``ch_en_tx1``
     - 0=Disabled,1=Enabled
     - TX channel 1 enable.
   * - ``[8]``
     - ``ch_en_tx0``
     - 0=Disabled,1=Enabled
     - TX channel 0 enable.
   * - ``[1]``
     - ``ch_en_rx1``
     - 0=Disabled,1=Enabled
     - RX channel 1 enable.
   * - ``[0]``
     - ``ch_en_rx0``
     - 0=Disabled,1=Enabled
     - RX channel 0 enable.

.. _mini_v1_reg_0008:

``0x0008`` - diq_ctrl
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0008`` | Default: ``0x0102`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "smpl_width [1:0]"},
     {"bits": 3, "name": "Reserved [4:2]"},
     {"bits": 1, "name": "mode"},
     {"bits": 1, "name": "ddr_en"},
     {"bits": 1, "name": "triq_pulse"},
     {"bits": 1, "name": "mimo_int_en"},
     {"bits": 1, "name": "synch_dis"},
     {"bits": 1, "name": "dlb_en"},
     {"bits": 5, "name": "Reserved [15:11]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[10]``
     - ``dlb_en``
     - 0=Disabled,1=Enabled
     - Loopback enable (not used).
   * - ``[9]``
     - ``synch_dis``
     - 0=Enabled,1=Disabled
     - Packet synchronization using timestamps.
   * - ``[8]``
     - ``mimo_int_en``
     - 0=Disabled,1=Enabled
     - MIMO mode control.
   * - ``[7]``
     - ``triq_pulse``
     - 0=OFF,1=ON
     - TRXIQ pulse mode.
   * - ``[6]``
     - ``ddr_en``
     - 0=SDR,1=DDR
     - DIQ interface mode.
   * - ``[5]``
     - ``mode``
     - 0=TRXIQ,1=JESD207
     - Limelight port mode (JESD207 not implemented).
   * - ``[1:0]``
     - ``smpl_width``
     - 10=12-bit,00=16-bit
     - Interface sample width selection.

.. _mini_v1_reg_0009:

``0x0009`` - packet_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0009`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "smpl_nr_clr"},
     {"bits": 1, "name": "txpct_loss_clr"},
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
   * - ``[1]``
     - ``txpct_loss_clr``
     - 0=Normal,1=Rising edge clears
     - TX packet dropping flag clear.
   * - ``[0]``
     - ``smpl_nr_clr``
     - 0=Normal,1=Clear
     - Timestamp reset control.

.. _mini_v1_reg_000a:

``0x000A`` - rx_tx_ctrl
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "rx_en"},
     {"bits": 1, "name": "tx_en"},
     {"bits": 6, "name": "Reserved [7:2]"},
     {"bits": 1, "name": "rx_ptrn_en"},
     {"bits": 1, "name": "tx_ptrn_en"},
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
     - ``tx_ptrn_en``
     - 0=Disabled,1=Enabled
     - TX test pattern enable.
   * - ``[8]``
     - ``rx_ptrn_en``
     - 0=Disabled,1=Enabled
     - RX test pattern enable.
   * - ``[1]``
     - ``tx_en``
     - 0=Disabled,1=Enabled
     - TX chain enable.
   * - ``[0]``
     - ``rx_en``
     - 0=Disabled,1=Enabled
     - RX chain enable.

.. _mini_v1_reg_000b:

``0x000B`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000B`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_000c:

``0x000C`` - wfm_ch_en
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000C`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "wfm_ch0_en"},
     {"bits": 1, "name": "wfm_ch1_en"},
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
   * - ``[1]``
     - ``wfm_ch1_en``
     - 0=Disabled,1=Enabled
     - WFM channel 1 enable.
   * - ``[0]``
     - ``wfm_ch0_en``
     - 0=Disabled,1=Enabled
     - WFM channel 0 enable.

.. _mini_v1_reg_000d:

``0x000D`` - wfm_ctrl2
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000D`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "wfm_play"},
     {"bits": 1, "name": "wfm_load"},
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
     - ``wfm_load``
     - 0->1 transition
     - Starts WFM file loading.
   * - ``[1]``
     - ``wfm_play``
     - 0=Disabled,1=Enabled
     - WFM loaded-file play enable.

.. _mini_v1_reg_000e:

``0x000E`` - wfm_smpl_width
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000E`` | Default: ``0x0002`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "wfm_smpl_width [1:0]"},
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
     - ``wfm_smpl_width``
     - 10=12-bit,00=16-bit
     - WFM sample width selection.

.. _mini_v1_reg_000f:

``0x000F`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000F`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0010:

``0x0010`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0010`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0011:

``0x0011`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0011`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0012:

``0x0012`` - spi_ss
^^^^^^^^^^^^^^^^^^^

Address: ``0x0012`` | Default: ``0xFFFF`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "spi_ss [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``spi_ss``
     -  
     - Controlled SPI slave-select bits.

.. _mini_v1_reg_0013:

``0x0013`` - lms_misc
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0013`` | Default: ``0x6F6F`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "lms1_reset"},
     {"bits": 1, "name": "lms1_core_ldo_en"},
     {"bits": 1, "name": "lms1_txnrx1"},
     {"bits": 1, "name": "lms1_txnrx2"},
     {"bits": 1, "name": "lms1_txen"},
     {"bits": 1, "name": "lms1_rxen"},
     {"bits": 1, "name": "Reserved [7:7]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[6]``
     - ``lms1_rxen``
     - 0=Disabled,1=Enabled
     - LMS1 RX hard enable.
   * - ``[5]``
     - ``lms1_txen``
     - 0=Disabled,1=Enabled
     - LMS1 TX hard enable.
   * - ``[4]``
     - ``lms1_txnrx2``
     - 0=TXIQ,1=RXIQ
     - LMS1 port 2 mode.
   * - ``[3]``
     - ``lms1_txnrx1``
     - 0=TXIQ,1=RXIQ
     - LMS1 port 1 mode.
   * - ``[2]``
     - ``lms1_core_ldo_en``
     - 0=Disabled,1=Enabled
     - LMS1 internal LDO control.
   * - ``[1]``
     - ``lms1_reset``
     - 0=Reset active,1=Inactive
     - LMS1 hardware reset.

.. _mini_v1_reg_0014:

``0x0014`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0014`` | Default: ``0x0000`` | Access: R/W

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
     - Reserved for lms3_4.

.. _mini_v1_reg_0015:

``0x0015`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0015`` | Default: ``0x0000`` | Access: R/W

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
     - Reserved for lms5_6.

.. _mini_v1_reg_0016:

``0x0016`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0016`` | Default: ``0x0000`` | Access: R/W

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
     - Reserved for lms7_8.

.. _mini_v1_reg_0017:

``0x0017`` - gpio
^^^^^^^^^^^^^^^^^

Address: ``0x0017`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "gpio0"},
     {"bits": 1, "name": "gpio1"},
     {"bits": 1, "name": "gpio2"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "gpio4"},
     {"bits": 1, "name": "gpio5"},
     {"bits": 1, "name": "gpio6"},
     {"bits": 1, "name": "Reserved [7:7]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[6]``
     - ``gpio6``
     - 0=Disabled,1=Enabled
     - Channel B shunt.
   * - ``[5]``
     - ``gpio5``
     - 0=Disabled,1=Enabled
     - Channel B attenuator.
   * - ``[4]``
     - ``gpio4``
     - 0=Disabled,1=Enabled
     - RF loopback channel B.
   * - ``[2]``
     - ``gpio2``
     - 0=Disabled,1=Enabled
     - Channel A shunt.
   * - ``[1]``
     - ``gpio1``
     - 0=Disabled,1=Enabled
     - Channel A attenuator.
   * - ``[0]``
     - ``gpio0``
     - 0=Disabled,1=Enabled
     - RF loopback channel A.

.. _mini_v1_reg_0018:

``0x0018`` - dev_ctrl0
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0018`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "dev_ctrl0"},
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
     - ``dev_ctrl0``
     -  
     - Device control bit (not used).

.. _mini_v1_reg_0019:

``0x0019`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0019`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_001a:

``0x001A`` - fpga_led_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "fpga_led1_ovrd"},
     {"bits": 1, "name": "fpga_led1_r"},
     {"bits": 1, "name": "fpga_led1_g"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "fpga_led2_ovrd"},
     {"bits": 1, "name": "fpga_led2_r"},
     {"bits": 1, "name": "fpga_led2_g"},
     {"bits": 1, "name": "Reserved [7:7]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[6]``
     - ``fpga_led2_g``
     - 0=OFF,1=ON
     - Green LED2 control.
   * - ``[5]``
     - ``fpga_led2_r``
     - 0=OFF,1=ON
     - Red LED2 control.
   * - ``[4]``
     - ``fpga_led2_ovrd``
     - 0=OFF,1=ON
     - LED2 control override.
   * - ``[2]``
     - ``fpga_led1_g``
     - 0=OFF,1=ON
     - Green LED1 control.
   * - ``[1]``
     - ``fpga_led1_r``
     - 0=OFF,1=ON
     - Red LED1 control.
   * - ``[0]``
     - ``fpga_led1_ovrd``
     - 0=OFF,1=ON
     - LED1 control override.

.. _mini_v1_reg_001b:

``0x001B`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001B`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_001c:

``0x001C`` - fx3_led_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001C`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "fx3_led_ovrd"},
     {"bits": 1, "name": "fx3_led_r"},
     {"bits": 1, "name": "fx3_led_g"},
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
     - ``fx3_led_g``
     - 0=OFF,1=ON
     - Green FX3 LED control.
   * - ``[1]``
     - ``fx3_led_r``
     - 0=OFF,1=ON
     - Red FX3 LED control.
   * - ``[0]``
     - ``fx3_led_ovrd``
     - 0=OFF,1=ON
     - FX3 LED override.

.. _mini_v1_reg_001d:

``0x001D`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001D`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_001e:

``0x001E`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001E`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_001f:

``0x001F`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x001F`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0020:

``0x0020`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0020`` | Default: ``0x0000`` | Access: R/W

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

.. _mini_v1_reg_0021:

``0x0021`` - pll_status
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0021`` | Default: ``0x0001`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "done"},
     {"bits": 1, "name": "busy"},
     {"bits": 1, "name": "auto_phcfg_done"},
     {"bits": 1, "name": "auto_phcfg_err"},
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
     - ``auto_phcfg_err``
     - 0=No error,1=Error
     - Auto phase configuration error status.
   * - ``[2]``
     - ``auto_phcfg_done``
     - 0=Not done,1=Done
     - Auto phase configuration done status.
   * - ``[1]``
     - ``busy``
     - 0=Idle,1=Busy
     - PLL reconfiguration busy status.
   * - ``[0]``
     - ``done``
     - 0=Not done,1=Done
     - PLL configuration done status.

.. _mini_v1_reg_0022:

``0x0022`` - pll_lock
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0022`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "pll_lock_tx"},
     {"bits": 1, "name": "pll_lock_rx"},
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
   * - ``[1]``
     - ``pll_lock_rx``
     - 0=No lock,1=Locked
     - RX PLL lock.
   * - ``[0]``
     - ``pll_lock_tx``
     - 0=No lock,1=Locked
     - TX PLL lock.

.. _mini_v1_reg_0023:

``0x0023`` - pll_ctrl
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0023`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "pllcfg_start"},
     {"bits": 1, "name": "phcfg_start"},
     {"bits": 1, "name": "pllrst_start"},
     {"bits": 5, "name": "pll_ind [7:3]"},
     {"bits": 5, "name": "cnt_ind [12:8]"},
     {"bits": 1, "name": "phcfg_updn"},
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
     - 0=Manual,1=AUTO
     - PLL phase configuration mode.
   * - ``[13]``
     - ``phcfg_updn``
     - 0=Down,1=Up
     - Phase shift direction.
   * - ``[12:8]``
     - ``cnt_ind``
     - 0000=all,0001=M,0010=C0,0011=C1
     - Counter index for phase shift.
   * - ``[7:3]``
     - ``pll_ind``
     - 0000=TX PLL,0001=RX PLL
     - PLL index for reconfiguration.
   * - ``[2]``
     - ``pllrst_start``
     - 0->1 transition
     - PLL reset trigger.
   * - ``[1]``
     - ``phcfg_start``
     - 0->1 transition
     - Phase shift start trigger.
   * - ``[0]``
     - ``pllcfg_start``
     - 0->1 transition
     - PLL reconfiguration start trigger.

.. _mini_v1_reg_0024:

``0x0024`` - cnt_phase
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0024`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "cnt_phase [7:0]"},
     {"bits": 8, "name": "cnt_phase [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``cnt_phase``
     -  
     - Counter phase value.

.. _mini_v1_reg_0025:

``0x0025`` - pllcfg_settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0025`` | Default: ``0x01F0`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "pllcfg_lf_cap [1:0]"},
     {"bits": 5, "name": "pllcfg_lf_res [6:2]"},
     {"bits": 1, "name": "pllcfg_vcodiv"},
     {"bits": 3, "name": "chp_curr [10:8]"},
     {"bits": 4, "name": "pllcfg_bs [14:11]"},
     {"bits": 1, "name": "Reserved"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[14:11]``
     - ``pllcfg_bs``
     -  
     - Bandwidth setting (not used).
   * - ``[10:8]``
     - ``chp_curr``
     -  
     - PLL charge pump current.
   * - ``[7]``
     - ``pllcfg_vcodiv``
     - 0=2,1=1
     - PLL VCO division value.
   * - ``[6:2]``
     - ``pllcfg_lf_res``
     -  
     - PLL loop filter resistance.
   * - ``[1:0]``
     - ``pllcfg_lf_cap``
     -  
     - PLL loop filter capacitance.

.. _mini_v1_reg_0026:

``0x0026`` - m_odddiv, m_byp, n_odddiv, n_byp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0026`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "m_odddiv, m_byp, n_odddiv, n_byp [7:0]"},
     {"bits": 8, "name": "m_odddiv, m_byp, n_odddiv, n_byp [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``m_odddiv, m_byp, n_odddiv, n_byp``
     -  
     - M/N counter bypass and odd-division control.

.. _mini_v1_reg_0027:

``0x0027`` - c0..c7_byp_odddiv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0027`` | Default: ``0x555A`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c0..c7_byp_odddiv [7:0]"},
     {"bits": 8, "name": "c0..c7_byp_odddiv [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c0..c7_byp_odddiv``
     -  
     - Counter bypass and odd-division control bits for C0..C7.

.. _mini_v1_reg_0028:

``0x0028`` - c8..c15_byp_odddiv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0028`` | Default: ``0x5555`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c8..c15_byp_odddiv [7:0]"},
     {"bits": 8, "name": "c8..c15_byp_odddiv [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c8..c15_byp_odddiv``
     -  
     - Counter bypass and odd-division control bits for C8..C15.

.. _mini_v1_reg_0029:

``0x0029`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0029`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_002a:

``0x002A`` - n_cnt
^^^^^^^^^^^^^^^^^^

Address: ``0x002A`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "n_lcnt [7:0]"},
     {"bits": 8, "name": "n_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``n_hcnt``
     -  
     - N_CNT high counter bits.
   * - ``[7:0]``
     - ``n_lcnt``
     -  
     - N_CNT low counter bits.

.. _mini_v1_reg_002b:

``0x002B`` - m_cnt
^^^^^^^^^^^^^^^^^^

Address: ``0x002B`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "m_lcnt [7:0]"},
     {"bits": 8, "name": "m_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``m_hcnt``
     -  
     - M_CNT high counter bits.
   * - ``[7:0]``
     - ``m_lcnt``
     -  
     - M_CNT low counter bits.

.. _mini_v1_reg_002c:

``0x002C`` - m_frac_l
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x002C`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "m_frac_l [7:0]"},
     {"bits": 8, "name": "m_frac_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``m_frac_l``
     -  
     - M fractional counter values [15:0].

.. _mini_v1_reg_002d:

``0x002D`` - m_frac_h
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x002D`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "m_frac_h [7:0]"},
     {"bits": 8, "name": "m_frac_h [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``m_frac_h``
     -  
     - M fractional counter values [31:16].

.. _mini_v1_reg_002e:

``0x002E`` - c0_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x002E`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c0_lcnt [7:0]"},
     {"bits": 8, "name": "c0_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c0_hcnt``
     -  
     - C0_CNT high counter bits.
   * - ``[7:0]``
     - ``c0_lcnt``
     -  
     - C0_CNT low counter bits.

.. _mini_v1_reg_002f:

``0x002F`` - c1_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x002F`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c1_lcnt [7:0]"},
     {"bits": 8, "name": "c1_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c1_hcnt``
     -  
     - C1_CNT high counter bits.
   * - ``[7:0]``
     - ``c1_lcnt``
     -  
     - C1_CNT low counter bits.

.. _mini_v1_reg_0030:

``0x0030`` - c2_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0030`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c2_lcnt [7:0]"},
     {"bits": 8, "name": "c2_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c2_hcnt``
     -  
     - C2_CNT high counter bits.
   * - ``[7:0]``
     - ``c2_lcnt``
     -  
     - C2_CNT low counter bits.

.. _mini_v1_reg_0031:

``0x0031`` - c3_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0031`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c3_lcnt [7:0]"},
     {"bits": 8, "name": "c3_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c3_hcnt``
     -  
     - C3_CNT high counter bits.
   * - ``[7:0]``
     - ``c3_lcnt``
     -  
     - C3_CNT low counter bits.

.. _mini_v1_reg_0032:

``0x0032`` - c4_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0032`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c4_lcnt [7:0]"},
     {"bits": 8, "name": "c4_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c4_hcnt``
     -  
     - C4_CNT high counter bits.
   * - ``[7:0]``
     - ``c4_lcnt``
     -  
     - C4_CNT low counter bits.

.. _mini_v1_reg_0033:

``0x0033`` - c5_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0033`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c5_lcnt [7:0]"},
     {"bits": 8, "name": "c5_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c5_hcnt``
     -  
     - C5_CNT high counter bits.
   * - ``[7:0]``
     - ``c5_lcnt``
     -  
     - C5_CNT low counter bits.

.. _mini_v1_reg_0034:

``0x0034`` - c6_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0034`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c6_lcnt [7:0]"},
     {"bits": 8, "name": "c6_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c6_hcnt``
     -  
     - C6_CNT high counter bits.
   * - ``[7:0]``
     - ``c6_lcnt``
     -  
     - C6_CNT low counter bits.

.. _mini_v1_reg_0035:

``0x0035`` - c7_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0035`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c7_lcnt [7:0]"},
     {"bits": 8, "name": "c7_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c7_hcnt``
     -  
     - C7_CNT high counter bits.
   * - ``[7:0]``
     - ``c7_lcnt``
     -  
     - C7_CNT low counter bits.

.. _mini_v1_reg_0036:

``0x0036`` - c8_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0036`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c8_lcnt [7:0]"},
     {"bits": 8, "name": "c8_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c8_hcnt``
     -  
     - C8_CNT high counter bits.
   * - ``[7:0]``
     - ``c8_lcnt``
     -  
     - C8_CNT low counter bits.

.. _mini_v1_reg_0037:

``0x0037`` - c9_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0037`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c9_lcnt [7:0]"},
     {"bits": 8, "name": "c9_hcnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:8]``
     - ``c9_hcnt``
     -  
     - C9_CNT high counter bits.
   * - ``[7:0]``
     - ``c9_lcnt``
     -  
     - C9_CNT low counter bits.

.. _mini_v1_reg_0038:

``0x0038`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0038`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for C10..C15 counter values.

.. _mini_v1_reg_0039:

``0x0039`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0039`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for C10..C15 counter values.

.. _mini_v1_reg_003a:

``0x003A`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003A`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for C10..C15 counter values.

.. _mini_v1_reg_003b:

``0x003B`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003B`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for C10..C15 counter values.

.. _mini_v1_reg_003c:

``0x003C`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003C`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for C10..C15 counter values.

.. _mini_v1_reg_003d:

``0x003D`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003D`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for C10..C15 counter values.

.. _mini_v1_reg_003e:

``0x003E`` - auto_phcfg_smpls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003E`` | Default: ``0x0FFF`` | Access: R/W

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
     - Samples to compare in auto phase-shift mode.

.. _mini_v1_reg_003f:

``0x003F`` - auto_phcfg_step
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003F`` | Default: ``0x0002`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "auto_phcfg_step [7:0]"},
     {"bits": 8, "name": "auto_phcfg_step [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``auto_phcfg_step``
     -  
     - Step size for auto phase.

.. _mini_v1_reg_0060:

``0x0060`` - spi_sign
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0060`` | Default: ``0x00F0`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 4, "name": "spi_sign [3:0]"},
     {"bits": 4, "name": "spi_sign_rezult [7:4]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:4]``
     - ``spi_sign_rezult``
     -  
     - Inverted bits from SPI_SIGN register.
   * - ``[3:0]``
     - ``spi_sign``
     -  
     - SPI module test register.

.. _mini_v1_reg_0061:

``0x0061`` - test_en
^^^^^^^^^^^^^^^^^^^^

Address: ``0x0061`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "fx3_pclk_tst_en"},
     {"bits": 1, "name": "si5351c_tst_en"},
     {"bits": 1, "name": "vctcxo_tst_en"},
     {"bits": 1, "name": "adf_tst_en"},
     {"bits": 1, "name": "ddr2_1_tst_en"},
     {"bits": 1, "name": "ddr2_2_tst_en"},
     {"bits": 2, "name": "Reserved [7:6]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[5]``
     - ``ddr2_2_tst_en``
     - 0=Disabled,1=Enabled
     - DDR2_2 memory test enable.
   * - ``[4]``
     - ``ddr2_1_tst_en``
     - 0=Disabled,1=Enabled
     - DDR2_1 memory test enable.
   * - ``[3]``
     - ``adf_tst_en``
     - 0=Disabled,1=Enabled
     - Phase detector test enable.
   * - ``[2]``
     - ``vctcxo_tst_en``
     - 0=Disabled,1=Enabled
     - VCTCXO test enable.
   * - ``[1]``
     - ``si5351c_tst_en``
     - 0=Disabled,1=Enabled
     - Si5351C clock test enable.
   * - ``[0]``
     - ``fx3_pclk_tst_en``
     - 0=Disabled,1=Enabled
     - FX3 PCLK test enable.

.. _mini_v1_reg_0062:

``0x0062`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0062`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_0063:

``0x0063`` - test_force_err
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0063`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "fx3_pclk_tst_frc_err"},
     {"bits": 1, "name": "si5351c_tst_frc_err"},
     {"bits": 1, "name": "vctcxo_tst_frc_err"},
     {"bits": 1, "name": "adf_tst_frc_err"},
     {"bits": 1, "name": "ddr2_1_tst_frc_err"},
     {"bits": 1, "name": "ddr2_2_tst_frc_err"},
     {"bits": 2, "name": "Reserved [7:6]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[5]``
     - ``ddr2_2_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error to DDR2_2 memory test.
   * - ``[4]``
     - ``ddr2_1_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error to DDR2_1 memory test.
   * - ``[3]``
     - ``adf_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error to phase detector test.
   * - ``[2]``
     - ``vctcxo_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error to VCTCXO test.
   * - ``[1]``
     - ``si5351c_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error to Si5351C test.
   * - ``[0]``
     - ``fx3_pclk_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error to FX3 PCLK test.

.. _mini_v1_reg_0064:

``0x0064`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0064`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_0065:

``0x0065`` - test_cmplt
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0065`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "fx3_pclk_tst_cmplt"},
     {"bits": 1, "name": "si5351c_tst_cmplt"},
     {"bits": 1, "name": "vctcxo_tst_cmplt"},
     {"bits": 1, "name": "adf_tst_cmplt"},
     {"bits": 1, "name": "ddr2_1_tst_cmplt"},
     {"bits": 1, "name": "ddr2_2_tst_cmplt"},
     {"bits": 2, "name": "Reserved [7:6]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[5]``
     - ``ddr2_2_tst_cmplt``
     - 0=Not completed,1=Completed
     - DDR2_2 test complete status.
   * - ``[4]``
     - ``ddr2_1_tst_cmplt``
     - 0=Not completed,1=Completed
     - DDR2_1 test complete status.
   * - ``[3]``
     - ``adf_tst_cmplt``
     - 0=Not completed,1=Completed
     - Phase detector test complete status.
   * - ``[2]``
     - ``vctcxo_tst_cmplt``
     - 0=Not completed,1=Completed
     - VCTCXO test complete status.
   * - ``[1]``
     - ``si5351c_tst_cmplt``
     - 0=Not completed,1=Completed
     - Si5351C test complete status.
   * - ``[0]``
     - ``fx3_pclk_tst_cmplt``
     - 0=Not completed,1=Completed
     - FX3 PCLK test complete status.

.. _mini_v1_reg_0066:

``0x0066`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0066`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_0067:

``0x0067`` - test_rez
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0067`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "fx3_pclk_tst_rez"},
     {"bits": 1, "name": "si5351c_tst_rez"},
     {"bits": 1, "name": "vctcxo_tst_rez"},
     {"bits": 1, "name": "adf_tst_rez"},
     {"bits": 1, "name": "ddr2_1_tst_rez"},
     {"bits": 1, "name": "ddr2_2_tst_rez"},
     {"bits": 2, "name": "Reserved [7:6]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[5]``
     - ``ddr2_2_tst_rez``
     -  
     - DDR2_2 test result.
   * - ``[4]``
     - ``ddr2_1_tst_rez``
     -  
     - DDR2_1 test result.
   * - ``[3]``
     - ``adf_tst_rez``
     -  
     - Not used.
   * - ``[2]``
     - ``vctcxo_tst_rez``
     -  
     - Not used.
   * - ``[1]``
     - ``si5351c_tst_rez``
     -  
     - Not used.
   * - ``[0]``
     - ``fx3_pclk_tst_rez``
     -  
     - Not used.

.. _mini_v1_reg_0068:

``0x0068`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0068`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_0069:

``0x0069`` - fx3_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0069`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "fx3_clk_cnt [7:0]"},
     {"bits": 8, "name": "fx3_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``fx3_clk_cnt``
     -  
     - FX3 PCLK counter value.

.. _mini_v1_reg_006a:

``0x006A`` - si5351c_clk0_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006A`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk0_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk0_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk0_cnt``
     -  
     - Si5351C CLK0 counter value.

.. _mini_v1_reg_006b:

``0x006B`` - si5351c_clk1_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006B`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk1_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk1_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk1_cnt``
     -  
     - Si5351C CLK1 counter value.

.. _mini_v1_reg_006c:

``0x006C`` - si5351c_clk2_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006C`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk2_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk2_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk2_cnt``
     -  
     - Si5351C CLK2 counter value.

.. _mini_v1_reg_006d:

``0x006D`` - si5351c_clk3_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006D`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk3_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk3_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk3_cnt``
     -  
     - Si5351C CLK3 counter value.

.. _mini_v1_reg_006e:

``0x006E`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006E`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_006f:

``0x006F`` - si5351c_clk5_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006F`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk5_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk5_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk5_cnt``
     -  
     - Si5351C CLK5 counter value.

.. _mini_v1_reg_0070:

``0x0070`` - si5351c_clk6_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0070`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk6_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk6_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk6_cnt``
     -  
     - Si5351C CLK6 counter value.

.. _mini_v1_reg_0071:

``0x0071`` - si5351c_clk7_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0071`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk7_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk7_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``si5351c_clk7_cnt``
     -  
     - Si5351C CLK7 counter value.

.. _mini_v1_reg_0072:

``0x0072`` - lmk_clk_cnt_l
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0072`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lmk_clk_cnt_l [7:0]"},
     {"bits": 8, "name": "lmk_clk_cnt_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``lmk_clk_cnt_l``
     -  
     - LMK clock counter low word.

.. _mini_v1_reg_0073:

``0x0073`` - lmk_clk_cnt_h
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0073`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lmk_clk_cnt_h [7:0]"},
     {"bits": 8, "name": "lmk_clk_cnt_h [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``lmk_clk_cnt_h``
     -  
     - LMK clock counter high word.

.. _mini_v1_reg_0074:

``0x0074`` - adf_cnt
^^^^^^^^^^^^^^^^^^^^

Address: ``0x0074`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "adf_cnt [7:0]"},
     {"bits": 8, "name": "adf_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``adf_cnt``
     -  
     - ADF transition count value.

.. _mini_v1_reg_0075:

``0x0075`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0075`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_0076:

``0x0076`` - ddr2_1_tst_detail1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0076`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "ddr2_1_tst_cmplt"},
     {"bits": 1, "name": "ddr2_1_tst_pass"},
     {"bits": 1, "name": "ddr2_1_tst_fail"},
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
     - ``ddr2_1_tst_fail``
     - 0=Not completed,1=Fail
     - DDR2_1 fail flag.
   * - ``[1]``
     - ``ddr2_1_tst_pass``
     - 0=Not completed,1=Pass
     - DDR2_1 pass flag.
   * - ``[0]``
     - ``ddr2_1_tst_cmplt``
     - 0=Not completed,1=Complete
     - DDR2_1 complete flag.

.. _mini_v1_reg_0077:

``0x0077`` - ddr2_1_pnf_per_bit_l
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0077`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_l [7:0]"},
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``ddr2_1_pnf_per_bit_l``
     -  
     - DDR2_1 data [15:0] pass/fail per bit.

.. _mini_v1_reg_0078:

``0x0078`` - ddr2_1_pnf_per_bit_h
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0078`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_h [7:0]"},
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_h [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``ddr2_1_pnf_per_bit_h``
     -  
     - DDR2_1 data [31:16] pass/fail per bit.

.. _mini_v1_reg_0079:

``0x0079`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0079`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_007a:

``0x007A`` - ddr2_2_tst_detail1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007A`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "ddr2_2_tst_cmplt"},
     {"bits": 1, "name": "ddr2_2_tst_pass"},
     {"bits": 1, "name": "ddr2_2_tst_fail"},
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
     - ``ddr2_2_tst_fail``
     - 0=Not completed,1=Fail
     - DDR2_2 fail flag.
   * - ``[1]``
     - ``ddr2_2_tst_pass``
     - 0=Not completed,1=Pass
     - DDR2_2 pass flag.
   * - ``[0]``
     - ``ddr2_2_tst_cmplt``
     - 0=Not completed,1=Complete
     - DDR2_2 complete flag.

.. _mini_v1_reg_007b:

``0x007B`` - ddr2_2_pnf_per_bit_l
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007B`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_l [7:0]"},
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``ddr2_2_pnf_per_bit_l``
     -  
     - DDR2_2 data [15:0] pass/fail per bit.

.. _mini_v1_reg_007c:

``0x007C`` - ddr2_2_pnf_per_bit_h
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007C`` | Default: ``not specified`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_h [7:0]"},
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_h [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``ddr2_2_pnf_per_bit_h``
     -  
     - DDR2_2 data [31:16] pass/fail per bit.

.. _mini_v1_reg_007d:

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
     - TX test pattern I sample value.

.. _mini_v1_reg_007e:

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
     - TX test pattern Q sample value.

.. _mini_v1_reg_007f:

``0x007F`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007F`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00c0:

``0x00C0`` - board_gpio_ovrd
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C0`` | Default: ``0xFFFF`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_ovrd [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``board_gpio_ovrd``
     - 0=Dedicated function,1=Overridden by user
     - GPIO override bits.

.. _mini_v1_reg_00c1:

``0x00C1`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C1`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for GPIO.

.. _mini_v1_reg_00c2:

``0x00C2`` - board_gpio_rd
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C2`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_rd [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``board_gpio_rd``
     - 0=Low,1=High
     - GPIO readback bits.

.. _mini_v1_reg_00c3:

``0x00C3`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C3`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for GPIO.

.. _mini_v1_reg_00c4:

``0x00C4`` - board_gpio_dir
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C4`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_dir [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``board_gpio_dir``
     - 0=Input,1=Output
     - GPIO direction bits.

.. _mini_v1_reg_00c5:

``0x00C5`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C5`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for GPIO.

.. _mini_v1_reg_00c6:

``0x00C6`` - board_gpio_val
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C6`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_val [7:0]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7:0]``
     - ``board_gpio_val``
     - 0=Low,1=High
     - GPIO output value bits.

.. _mini_v1_reg_00c7:

``0x00C7`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C7`` | Default: ``not specified`` | Access: R/W

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
     - Reserved for GPIO.

.. _mini_v1_reg_00c8:

``0x00C8`` - periph_input_rd_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C8`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "periph_input_rd_0 [7:0]"},
     {"bits": 8, "name": "periph_input_rd_0 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``periph_input_rd_0``
     -  
     - Peripheral input readback 0 (not used).

.. _mini_v1_reg_00c9:

``0x00C9`` - periph_input_rd_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C9`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "periph_input_rd_1 [7:0]"},
     {"bits": 8, "name": "periph_input_rd_1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``periph_input_rd_1``
     -  
     - Peripheral input readback 1 (not used).

.. _mini_v1_reg_00ca:

``0x00CA`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CA`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00cb:

``0x00CB`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CB`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00cc:

``0x00CC`` - periph_output_ovrd_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CC`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "periph_output_ovrd_0"},
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
     - ``periph_output_ovrd_0``
     - 0=Dedicated,1=User controlled
     - Fan control override.

.. _mini_v1_reg_00cd:

``0x00CD`` - periph_output_val_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CD`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "periph_output_val_0"},
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
     - ``periph_output_val_0``
     - 0=OFF,1=ON
     - Fan control pin value.

.. _mini_v1_reg_00ce:

``0x00CE`` - periph_output_ovrd_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CE`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "periph_output_ovrd_1 [7:0]"},
     {"bits": 8, "name": "periph_output_ovrd_1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``periph_output_ovrd_1``
     -  
     - Peripheral output override 1 (not used).

.. _mini_v1_reg_00cf:

``0x00CF`` - periph_output_val_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CF`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "periph_output_val_1 [7:0]"},
     {"bits": 8, "name": "periph_output_val_1 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 900 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``periph_output_val_1``
     -  
     - Peripheral output value 1 (not used).

.. _mini_v1_reg_00d0:

``0x00D0`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D0`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d1:

``0x00D1`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D1`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d2:

``0x00D2`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D2`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d3:

``0x00D3`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D3`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d4:

``0x00D4`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D4`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d5:

``0x00D5`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D5`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d6:

``0x00D6`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D6`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d7:

``0x00D7`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D7`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d8:

``0x00D8`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D8`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00d9:

``0x00D9`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00D9`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00da:

``0x00DA`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00DA`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00db:

``0x00DB`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00DB`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00dc:

``0x00DC`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00DC`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00dd:

``0x00DD`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00DD`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00de:

``0x00DE`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00DE`` | Default: ``not specified`` | Access: R/W

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

.. _mini_v1_reg_00df:

``0x00DF`` - reserved
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00DF`` | Default: ``not specified`` | Access: R/W

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
