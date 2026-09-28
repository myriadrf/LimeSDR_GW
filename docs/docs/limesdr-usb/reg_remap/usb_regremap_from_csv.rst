LimeSDR USB Host Register Reference
===================================

This page is generated from CSV sources.

Quick Navigation
----------------

.. list-table:: Module overview
   :header-rows: 1
   :widths: 20 16 64

   * - Module
     - Address range
     - Typical use
   * - :ref:`FPGACFG <usb_regmap_fpgacfg>`
     - ``0x0000`` - ``0x001F``
     - Board ID, gateware revision, stream/interface control, LED and LMS enable/reset
   * - :ref:`PLLCFG <usb_regmap_pllcfg>`
     - ``0x0020`` - ``0x003F``
     - PLL status/control, divider and counter configuration
   * - :ref:`TSTCFG <usb_regmap_tstcfg>`
     - ``0x0060`` - ``0x007F``
     - Built-in test control/status, clock counters, and DDR2 self-test
   * - :ref:`PERIPHCFG <usb_regmap_periphcfg>`
     - ``0x00C0`` - ``0x00D3``
     - GPIO override/readback and fan control

.. _usb_regmap_fpgacfg:

FPGACFG Registers (``0x0000`` - ``0x001F``)
-------------------------------------------

.. list-table:: FPGACFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0000 <usb_reg_0000>`
     - ``0x0011``
     - ``board_id``
     - Board identification number (LimeSDR USB default 0x0011).
   * - :ref:`0x0001 <usb_reg_0001>`
     -  
     - ``major_rev``
     - Major gateware revision number.
   * - :ref:`0x0002 <usb_reg_0002>`
     -  
     - ``compile_rev``
     - Gateware compile revision number.
   * - :ref:`0x0003 <usb_reg_0003>`
     - ``0x0002``
     - ``board_ver_ctrl``
     - Board version/BOM control. Documented quirk: firmware currently always returns a fixed 0x0002, regardless of actual hardware/BOM version.
   * - :ref:`0x0004 <usb_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0005 <usb_reg_0005>`
     - ``0x0000``
     - ``drct_clk_en``
     - Clock source selection for RX/TX interfaces (direct vs. PLL clocking).
   * - :ref:`0x0006 <usb_regmap_fpgacfg>`
     - ``0x0000``
     - \-
     - Reserved.
   * - :ref:`0x0007 <usb_reg_0007>`
     - ``0x0303``
     - ``ch_en``
     - RX/TX MIMO channel enable control.
   * - :ref:`0x0008 <usb_reg_0008>`
     - ``0x0102``
     - ``stream_ctrl``
     - Sample-width and interface-mode control. Only smpl_width[1:0], trxiq_pulse[7], mimo_int_en[8] and synch_dis[9] are writable; other bits are read-only pass-through.
   * - :ref:`0x0009 <usb_reg_0009>`
     - ``0x0003``
     - ``reg09``
     - Packet control: TX packet-loss clear and timestamp reset.
   * - :ref:`0x000A <usb_reg_000a>`
     - ``0x0000``
     - ``reg10``
     - RX/TX module control.
   * - :ref:`0x000B <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x000C <usb_reg_000c>`
     - ``0x0003``
     - ``wfm_ch_en``
     - WFM player channel enable control (DDR2/WFM-capable variants only, guarded by DDR_MODULES_PRESENT).
   * - :ref:`0x000D <usb_reg_000d>`
     - ``0x0001``
     - ``wfm_ctrl2``
     - WFM player sample-width/play/load control (DDR2/WFM-capable variants only, guarded by DDR_MODULES_PRESENT).
   * - :ref:`0x000E <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x000F <usb_reg_000f>`
     - ``0x0000``
     - ``txant_pre``
     - Number of samples to delay turning on the internal TDD signal.
   * - :ref:`0x0010 <usb_reg_0010>`
     - ``0x0000``
     - ``txant_post``
     - Number of samples to delay turning off the internal TDD signal.
   * - :ref:`0x0011 <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0012 <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0013 <usb_reg_0013>`
     - ``0x0000``
     - ``lms1``
     - LMS7002 hard-enable/reset and digital-interface control. Write-only in the current firmware (no read-back case is implemented, so readback always returns 0).
   * - :ref:`0x0014 <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved for lms3_4.
   * - :ref:`0x0015 <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved for lms5_6.
   * - :ref:`0x0016 <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved for lms7_8.
   * - :ref:`0x0017 <usb_reg_0017>`
     - ``0x0000``
     - ``lb_out_override``
     - RF-loopback override value for the onboard TX1/TX2 loopback switches. Board-specific: writing this register also force-enables the override-enable bits for all loopback outputs (0xFF).
   * - :ref:`0x0018 <usb_reg_0018>`
     - ``0x0000``
     - ``reg18``
     - Board-specific control register (documented as reg18; not further decoded by firmware).
   * - :ref:`0x0019 <usb_reg_0019>`
     - ``0x1000``
     - ``rx_packet_size``
     - RX packet size in bytes.
   * - :ref:`0x001A <usb_reg_001a>`
     - ``0x0000``
     - ``fpga_led_ctrl``
     - Onboard FPGA LED1/LED2 override/control. The single legacy OVRD bit for each LED is fanned out to the independent green- and red-pin override-enable bits.
   * - :ref:`0x001B <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x001C <usb_reg_001c>`
     - ``0x0000``
     - ``fx3_led_ctrl``
     - Onboard FX3 activity LED override/control. The single legacy OVRD bit is fanned out to the independent green- and red-pin override-enable bits.
   * - :ref:`0x001D <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x001E <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x001F <usb_regmap_fpgacfg>`
     -  
     - \-
     - Reserved.

.. _usb_regmap_pllcfg:

PLLCFG Registers (``0x0020`` - ``0x003F``)
------------------------------------------

.. list-table:: PLLCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0020 <usb_reg_0020>`
     - ``0x0000``
     - ``c1_phase``
     - Phase value for PLL output clock 1.
   * - :ref:`0x0021 <usb_reg_0021>`
     - ``0x0001``
     - ``pll_status``
     - PLL and phase-configuration status. Board-specific: adds a pllcfg_error status bit at bit 7, in addition to the common pllcfg_done/pllcfg_busy/phcfg_done/phcfg_err bits.
   * - :ref:`0x0022 <usb_reg_0022>`
     - ``0x0000``
     - ``pll_lock``
     - PLL lock status (single PLL on this board).
   * - :ref:`0x0023 <usb_reg_0023>`
     - ``0x0000``
     - ``pll_ctrl``
     - PLL reconfiguration control: start triggers, PLL/counter index selection, and phase-shift direction/mode.
   * - :ref:`0x0024 <usb_reg_0024>`
     - ``0x0000``
     - ``cnt_phase``
     - Counter phase value.
   * - :ref:`0x0025 <usb_reg_0025>`
     - ``0x0000``
     - ``pllcfg_vcodiv``
     - PLL VCO division control. Board-specific: only bit 7 (pllcfg_vcodiv) is wired; the loop-filter/charge-pump bits used on other boards are not implemented here.
   * - :ref:`0x0026 <usb_reg_0026>`
     - ``0x0000``
     - ``mn_bypass_ctrl``
     - M/N counter bypass and odd-division control.
   * - :ref:`0x0027 <usb_reg_0027>`
     - ``0x0000``
     - ``c0..c4_byp_odddiv``
     - Counter bypass and odd-division control bits for C0..C4. Board-specific: this board's PLL reconfiguration block has 5 output counters (C0..C4), unlike the 8-counter (C0..C7) block used on other boards.
   * - :ref:`0x0028 <usb_regmap_pllcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0029 <usb_regmap_pllcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x002A <usb_reg_002a>`
     - ``0x0000``
     - ``n_cnt``
     - N counter value.
   * - :ref:`0x002B <usb_reg_002b>`
     - ``0x0000``
     - ``m_cnt``
     - M counter value.
   * - :ref:`0x002C <usb_regmap_pllcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x002D <usb_regmap_pllcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x002E <usb_reg_002e>`
     - ``0x0000``
     - ``c0_cnt``
     - C0 counter value.
   * - :ref:`0x002F <usb_reg_002f>`
     - ``0x0000``
     - ``c1_cnt``
     - C1 counter value.
   * - :ref:`0x0030 <usb_reg_0030>`
     - ``0x0000``
     - ``c2_cnt``
     - C2 counter value.
   * - :ref:`0x0031 <usb_reg_0031>`
     - ``0x0000``
     - ``c3_cnt``
     - C3 counter value.
   * - :ref:`0x0032 <usb_reg_0032>`
     - ``0x0000``
     - ``c4_cnt``
     - C4 counter value.
   * - :ref:`0x0033 <usb_regmap_pllcfg>` - :ref:`0x003D <usb_regmap_pllcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x003E <usb_reg_003e>`
     - ``0x0FFF``
     - ``phcfg_samples``
     - Number of samples to compare during automatic phase configuration.
   * - :ref:`0x003F <usb_reg_003f>`
     - ``0x0002``
     - ``phcfg_step``
     - Step size used during automatic phase configuration.

.. _usb_regmap_tstcfg:

TSTCFG Registers (``0x0060`` - ``0x007F``)
------------------------------------------

.. list-table:: TSTCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x0060 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0061 <usb_reg_0061>`
     - ``0x0000``
     - ``test_en``
     - Test enable controls.
   * - :ref:`0x0062 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0063 <usb_reg_0063>`
     - ``0x0000``
     - ``test_frc_err``
     - Error insertion controls for tests.
   * - :ref:`0x0064 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0065 <usb_reg_0065>`
     - ``0x0000``
     - ``test_cmplt``
     - Test completion status bits.
   * - :ref:`0x0066 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0067 <usb_reg_0067>`
     - ``0x0000``
     - ``test_rez``
     - Test result bits.
   * - :ref:`0x0068 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0069 <usb_reg_0069>`
     -  
     - ``fx3_clk_cnt``
     - FX3 PCLK counter value.
   * - :ref:`0x006A <usb_reg_006a>`
     -  
     - ``si5351c_clk7_cnt``
     - Si5351C CLK7 counter value. Board-specific: this board's SI5351C counters are addressed in descending order (0x6A=CLK7 .. 0x71=CLK0), unlike the ascending order (0x6A=CLK0 .. 0x71=CLK7) used on other boards.
   * - :ref:`0x006B <usb_reg_006b>`
     -  
     - ``si5351c_clk6_cnt``
     - Si5351C CLK6 counter value.
   * - :ref:`0x006C <usb_reg_006c>`
     -  
     - ``si5351c_clk5_cnt``
     - Si5351C CLK5 counter value.
   * - :ref:`0x006D <usb_reg_006d>`
     -  
     - ``si5351c_clk3_cnt``
     - Si5351C CLK3 counter value.
   * - :ref:`0x006E <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved (no Si5351C CLK4 counter on this board).
   * - :ref:`0x006F <usb_reg_006f>`
     -  
     - ``si5351c_clk2_cnt``
     - Si5351C CLK2 counter value.
   * - :ref:`0x0070 <usb_reg_0070>`
     -  
     - ``si5351c_clk1_cnt``
     - Si5351C CLK1 counter value.
   * - :ref:`0x0071 <usb_reg_0071>`
     -  
     - ``si5351c_clk0_cnt``
     - Si5351C CLK0 counter value.
   * - :ref:`0x0072 <usb_reg_0072>`
     -  
     - ``lmk_clk_cnt_l``
     - LMK clock counter low word.
   * - :ref:`0x0073 <usb_reg_0073>`
     -  
     - ``lmk_clk_cnt_h``
     - LMK clock counter high word.
   * - :ref:`0x0074 <usb_reg_0074>`
     -  
     - ``adf_muxout_cnt``
     - ADF4002 MUXOUT transition count value.
   * - :ref:`0x0075 <usb_regmap_tstcfg>` - :ref:`0x0076 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x0077 <usb_reg_0077>`
     -  
     - ``ddr2_1_pnf_per_bit_l``
     - DDR2_1 data [15:0] pass/fail-per-bit result. Board-specific: LimeSDR-USB has WFM-player-driven DDR2 self-test registers not present on other boards.
   * - :ref:`0x0078 <usb_reg_0078>`
     -  
     - ``ddr2_1_pnf_per_bit_h``
     - DDR2_1 data [31:16] pass/fail-per-bit result.
   * - :ref:`0x0079 <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x007A <usb_reg_007a>`
     -  
     - ``ddr2_2_tst_detail1``
     - DDR2_2 test complete/pass/fail status bits.
   * - :ref:`0x007B <usb_reg_007b>`
     -  
     - ``ddr2_2_pnf_per_bit_l``
     - DDR2_2 data [15:0] pass/fail-per-bit result.
   * - :ref:`0x007C <usb_reg_007c>`
     -  
     - ``ddr2_2_pnf_per_bit_h``
     - DDR2_2 data [31:16] pass/fail-per-bit result.
   * - :ref:`0x007D <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved on this board (TX test pattern I register is not implemented in LimeSDR-USB firmware).
   * - :ref:`0x007E <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved on this board (TX test pattern Q register is not implemented in LimeSDR-USB firmware).
   * - :ref:`0x007F <usb_regmap_tstcfg>`
     -  
     - \-
     - Reserved.

.. _usb_regmap_periphcfg:

PERIPHCFG Registers (``0x00C0`` - ``0x00D3``)
---------------------------------------------

.. list-table:: PERIPHCFG registers
   :header-rows: 1
   :widths: 10 12 20 58

   * - Address
     - Default
     - Name
     - Description
   * - :ref:`0x00C0 <usb_reg_00c0>`
     - ``0x0000``
     - ``board_gpio_ovrd``
     - Board GPIO override control.
   * - :ref:`0x00C1 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C2 <usb_reg_00c2>`
     - ``0x0000``
     - ``board_gpio_rd``
     - Board GPIO read value.
   * - :ref:`0x00C3 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C4 <usb_reg_00c4>`
     - ``0x0000``
     - ``board_gpio_dir``
     - Board GPIO direction control. Legacy convention (1=output) is inverted from the underlying gpio_io CSR's convention (0=output) by the firmware.
   * - :ref:`0x00C5 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved for GPIO.
   * - :ref:`0x00C6 <usb_reg_00c6>`
     - ``0x0000``
     - ``board_gpio_val``
     - Board GPIO output value control.
   * - :ref:`0x00C7 <usb_regmap_periphcfg>` - :ref:`0x00C9 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00CA <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved for this board. PERIPH_INPUT_SEL is defined in the shared register layout but not wired in LimeSDR-USB firmware (commented out in regremap.c).
   * - :ref:`0x00CB <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00CC <usb_reg_00cc>`
     - ``0x0000``
     - ``periph_output_ovrd_0``
     - Peripheral output override 0 (fan control override enable).
   * - :ref:`0x00CD <usb_reg_00cd>`
     - ``0x0000``
     - ``periph_output_val_0``
     - Peripheral output value 0 (fan control value).
   * - :ref:`0x00CE <usb_regmap_periphcfg>` - :ref:`0x00D1 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved.
   * - :ref:`0x00D2 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved for this board. PERIPH_EN is defined in the shared register layout but not wired in LimeSDR-USB firmware (commented out in regremap.c).
   * - :ref:`0x00D3 <usb_regmap_periphcfg>`
     -  
     - \-
     - Reserved for this board. PERIPH_SEL is defined in the shared register layout but not wired in LimeSDR-USB firmware (commented out in regremap.c).

Register Bitfield Reference
-------------------

Bit fields below are shown from MSB to LSB where applicable.

.. _usb_reg_0000:

``0x0000`` - board_id
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0000`` | Default: ``0x0011`` | Access: R

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
     - Board identification number, read only (LimeSDR USB default 0x0011).

.. _usb_reg_0001:

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
     - Major gateware revision, read only.

.. _usb_reg_0002:

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
     - Gateware compile revision, read only.

.. _usb_reg_0003:

``0x0003`` - board_ver_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0003`` | Default: ``0x0002`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_ver_ctrl [7:0]"},
     {"bits": 8, "name": "board_ver_ctrl [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``board_ver_ctrl``
     -  
     - Board version/BOM control. Documented quirk: firmware currently always returns a fixed 0x0002, regardless of actual hardware/BOM version.

.. _usb_reg_0005:

``0x0005`` - drct_clk_en
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0005`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "drct_clk_en_tx"},
     {"bits": 1, "name": "drct_clk_en_rx"},
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
     - ``drct_clk_en_rx``
     - 0=PLL,1=Direct
     - RX clock source selection.
   * - ``[0]``
     - ``drct_clk_en_tx``
     - 0=PLL,1=Direct
     - TX clock source selection.

.. _usb_reg_0007:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_0008:

``0x0008`` - stream_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0008`` | Default: ``0x0102`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 2, "name": "smpl_width [1:0]"},
     {"bits": 5, "name": "Reserved [6:2]"},
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
     - 0=Enabled,1=Disabled
     - Packet synchronization using timestamps.
   * - ``[8]``
     - ``mimo_int_en``
     - 0=Disabled,1=Enabled
     - MIMO mode control.
   * - ``[7]``
     - ``trxiq_pulse``
     - 0=OFF,1=ON
     - TRXIQ pulse mode. Only bit exposed by firmware on both read and write; other reg08 bits are not accessible through this legacy register.
   * - ``[1:0]``
     - ``smpl_width``
     - 10=12-bit,00=16-bit
     - Interface sample width selection.

.. _usb_reg_0009:

``0x0009`` - reg09
^^^^^^^^^^^^^^^^^^

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
     - 0=Normal,1=Rising edge clears
     - TX packet dropping flag clear.
   * - ``[0]``
     - ``smpl_nr_clr``
     - 0=Normal,1=Clear
     - Timestamp reset control.

.. _usb_reg_000a:

``0x000A`` - reg10
^^^^^^^^^^^^^^^^^^

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
     - TX test pattern enable.
   * - ``[8]``
     - ``rx_ptrn_en``
     - 0/1
     - RX test pattern enable.
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
     - 0=TX2,1=TX1
     - TX RF switch select.
   * - ``[3:2]``
     - ``rx_rf_sw``
     - 00=RX_W,01=RX_L,10=RX_H,11=NC
     - RX RF switch select.
   * - ``[0]``
     - ``rx_en``
     - 0=Disabled,1=Enabled
     - Unified RX enable.

.. _usb_reg_000c:

``0x000C`` - wfm_ch_en
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000C`` | Default: ``0x0003`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "wfm_ch0_en"},
     {"bits": 1, "name": "wfm_ch1_en"},
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
     - ``wfm_ch1_en``
     - 0=Disabled,1=Enabled
     - WFM channel 1 enable.
   * - ``[0]``
     - ``wfm_ch0_en``
     - 0=Disabled,1=Enabled
     - WFM channel 0 enable.

.. _usb_reg_000d:

``0x000D`` - wfm_ctrl2
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000D`` | Default: ``0x0001`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "wfm_smpl_width"},
     {"bits": 1, "name": "wfm_play"},
     {"bits": 1, "name": "wfm_load"},
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
     - ``wfm_load``
     - 0->1 transition
     - Starts WFM file loading.
   * - ``[1]``
     - ``wfm_play``
     - 0=Disabled,1=Enabled
     - WFM loaded-file play enable.
   * - ``[0]``
     - ``wfm_smpl_width``
     - 0=16-bit,1=12-bit
     - WFM sample width selection.

.. _usb_reg_000f:

``0x000F`` - txant_pre
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x000F`` | Default: ``0x0000`` | Access: R/W

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

.. _usb_reg_0010:

``0x0010`` - txant_post
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0010`` | Default: ``0x0000`` | Access: R/W

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

.. _usb_reg_0013:

``0x0013`` - lms1
^^^^^^^^^^^^^^^^^

Address: ``0x0013`` | Default: ``0x0000`` | Access: W

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - LMS1 hardware reset. Board-specific: this register is write-only in the current firmware, no read-back case is implemented.

.. _usb_reg_0017:

``0x0017`` - lb_out_override
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0017`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "tx1_l_h"},
     {"bits": 1, "name": "tx1_at"},
     {"bits": 1, "name": "tx1_sh"},
     {"bits": 1, "name": "Reserved"},
     {"bits": 1, "name": "tx2_l_h"},
     {"bits": 1, "name": "tx2_at"},
     {"bits": 1, "name": "tx2_sh"},
     {"bits": 1, "name": "Reserved [7:7]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[6]``
     - ``tx2_sh``
     - 0=Disabled,1=Enabled
     - Channel TX2 loopback shunt enable.
   * - ``[5]``
     - ``tx2_at``
     - 0=Disabled,1=Enabled
     - Channel TX2 loopback attenuator enable.
   * - ``[4]``
     - ``tx2_l_h``
     - 0=Disabled,1=Enabled
     - RF loopback TX2 path enable (drives complementary TX2_L/TX2_H switch outputs).
   * - ``[2]``
     - ``tx1_sh``
     - 0=Disabled,1=Enabled
     - Channel TX1 loopback shunt enable.
   * - ``[1]``
     - ``tx1_at``
     - 0=Disabled,1=Enabled
     - Channel TX1 loopback attenuator enable.
   * - ``[0]``
     - ``tx1_l_h``
     - 0=Disabled,1=Enabled
     - RF loopback TX1 path enable (drives complementary TX1_L/TX1_H switch outputs). Board-specific: writing this register also force-enables the override for all loopback outputs.

.. _usb_reg_0018:

``0x0018`` - reg18
^^^^^^^^^^^^^^^^^^

Address: ``0x0018`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "reg18 [7:0]"},
     {"bits": 8, "name": "reg18 [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``reg18``
     -  
     - Board-specific control register; not further decoded by firmware.

.. _usb_reg_0019:

``0x0019`` - rx_packet_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0019`` | Default: ``0x1000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "rx_packet_size [7:0]"},
     {"bits": 8, "name": "rx_packet_size [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``rx_packet_size``
     -  
     - RX packet size in bytes.

.. _usb_reg_001a:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_001c:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - FX3 activity LED override.

.. _usb_reg_0020:

``0x0020`` - c1_phase
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0020`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c1_phase [7:0]"},
     {"bits": 1, "name": "c1_phase [8:8]"},
     {"bits": 7, "name": "Reserved [15:9]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[8:0]``
     - ``c1_phase``
     -  
     - Phase value for PLL output clock 1.

.. _usb_reg_0021:

``0x0021`` - pll_status
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0021`` | Default: ``0x0001`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "pllcfg_done"},
     {"bits": 1, "name": "pllcfg_busy"},
     {"bits": 1, "name": "phcfg_done"},
     {"bits": 1, "name": "phcfg_err"},
     {"bits": 3, "name": "Reserved [6:4]"},
     {"bits": 1, "name": "pllcfg_error"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7]``
     - ``pllcfg_error``
     - status
     - PLL configuration error, read only. Board-specific: added in addition to the common status bits.
   * - ``[3]``
     - ``phcfg_err``
     - status
     - Phase configuration error, read only.
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

.. _usb_reg_0022:

``0x0022`` - pll_lock
^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0022`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "pll_lock"},
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
     - ``pll_lock``
     - 0=No lock,1=Locked
     - PLL lock status (single PLL on this board).

.. _usb_reg_0023:

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
     - 0=Manual,1=AUTO
     - PLL phase configuration mode.
   * - ``[13]``
     - ``phcfg_updn``
     - 0=Down,1=Up
     - Phase shift direction.
   * - ``[12:8]``
     - ``cnt_ind``
     - 00000=all,00010=C0,00011=C1,...
     - Counter index for phase shift.
   * - ``[7:3]``
     - ``pll_ind``
     -  
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

.. _usb_reg_0024:

``0x0024`` - cnt_phase
^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0024`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "cnt_phase [7:0]"},
     {"bits": 8, "name": "cnt_phase [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_0025:

``0x0025`` - pllcfg_vcodiv
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0025`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 7, "name": "Reserved [6:0]"},
     {"bits": 1, "name": "pllcfg_vcodiv"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[7]``
     - ``pllcfg_vcodiv``
     - 0=2,1=1
     - PLL VCO division value. Board-specific: only this bit is wired; loop-filter/charge-pump bits used on other boards are not implemented.

.. _usb_reg_0026:

``0x0026`` - mn_bypass_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0026`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "n_div_byp"},
     {"bits": 1, "name": "n_odd_div"},
     {"bits": 1, "name": "m_div_byp"},
     {"bits": 1, "name": "m_odd_div"},
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
     - ``m_odd_div``
     - 0=even,1=odd
     - M counter odd-division enable.
   * - ``[2]``
     - ``m_div_byp``
     - 0=normal,1=bypass
     - M counter bypass control.
   * - ``[1]``
     - ``n_odd_div``
     - 0=even,1=odd
     - N counter odd-division enable.
   * - ``[0]``
     - ``n_div_byp``
     - 0=normal,1=bypass
     - N counter bypass control.

.. _usb_reg_0027:

``0x0027`` - c0..c4_byp_odddiv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0027`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "c0_div_byp"},
     {"bits": 1, "name": "c0_odddiv"},
     {"bits": 1, "name": "c1_div_byp"},
     {"bits": 1, "name": "c1_odddiv"},
     {"bits": 1, "name": "c2_div_byp"},
     {"bits": 1, "name": "c2_odddiv"},
     {"bits": 1, "name": "c3_div_byp"},
     {"bits": 1, "name": "c3_odddiv"},
     {"bits": 1, "name": "c4_div_byp"},
     {"bits": 1, "name": "c4_odddiv"},
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
     - ``c4_odddiv``
     - 0=even,1=odd
     - C4 counter odd-division enable. Board-specific: this board's PLL reconfiguration block has 5 output counters (C0..C4), unlike the 8-counter block used on other boards.
   * - ``[8]``
     - ``c4_div_byp``
     - 0=normal,1=bypass
     - C4 counter bypass control.
   * - ``[7]``
     - ``c3_odddiv``
     - 0=even,1=odd
     - C3 counter odd-division enable.
   * - ``[6]``
     - ``c3_div_byp``
     - 0=normal,1=bypass
     - C3 counter bypass control.
   * - ``[5]``
     - ``c2_odddiv``
     - 0=even,1=odd
     - C2 counter odd-division enable.
   * - ``[4]``
     - ``c2_div_byp``
     - 0=normal,1=bypass
     - C2 counter bypass control.
   * - ``[3]``
     - ``c1_odddiv``
     - 0=even,1=odd
     - C1 counter odd-division enable.
   * - ``[2]``
     - ``c1_div_byp``
     - 0=normal,1=bypass
     - C1 counter bypass control.
   * - ``[1]``
     - ``c0_odddiv``
     - 0=even,1=odd
     - C0 counter odd-division enable.
   * - ``[0]``
     - ``c0_div_byp``
     - 0=normal,1=bypass
     - C0 counter bypass control.

.. _usb_reg_002a:

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
     - N counter (PLL divider) value.

.. _usb_reg_002b:

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
     - M counter (PLL multiplier) value.

.. _usb_reg_002e:

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
     - C0 counter (PLL output 0 divider) value.

.. _usb_reg_002f:

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
     - C1 counter (PLL output 1 divider) value.

.. _usb_reg_0030:

``0x0030`` - c2_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0030`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c2_cnt [7:0]"},
     {"bits": 8, "name": "c2_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c2_cnt``
     -  
     - C2 counter (PLL output 2 divider) value.

.. _usb_reg_0031:

``0x0031`` - c3_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0031`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c3_cnt [7:0]"},
     {"bits": 8, "name": "c3_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c3_cnt``
     -  
     - C3 counter (PLL output 3 divider) value.

.. _usb_reg_0032:

``0x0032`` - c4_cnt
^^^^^^^^^^^^^^^^^^^

Address: ``0x0032`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "c4_cnt [7:0]"},
     {"bits": 8, "name": "c4_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``c4_cnt``
     -  
     - C4 counter (PLL output 4 divider) value.

.. _usb_reg_003e:

``0x003E`` - phcfg_samples
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003E`` | Default: ``0x0FFF`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "phcfg_samples [7:0]"},
     {"bits": 8, "name": "phcfg_samples [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``phcfg_samples``
     -  
     - Number of samples to compare during automatic phase configuration.

.. _usb_reg_003f:

``0x003F`` - phcfg_step
^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x003F`` | Default: ``0x0002`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "phcfg_step [7:0]"},
     {"bits": 8, "name": "phcfg_step [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``phcfg_step``
     -  
     - Step size used during automatic phase configuration.

.. _usb_reg_0061:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_1 memory test enable (WFM-player-driven self-test).
   * - ``[3]``
     - ``adf_tst_en``
     - 0=Disabled,1=Enabled
     - ADF4002 phase-detector test enable.
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

.. _usb_reg_0063:

``0x0063`` - test_frc_err
^^^^^^^^^^^^^^^^^^^^^^^^^

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - Insert error into DDR2_2 memory test.
   * - ``[4]``
     - ``ddr2_1_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error into DDR2_1 memory test.
   * - ``[3]``
     - ``adf_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error into ADF4002 phase-detector test.
   * - ``[2]``
     - ``vctcxo_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error into VCTCXO test.
   * - ``[1]``
     - ``si5351c_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error into Si5351C test.
   * - ``[0]``
     - ``fx3_pclk_tst_frc_err``
     - 0=Disabled,1=Enabled
     - Insert error into FX3 PCLK test.

.. _usb_reg_0065:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - ADF4002 phase-detector test complete status.
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

.. _usb_reg_0067:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_2 test pass result.
   * - ``[4]``
     - ``ddr2_1_tst_rez``
     -  
     - DDR2_1 test pass result.
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

.. _usb_reg_0069:

``0x0069`` - fx3_clk_cnt
^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0069`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "fx3_clk_cnt [7:0]"},
     {"bits": 8, "name": "fx3_clk_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_006a:

``0x006A`` - si5351c_clk7_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006A`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk7_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk7_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - Si5351C CLK7 counter value. Board-specific: SI5351C counters are addressed in descending order (0x6A=CLK7 .. 0x71=CLK0) on this board, unlike the ascending order used on other boards.

.. _usb_reg_006b:

``0x006B`` - si5351c_clk6_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006B`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk6_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk6_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_006c:

``0x006C`` - si5351c_clk5_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006C`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk5_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk5_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_006d:

``0x006D`` - si5351c_clk3_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006D`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk3_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk3_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_006f:

``0x006F`` - si5351c_clk2_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x006F`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk2_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk2_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_0070:

``0x0070`` - si5351c_clk1_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0070`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk1_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk1_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_0071:

``0x0071`` - si5351c_clk0_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0071`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "si5351c_clk0_cnt [7:0]"},
     {"bits": 8, "name": "si5351c_clk0_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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

.. _usb_reg_0072:

``0x0072`` - lmk_clk_cnt_l
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0072`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "lmk_clk_cnt_l [7:0]"},
     {"bits": 8, "name": "lmk_clk_cnt_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - LMK clock counter, bits [15:0].

.. _usb_reg_0073:

``0x0073`` - lmk_clk_cnt_h
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0073`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 7, "name": "lmk_clk_cnt_h [6:0]"},
     {"bits": 1, "name": "Reserved [7:7]"},
     {"bits": 8, "name": "Reserved [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[6:0]``
     - ``lmk_clk_cnt_h``
     -  
     - LMK clock counter, bits [22:16].

.. _usb_reg_0074:

``0x0074`` - adf_muxout_cnt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0074`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "adf_muxout_cnt [7:0]"},
     {"bits": 8, "name": "adf_muxout_cnt [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 40

   * - Bit(s)
     - Field
     - Values
     - Description
   * - ``[15:0]``
     - ``adf_muxout_cnt``
     -  
     - ADF4002 MUXOUT transition count value.

.. _usb_reg_0077:

``0x0077`` - ddr2_1_pnf_per_bit_l
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0077`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_l [7:0]"},
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_1 data [15:0] pass/fail-per-bit result. Board-specific: LimeSDR-USB has WFM-player-driven DDR2 self-test registers not present on other boards.

.. _usb_reg_0078:

``0x0078`` - ddr2_1_pnf_per_bit_h
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x0078`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_h [7:0]"},
     {"bits": 8, "name": "ddr2_1_pnf_per_bit_h [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_1 data [31:16] pass/fail-per-bit result.

.. _usb_reg_007a:

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
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_2 test fail flag.
   * - ``[1]``
     - ``ddr2_2_tst_pass``
     - 0=Not completed,1=Pass
     - DDR2_2 test pass flag.
   * - ``[0]``
     - ``ddr2_2_tst_cmplt``
     - 0=Not completed,1=Complete
     - DDR2_2 test complete flag.

.. _usb_reg_007b:

``0x007B`` - ddr2_2_pnf_per_bit_l
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007B`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_l [7:0]"},
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_l [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_2 data [15:0] pass/fail-per-bit result.

.. _usb_reg_007c:

``0x007C`` - ddr2_2_pnf_per_bit_h
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x007C`` | Default: ``not specified`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_h [7:0]"},
     {"bits": 8, "name": "ddr2_2_pnf_per_bit_h [15:8]"}
   ], "config": { "bits": 16, "lanes": 2, "hspace": 1150 } }

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
     - DDR2_2 data [31:16] pass/fail-per-bit result.

.. _usb_reg_00c0:

``0x00C0`` - board_gpio_ovrd
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C0`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_ovrd [7:0]"},
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
     - ``board_gpio_ovrd``
     - 0=Dedicated function,1=Overridden by user
     - GPIO override bits for FPGA_GPIO[7:0].

.. _usb_reg_00c2:

``0x00C2`` - board_gpio_rd
^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C2`` | Default: ``0x0000`` | Access: R

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_rd [7:0]"},
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
     - ``board_gpio_rd``
     - 0=Low,1=High
     - GPIO readback bits for FPGA_GPIO[7:0].

.. _usb_reg_00c4:

``0x00C4`` - board_gpio_dir
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C4`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_dir [7:0]"},
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
     - ``board_gpio_dir``
     - 0=Input,1=Output
     - GPIO direction bits for FPGA_GPIO[7:0]. Board-specific: firmware inverts this legacy 1=output convention before writing it to the underlying gpio_io CSR, whose own convention is 0=output.

.. _usb_reg_00c6:

``0x00C6`` - board_gpio_val
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00C6`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 8, "name": "board_gpio_val [7:0]"},
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
     - ``board_gpio_val``
     - 0=Low,1=High
     - GPIO output value bits for FPGA_GPIO[7:0].

.. _usb_reg_00cc:

``0x00CC`` - periph_output_ovrd_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CC`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "periph_output_ovrd_0"},
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
     - ``periph_output_ovrd_0``
     - 0=Dedicated,1=User controlled
     - Fan control override enable.

.. _usb_reg_00cd:

``0x00CD`` - periph_output_val_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Address: ``0x00CD`` | Default: ``0x0000`` | Access: R/W

.. wavedrom::

   { "reg": [
     {"bits": 1, "name": "periph_output_val_0"},
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
     - ``periph_output_val_0``
     - 0=OFF,1=ON
     - Fan control value.
