AFE
===

Register Listing for AFE
------------------------

+------------------------------------------------------------------+-------------------------------------------------+
| Register                                                         | Address                                         |
+==================================================================+=================================================+
| :ref:`AFE_REG00 <AFE_REG00>`                                     | :ref:`0xf000d800 <AFE_REG00>`                   |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_CORE_CTRL <AFE_CORE_CTRL>`                             | :ref:`0xf000d804 <AFE_CORE_CTRL>`               |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RX_CTRL <AFE_RX_CTRL>`                                 | :ref:`0xf000d808 <AFE_RX_CTRL>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RX_CFG0 <AFE_RX_CFG0>`                                 | :ref:`0xf000d80c <AFE_RX_CFG0>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RX_CFG1 <AFE_RX_CFG1>`                                 | :ref:`0xf000d810 <AFE_RX_CFG1>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RX_CFG2 <AFE_RX_CFG2>`                                 | :ref:`0xf000d814 <AFE_RX_CFG2>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RX_CFG3 <AFE_RX_CFG3>`                                 | :ref:`0xf000d818 <AFE_RX_CFG3>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RX_STATUS0 <AFE_RX_STATUS0>`                           | :ref:`0xf000d81c <AFE_RX_STATUS0>`              |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_TX_CTRL <AFE_TX_CTRL>`                                 | :ref:`0xf000d820 <AFE_TX_CTRL>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_TX_CFG0 <AFE_TX_CFG0>`                                 | :ref:`0xf000d824 <AFE_TX_CFG0>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_TX_CFG1 <AFE_TX_CFG1>`                                 | :ref:`0xf000d828 <AFE_TX_CFG1>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_TX_CFG3 <AFE_TX_CFG3>`                                 | :ref:`0xf000d82c <AFE_TX_CFG3>`                 |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_TX_STATUS0 <AFE_TX_STATUS0>`                           | :ref:`0xf000d830 <AFE_TX_STATUS0>`              |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_CH_EN <AFE_CH_EN>`                                     | :ref:`0xf000d834 <AFE_CH_EN>`                   |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_SMPL_WIDTH <AFE_SMPL_WIDTH>`                           | :ref:`0xf000d838 <AFE_SMPL_WIDTH>`              |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_PKT_SIZE <AFE_PKT_SIZE>`                               | :ref:`0xf000d83c <AFE_PKT_SIZE>`                |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_CORE_STATUS0 <AFE_CORE_STATUS0>`                       | :ref:`0xf000d840 <AFE_CORE_STATUS0>`            |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_DECIMATE_STAGE_COUNT <AFE_DECIMATE_STAGE_COUNT>`       | :ref:`0xf000d844 <AFE_DECIMATE_STAGE_COUNT>`    |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_DECIMATE_BLANK0 <AFE_DECIMATE_BLANK0>`                 | :ref:`0xf000d848 <AFE_DECIMATE_BLANK0>`         |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_DECIMATE_BLANK1 <AFE_DECIMATE_BLANK1>`                 | :ref:`0xf000d84c <AFE_DECIMATE_BLANK1>`         |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_DECIMATE_BLANK2 <AFE_DECIMATE_BLANK2>`                 | :ref:`0xf000d850 <AFE_DECIMATE_BLANK2>`         |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_INTERPOLATE_STAGE_COUNT <AFE_INTERPOLATE_STAGE_COUNT>` | :ref:`0xf000d854 <AFE_INTERPOLATE_STAGE_COUNT>` |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_INTERPOLATE_BLANK0 <AFE_INTERPOLATE_BLANK0>`           | :ref:`0xf000d858 <AFE_INTERPOLATE_BLANK0>`      |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_INTERPOLATE_BLANK1 <AFE_INTERPOLATE_BLANK1>`           | :ref:`0xf000d85c <AFE_INTERPOLATE_BLANK1>`      |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_INTERPOLATE_BLANK2 <AFE_INTERPOLATE_BLANK2>`           | :ref:`0xf000d860 <AFE_INTERPOLATE_BLANK2>`      |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`AFE_RESAMPLER_MAX_VALUE <AFE_RESAMPLER_MAX_VALUE>`         | :ref:`0xf000d864 <AFE_RESAMPLER_MAX_VALUE>`     |
+------------------------------------------------------------------+-------------------------------------------------+

AFE_REG00
^^^^^^^^^

`Address: 0xf000d800 + 0x0 = 0xf000d800`


    .. wavedrom::
        :caption: AFE_REG00

        {
            "reg": [
                {"name": "afe_reset",  "bits": 1},
                {"name": "afe_trst",  "bits": 1},
                {"name": "afe_sleep",  "bits": 1},
                {"bits": 29}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-----------+-------------+
| Field | Name      | Description |
+=======+===========+=============+
+-------+-----------+-------------+
+-------+-----------+-------------+
+-------+-----------+-------------+

AFE_CORE_CTRL
^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x4 = 0xf000d804`


    .. wavedrom::
        :caption: AFE_CORE_CTRL

        {
            "reg": [
                {"name": "afe_core_rst_n",  "bits": 1},
                {"name": "afe_init_trigger",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------------------+-------------+
| Field | Name             | Description |
+=======+==================+=============+
+-------+------------------+-------------+
+-------+------------------+-------------+

AFE_RX_CTRL
^^^^^^^^^^^

`Address: 0xf000d800 + 0x8 = 0xf000d808`


    .. wavedrom::
        :caption: AFE_RX_CTRL

        {
            "reg": [
                {"name": "tiafe_rx_sync_reset",  "attr": '1', "bits": 1},
                {"name": "rx_clr_sysref_realign_count",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-----------------------------+-------------+
| Field | Name                        | Description |
+=======+=============================+=============+
+-------+-----------------------------+-------------+
+-------+-----------------------------+-------------+

AFE_RX_CFG0
^^^^^^^^^^^

`Address: 0xf000d800 + 0xc = 0xf000d80c`


    .. wavedrom::
        :caption: AFE_RX_CFG0

        {
            "reg": [
                {"name": "tiafe_cfg_rx_lane_enabled",  "bits": 4},
                {"name": "tiafe_cfg_rx_lane_polarity",  "bits": 4},
                {"bits": 24}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+----------------------------+-------------+
| Field | Name                       | Description |
+=======+============================+=============+
+-------+----------------------------+-------------+
+-------+----------------------------+-------------+

AFE_RX_CFG1
^^^^^^^^^^^

`Address: 0xf000d800 + 0x10 = 0xf000d810`


    .. wavedrom::
        :caption: AFE_RX_CFG1

        {
            "reg": [
                {"name": "tiafe_cfg_rx_lane_map",  "bits": 16},
                {"bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-----------------------+-------------+
| Field  | Name                  | Description |
+========+=======================+=============+
+--------+-----------------------+-------------+

AFE_RX_CFG2
^^^^^^^^^^^

`Address: 0xf000d800 + 0x14 = 0xf000d814`


    .. wavedrom::
        :caption: AFE_RX_CFG2

        {
            "reg": [
                {"name": "tiafe_cfg_rx_buffer_release_delay",  "bits": 10},
                {"bits": 22}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+-------+-----------------------------------+-------------+
| Field | Name                              | Description |
+=======+===================================+=============+
+-------+-----------------------------------+-------------+

AFE_RX_CFG3
^^^^^^^^^^^

`Address: 0xf000d800 + 0x18 = 0xf000d818`


    .. wavedrom::
        :caption: AFE_RX_CFG3

        {
            "reg": [
                {"name": "swap_iq",  "attr": '15', "bits": 4},
                {"bits": 28}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+---------+-------------+
| Field | Name    | Description |
+=======+=========+=============+
+-------+---------+-------------+

AFE_RX_STATUS0
^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x1c = 0xf000d81c`


    .. wavedrom::
        :caption: AFE_RX_STATUS0

        {
            "reg": [
                {"name": "jesd_rx_sysref_realign_count",  "bits": 4},
                {"bits": 28}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------------------------------+-------------+
| Field | Name                         | Description |
+=======+==============================+=============+
+-------+------------------------------+-------------+

AFE_TX_CTRL
^^^^^^^^^^^

`Address: 0xf000d800 + 0x20 = 0xf000d820`


    .. wavedrom::
        :caption: AFE_TX_CTRL

        {
            "reg": [
                {"name": "tiafe_tx_sync_reset",  "attr": '1', "bits": 1},
                {"name": "tx_clr_sysref_realign_count",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-----------------------------+-------------+
| Field | Name                        | Description |
+=======+=============================+=============+
+-------+-----------------------------+-------------+
+-------+-----------------------------+-------------+

AFE_TX_CFG0
^^^^^^^^^^^

`Address: 0xf000d800 + 0x24 = 0xf000d824`


    .. wavedrom::
        :caption: AFE_TX_CFG0

        {
            "reg": [
                {"name": "tiafe_cfg_tx_lane_enabled",  "bits": 4},
                {"name": "tiafe_cfg_tx_lane_polarity",  "bits": 4},
                {"bits": 24}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+----------------------------+-------------+
| Field | Name                       | Description |
+=======+============================+=============+
+-------+----------------------------+-------------+
+-------+----------------------------+-------------+

AFE_TX_CFG1
^^^^^^^^^^^

`Address: 0xf000d800 + 0x28 = 0xf000d828`


    .. wavedrom::
        :caption: AFE_TX_CFG1

        {
            "reg": [
                {"name": "tiafe_cfg_tx_lane_map",  "bits": 16},
                {"bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-----------------------+-------------+
| Field  | Name                  | Description |
+========+=======================+=============+
+--------+-----------------------+-------------+

AFE_TX_CFG3
^^^^^^^^^^^

`Address: 0xf000d800 + 0x2c = 0xf000d82c`


    .. wavedrom::
        :caption: AFE_TX_CFG3

        {
            "reg": [
                {"name": "swap_iq",  "attr": '15', "bits": 4},
                {"bits": 28}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+---------+-------------+
| Field | Name    | Description |
+=======+=========+=============+
+-------+---------+-------------+

AFE_TX_STATUS0
^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x30 = 0xf000d830`


    .. wavedrom::
        :caption: AFE_TX_STATUS0

        {
            "reg": [
                {"name": "jesd_tx_sysref_realign_count",  "bits": 4},
                {"bits": 28}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------------------------------+-------------+
| Field | Name                         | Description |
+=======+==============================+=============+
+-------+------------------------------+-------------+

AFE_CH_EN
^^^^^^^^^

`Address: 0xf000d800 + 0x34 = 0xf000d834`

    01 - Channel A enabled, 10 - Channel B enabled, 11 - Channels A and B enabled

    .. wavedrom::
        :caption: AFE_CH_EN

        {
            "reg": [
                {"name": "ch_en[1:0]", "attr": 'reset: 3', "bits": 2},
                {"bits": 30},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_SMPL_WIDTH
^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x38 = 0xf000d838`

    10 - 12bit, 01 - Reserved, 00 - 16bit

    .. wavedrom::
        :caption: AFE_SMPL_WIDTH

        {
            "reg": [
                {"name": "smpl_width[1:0]", "attr": 'reset: 2', "bits": 2},
                {"bits": 30},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_PKT_SIZE
^^^^^^^^^^^^

`Address: 0xf000d800 + 0x3c = 0xf000d83c`

    Packet Size in bytes,

    .. wavedrom::
        :caption: AFE_PKT_SIZE

        {
            "reg": [
                {"name": "pkt_size[15:0]", "attr": 'reset: 253', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


AFE_CORE_STATUS0
^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x40 = 0xf000d840`


    .. wavedrom::
        :caption: AFE_CORE_STATUS0

        {
            "reg": [
                {"name": "xcvr_plls_locked",  "bits": 1},
                {"name": "rx_all_lanes_locked",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+---------------------+-------------+
| Field | Name                | Description |
+=======+=====================+=============+
+-------+---------------------+-------------+
+-------+---------------------+-------------+

AFE_DECIMATE_STAGE_COUNT
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x44 = 0xf000d844`

    Number of active decimator stages (0..4).

    .. wavedrom::
        :caption: AFE_DECIMATE_STAGE_COUNT

        {
            "reg": [
                {"name": "decimate_stage_count[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_DECIMATE_BLANK0
^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x48 = 0xf000d848`

    Blank

    .. wavedrom::
        :caption: AFE_DECIMATE_BLANK0

        {
            "reg": [
                {"name": "decimate_blank0", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_DECIMATE_BLANK1
^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x4c = 0xf000d84c`

    Blank

    .. wavedrom::
        :caption: AFE_DECIMATE_BLANK1

        {
            "reg": [
                {"name": "decimate_blank1", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_DECIMATE_BLANK2
^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x50 = 0xf000d850`

    Blank

    .. wavedrom::
        :caption: AFE_DECIMATE_BLANK2

        {
            "reg": [
                {"name": "decimate_blank2", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_INTERPOLATE_STAGE_COUNT
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x54 = 0xf000d854`

    Number of active interpolator stages (0..4).

    .. wavedrom::
        :caption: AFE_INTERPOLATE_STAGE_COUNT

        {
            "reg": [
                {"name": "interpolate_stage_count[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_INTERPOLATE_BLANK0
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x58 = 0xf000d858`

    Blank

    .. wavedrom::
        :caption: AFE_INTERPOLATE_BLANK0

        {
            "reg": [
                {"name": "interpolate_blank0", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_INTERPOLATE_BLANK1
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x5c = 0xf000d85c`

    Blank

    .. wavedrom::
        :caption: AFE_INTERPOLATE_BLANK1

        {
            "reg": [
                {"name": "interpolate_blank1", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_INTERPOLATE_BLANK2
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x60 = 0xf000d860`

    Blank

    .. wavedrom::
        :caption: AFE_INTERPOLATE_BLANK2

        {
            "reg": [
                {"name": "interpolate_blank2", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_RESAMPLER_MAX_VALUE
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000d800 + 0x64 = 0xf000d864`

    Maximum divider value for resampling

    .. wavedrom::
        :caption: AFE_RESAMPLER_MAX_VALUE

        {
            "reg": [
                {"name": "resampler_max_value[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


