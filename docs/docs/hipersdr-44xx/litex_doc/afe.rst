AFE
===

Register Listing for AFE
------------------------

+----------------------------------------------------------------+------------------------------------------------+
| Register                                                       | Address                                        |
+================================================================+================================================+
| :ref:`AFE_REG00 <AFE_REG00>`                                   | :ref:`0xf0002000 <AFE_REG00>`                  |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_CORE_CTRL <AFE_CORE_CTRL>`                           | :ref:`0xf0002004 <AFE_CORE_CTRL>`              |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_CTRL <AFE_RX_CTRL>`                               | :ref:`0xf0002008 <AFE_RX_CTRL>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_CFG0 <AFE_RX_CFG0>`                               | :ref:`0xf000200c <AFE_RX_CFG0>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_CFG1 <AFE_RX_CFG1>`                               | :ref:`0xf0002010 <AFE_RX_CFG1>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_CFG2 <AFE_RX_CFG2>`                               | :ref:`0xf0002014 <AFE_RX_CFG2>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_STATUS0 <AFE_RX_STATUS0>`                         | :ref:`0xf0002018 <AFE_RX_STATUS0>`             |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_CTRL <AFE_TX_CTRL>`                               | :ref:`0xf000201c <AFE_TX_CTRL>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_CFG0 <AFE_TX_CFG0>`                               | :ref:`0xf0002020 <AFE_TX_CFG0>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_CFG1 <AFE_TX_CFG1>`                               | :ref:`0xf0002024 <AFE_TX_CFG1>`                |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_STATUS0 <AFE_TX_STATUS0>`                         | :ref:`0xf0002028 <AFE_TX_STATUS0>`             |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_CH_EN <AFE_CH_EN>`                                   | :ref:`0xf000202c <AFE_CH_EN>`                  |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_SMPL_WIDTH <AFE_SMPL_WIDTH>`                         | :ref:`0xf0002030 <AFE_SMPL_WIDTH>`             |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_PKT_SIZE <AFE_PKT_SIZE>`                             | :ref:`0xf0002034 <AFE_PKT_SIZE>`               |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_A_RESAMPLER_OUT_MUX <AFE_RX_A_RESAMPLER_OUT_MUX>` | :ref:`0xf0002038 <AFE_RX_A_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_B_RESAMPLER_OUT_MUX <AFE_RX_B_RESAMPLER_OUT_MUX>` | :ref:`0xf000203c <AFE_RX_B_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_C_RESAMPLER_OUT_MUX <AFE_RX_C_RESAMPLER_OUT_MUX>` | :ref:`0xf0002040 <AFE_RX_C_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_RX_D_RESAMPLER_OUT_MUX <AFE_RX_D_RESAMPLER_OUT_MUX>` | :ref:`0xf0002044 <AFE_RX_D_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_A_RESAMPLER_OUT_MUX <AFE_TX_A_RESAMPLER_OUT_MUX>` | :ref:`0xf0002048 <AFE_TX_A_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_B_RESAMPLER_OUT_MUX <AFE_TX_B_RESAMPLER_OUT_MUX>` | :ref:`0xf000204c <AFE_TX_B_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_C_RESAMPLER_OUT_MUX <AFE_TX_C_RESAMPLER_OUT_MUX>` | :ref:`0xf0002050 <AFE_TX_C_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`AFE_TX_D_RESAMPLER_OUT_MUX <AFE_TX_D_RESAMPLER_OUT_MUX>` | :ref:`0xf0002054 <AFE_TX_D_RESAMPLER_OUT_MUX>` |
+----------------------------------------------------------------+------------------------------------------------+

AFE_REG00
^^^^^^^^^

`Address: 0xf0002000 + 0x0 = 0xf0002000`


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

`Address: 0xf0002000 + 0x4 = 0xf0002004`


    .. wavedrom::
        :caption: AFE_CORE_CTRL

        {
            "reg": [
                {"name": "afe_core_rst_n",  "bits": 1},
                {"bits": 31}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+----------------+-------------+
| Field | Name           | Description |
+=======+================+=============+
+-------+----------------+-------------+

AFE_RX_CTRL
^^^^^^^^^^^

`Address: 0xf0002000 + 0x8 = 0xf0002008`


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

`Address: 0xf0002000 + 0xc = 0xf000200c`


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

`Address: 0xf0002000 + 0x10 = 0xf0002010`


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

`Address: 0xf0002000 + 0x14 = 0xf0002014`


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

AFE_RX_STATUS0
^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x18 = 0xf0002018`


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

`Address: 0xf0002000 + 0x1c = 0xf000201c`


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

`Address: 0xf0002000 + 0x20 = 0xf0002020`


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

`Address: 0xf0002000 + 0x24 = 0xf0002024`


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

AFE_TX_STATUS0
^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x28 = 0xf0002028`


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

`Address: 0xf0002000 + 0x2c = 0xf000202c`

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

`Address: 0xf0002000 + 0x30 = 0xf0002030`

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

`Address: 0xf0002000 + 0x34 = 0xf0002034`

    Packet Size in bytes,

    .. wavedrom::
        :caption: AFE_PKT_SIZE

        {
            "reg": [
                {"name": "pkt_size[15:0]", "attr": 'reset: 253', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


AFE_RX_A_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x38 = 0xf0002038`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_RX_A_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "rx_a_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_RX_B_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x3c = 0xf000203c`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_RX_B_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "rx_b_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_RX_C_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x40 = 0xf0002040`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_RX_C_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "rx_c_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_RX_D_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x44 = 0xf0002044`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_RX_D_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "rx_d_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_TX_A_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x48 = 0xf0002048`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_TX_A_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "tx_a_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_TX_B_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x4c = 0xf000204c`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_TX_B_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "tx_b_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_TX_C_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x50 = 0xf0002050`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_TX_C_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "tx_c_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


AFE_TX_D_RESAMPLER_OUT_MUX
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x54 = 0xf0002054`

    Select which resampling stage should be used as output

    .. wavedrom::
        :caption: AFE_TX_D_RESAMPLER_OUT_MUX

        {
            "reg": [
                {"name": "tx_d_resampler_out_mux[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


