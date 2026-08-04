RFSW_CONTROL
============

Register Listing for RFSW_CONTROL
---------------------------------

+------------------------------------------------------------------+-------------------------------------------------+
| Register                                                         | Address                                         |
+==================================================================+=================================================+
| :ref:`RFSW_CONTROL_TDD_MANUAL_VAL <RFSW_CONTROL_TDD_MANUAL_VAL>` | :ref:`0xf000a000 <RFSW_CONTROL_TDD_MANUAL_VAL>` |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`RFSW_CONTROL_TDD_AUTO_EN <RFSW_CONTROL_TDD_AUTO_EN>`       | :ref:`0xf000a004 <RFSW_CONTROL_TDD_AUTO_EN>`    |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`RFSW_CONTROL_TDD_INVERT <RFSW_CONTROL_TDD_INVERT>`         | :ref:`0xf000a008 <RFSW_CONTROL_TDD_INVERT>`     |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`RFSW_CONTROL_RFSW_RX <RFSW_CONTROL_RFSW_RX>`               | :ref:`0xf000a00c <RFSW_CONTROL_RFSW_RX>`        |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`RFSW_CONTROL_RFSW_TX <RFSW_CONTROL_RFSW_TX>`               | :ref:`0xf000a010 <RFSW_CONTROL_RFSW_TX>`        |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`RFSW_CONTROL_RFSW_AUTO_EN <RFSW_CONTROL_RFSW_AUTO_EN>`     | :ref:`0xf000a014 <RFSW_CONTROL_RFSW_AUTO_EN>`   |
+------------------------------------------------------------------+-------------------------------------------------+

RFSW_CONTROL_TDD_MANUAL_VAL
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x0 = 0xf000a000`

    TDD Signal manual control value

    .. wavedrom::
        :caption: RFSW_CONTROL_TDD_MANUAL_VAL

        {
            "reg": [
                {"name": "tdd_manual_val", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


RFSW_CONTROL_TDD_AUTO_EN
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x4 = 0xf000a004`

    0- TDD auto control disabled, 1-TDD auto control enabled

    .. wavedrom::
        :caption: RFSW_CONTROL_TDD_AUTO_EN

        {
            "reg": [
                {"name": "tdd_auto_en", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


RFSW_CONTROL_TDD_INVERT
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x8 = 0xf000a008`

    0- TDD Control signal not inverted, 1- TDD Control signal inverted

    .. wavedrom::
        :caption: RFSW_CONTROL_TDD_INVERT

        {
            "reg": [
                {"name": "tdd_invert", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


RFSW_CONTROL_RFSW_RX
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0xc = 0xf000a00c`

    00- RF1 (WIDE), 01- RF2 (LOW), 10- RF3 (HIGH)

    .. wavedrom::
        :caption: RFSW_CONTROL_RFSW_RX

        {
            "reg": [
                {"name": "rfsw_rx[1:0]", "bits": 2},
                {"bits": 30},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


RFSW_CONTROL_RFSW_TX
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x10 = 0xf000a010`

    0- TX1_2 (BAND2), 1- TX1_1 (BAND1)

    .. wavedrom::
        :caption: RFSW_CONTROL_RFSW_TX

        {
            "reg": [
                {"name": "rfsw_tx", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


RFSW_CONTROL_RFSW_AUTO_EN
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x14 = 0xf000a014`

    0- RFSW Auto control disabled, 1- RFSW Auto control Enabled

    .. wavedrom::
        :caption: RFSW_CONTROL_RFSW_AUTO_EN

        {
            "reg": [
                {"name": "rfsw_auto_en", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


