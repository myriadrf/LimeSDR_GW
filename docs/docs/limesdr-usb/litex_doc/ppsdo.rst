PPSDO
=====

Register Listing for PPSDO
--------------------------

+----------------------------------------------------------------------+---------------------------------------------------+
| Register                                                             | Address                                           |
+======================================================================+===================================================+
| :ref:`PPSDO_ENABLE <PPSDO_ENABLE>`                                   | :ref:`0xf000b000 <PPSDO_ENABLE>`                  |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_CONFIG_ONE_S_TARGET <PPSDO_CONFIG_ONE_S_TARGET>`         | :ref:`0xf000b004 <PPSDO_CONFIG_ONE_S_TARGET>`     |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_CONFIG_ONE_S_TOL <PPSDO_CONFIG_ONE_S_TOL>`               | :ref:`0xf000b008 <PPSDO_CONFIG_ONE_S_TOL>`        |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_CONFIG_TEN_S_TARGET <PPSDO_CONFIG_TEN_S_TARGET>`         | :ref:`0xf000b00c <PPSDO_CONFIG_TEN_S_TARGET>`     |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_CONFIG_TEN_S_TOL <PPSDO_CONFIG_TEN_S_TOL>`               | :ref:`0xf000b010 <PPSDO_CONFIG_TEN_S_TOL>`        |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_CONFIG_HUNDRED_S_TARGET <PPSDO_CONFIG_HUNDRED_S_TARGET>` | :ref:`0xf000b014 <PPSDO_CONFIG_HUNDRED_S_TARGET>` |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_CONFIG_HUNDRED_S_TOL <PPSDO_CONFIG_HUNDRED_S_TOL>`       | :ref:`0xf000b018 <PPSDO_CONFIG_HUNDRED_S_TOL>`    |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_ONE_S_ERROR <PPSDO_STATUS_ONE_S_ERROR>`           | :ref:`0xf000b01c <PPSDO_STATUS_ONE_S_ERROR>`      |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_TEN_S_ERROR <PPSDO_STATUS_TEN_S_ERROR>`           | :ref:`0xf000b020 <PPSDO_STATUS_TEN_S_ERROR>`      |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_HUNDRED_S_ERROR <PPSDO_STATUS_HUNDRED_S_ERROR>`   | :ref:`0xf000b024 <PPSDO_STATUS_HUNDRED_S_ERROR>`  |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_DAC_TUNED_VAL <PPSDO_STATUS_DAC_TUNED_VAL>`       | :ref:`0xf000b028 <PPSDO_STATUS_DAC_TUNED_VAL>`    |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_ACCURACY <PPSDO_STATUS_ACCURACY>`                 | :ref:`0xf000b02c <PPSDO_STATUS_ACCURACY>`         |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_PPS_ACTIVE <PPSDO_STATUS_PPS_ACTIVE>`             | :ref:`0xf000b030 <PPSDO_STATUS_PPS_ACTIVE>`       |
+----------------------------------------------------------------------+---------------------------------------------------+
| :ref:`PPSDO_STATUS_STATE <PPSDO_STATUS_STATE>`                       | :ref:`0xf000b034 <PPSDO_STATUS_STATE>`            |
+----------------------------------------------------------------------+---------------------------------------------------+

PPSDO_ENABLE
^^^^^^^^^^^^

`Address: 0xf000b000 + 0x0 = 0xf000b000`

    Enable control for PPSDO core

    .. wavedrom::
        :caption: PPSDO_ENABLE

        {
            "reg": [
                {"name": "enable", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PPSDO_CONFIG_ONE_S_TARGET
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x4 = 0xf000b004`

    Target value for 1-second interval.

    .. wavedrom::
        :caption: PPSDO_CONFIG_ONE_S_TARGET

        {
            "reg": [
                {"name": "config_one_s_target[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_CONFIG_ONE_S_TOL
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x8 = 0xf000b008`

    Tolerance for 1-second interval.

    .. wavedrom::
        :caption: PPSDO_CONFIG_ONE_S_TOL

        {
            "reg": [
                {"name": "config_one_s_tol[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_CONFIG_TEN_S_TARGET
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0xc = 0xf000b00c`

    Target value for 10-second interval.

    .. wavedrom::
        :caption: PPSDO_CONFIG_TEN_S_TARGET

        {
            "reg": [
                {"name": "config_ten_s_target[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_CONFIG_TEN_S_TOL
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x10 = 0xf000b010`

    Tolerance for 10-second interval.

    .. wavedrom::
        :caption: PPSDO_CONFIG_TEN_S_TOL

        {
            "reg": [
                {"name": "config_ten_s_tol[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_CONFIG_HUNDRED_S_TARGET
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x14 = 0xf000b014`

    Target value for 100-second interval.

    .. wavedrom::
        :caption: PPSDO_CONFIG_HUNDRED_S_TARGET

        {
            "reg": [
                {"name": "config_hundred_s_target[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_CONFIG_HUNDRED_S_TOL
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x18 = 0xf000b018`

    Tolerance for 100-second interval.

    .. wavedrom::
        :caption: PPSDO_CONFIG_HUNDRED_S_TOL

        {
            "reg": [
                {"name": "config_hundred_s_tol[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_STATUS_ONE_S_ERROR
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x1c = 0xf000b01c`

    Error value for 1-second interval.

    .. wavedrom::
        :caption: PPSDO_STATUS_ONE_S_ERROR

        {
            "reg": [
                {"name": "status_one_s_error[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_STATUS_TEN_S_ERROR
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x20 = 0xf000b020`

    Error value for 10-second interval.

    .. wavedrom::
        :caption: PPSDO_STATUS_TEN_S_ERROR

        {
            "reg": [
                {"name": "status_ten_s_error[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_STATUS_HUNDRED_S_ERROR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x24 = 0xf000b024`

    Error value for 100-second interval.

    .. wavedrom::
        :caption: PPSDO_STATUS_HUNDRED_S_ERROR

        {
            "reg": [
                {"name": "status_hundred_s_error[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_STATUS_DAC_TUNED_VAL
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x28 = 0xf000b028`

    DAC tuned value.

    .. wavedrom::
        :caption: PPSDO_STATUS_DAC_TUNED_VAL

        {
            "reg": [
                {"name": "status_dac_tuned_val[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PPSDO_STATUS_ACCURACY
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x2c = 0xf000b02c`

    Accuracy status.

    .. wavedrom::
        :caption: PPSDO_STATUS_ACCURACY

        {
            "reg": [
                {"name": "status_accuracy[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PPSDO_STATUS_PPS_ACTIVE
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x30 = 0xf000b030`

    PPS active status.

    .. wavedrom::
        :caption: PPSDO_STATUS_PPS_ACTIVE

        {
            "reg": [
                {"name": "status_pps_active", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PPSDO_STATUS_STATE
^^^^^^^^^^^^^^^^^^

`Address: 0xf000b000 + 0x34 = 0xf000b034`

    Current state.

    .. wavedrom::
        :caption: PPSDO_STATUS_STATE

        {
            "reg": [
                {"name": "status_state[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


