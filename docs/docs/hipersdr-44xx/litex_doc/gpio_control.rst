GPIO_CONTROL
============

Register Listing for GPIO_CONTROL
---------------------------------

+------------------------------------------------------------------------------------+----------------------------------------------------------+
| Register                                                                           | Address                                                  |
+====================================================================================+==========================================================+
| :ref:`GPIO_CONTROL_PORT_DIRECTION_U114 <GPIO_CONTROL_PORT_DIRECTION_U114>`         | :ref:`0xf0004000 <GPIO_CONTROL_PORT_DIRECTION_U114>`     |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_DIRECTION_U113 <GPIO_CONTROL_PORT_DIRECTION_U113>`         | :ref:`0xf0004004 <GPIO_CONTROL_PORT_DIRECTION_U113>`     |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_DIRECTION_U115 <GPIO_CONTROL_PORT_DIRECTION_U115>`         | :ref:`0xf0004008 <GPIO_CONTROL_PORT_DIRECTION_U115>`     |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_DIRECTION_U110 <GPIO_CONTROL_PORT_DIRECTION_U110>`         | :ref:`0xf000400c <GPIO_CONTROL_PORT_DIRECTION_U110>`     |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U114 <GPIO_CONTROL_POLARITY_INVERSION_U114>` | :ref:`0xf0004010 <GPIO_CONTROL_POLARITY_INVERSION_U114>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U113 <GPIO_CONTROL_POLARITY_INVERSION_U113>` | :ref:`0xf0004014 <GPIO_CONTROL_POLARITY_INVERSION_U113>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U115 <GPIO_CONTROL_POLARITY_INVERSION_U115>` | :ref:`0xf0004018 <GPIO_CONTROL_POLARITY_INVERSION_U115>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U110 <GPIO_CONTROL_POLARITY_INVERSION_U110>` | :ref:`0xf000401c <GPIO_CONTROL_POLARITY_INVERSION_U110>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_114 <GPIO_CONTROL_PORT_OUT_VALUE_114>`           | :ref:`0xf0004020 <GPIO_CONTROL_PORT_OUT_VALUE_114>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_113 <GPIO_CONTROL_PORT_OUT_VALUE_113>`           | :ref:`0xf0004024 <GPIO_CONTROL_PORT_OUT_VALUE_113>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_115 <GPIO_CONTROL_PORT_OUT_VALUE_115>`           | :ref:`0xf0004028 <GPIO_CONTROL_PORT_OUT_VALUE_115>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_110 <GPIO_CONTROL_PORT_OUT_VALUE_110>`           | :ref:`0xf000402c <GPIO_CONTROL_PORT_OUT_VALUE_110>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_114 <GPIO_CONTROL_PORT_IN_VALUE_114>`             | :ref:`0xf0004030 <GPIO_CONTROL_PORT_IN_VALUE_114>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_113 <GPIO_CONTROL_PORT_IN_VALUE_113>`             | :ref:`0xf0004034 <GPIO_CONTROL_PORT_IN_VALUE_113>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_115 <GPIO_CONTROL_PORT_IN_VALUE_115>`             | :ref:`0xf0004038 <GPIO_CONTROL_PORT_IN_VALUE_115>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_110 <GPIO_CONTROL_PORT_IN_VALUE_110>`             | :ref:`0xf000403c <GPIO_CONTROL_PORT_IN_VALUE_110>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_GPIO <GPIO_CONTROL_GPIO>`                                       | :ref:`0xf0004040 <GPIO_CONTROL_GPIO>`                    |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_GPIO2 <GPIO_CONTROL_GPIO2>`                                     | :ref:`0xf0004044 <GPIO_CONTROL_GPIO2>`                   |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_TDD_TXANT_PRE <GPIO_CONTROL_TDD_TXANT_PRE>`                     | :ref:`0xf0004048 <GPIO_CONTROL_TDD_TXANT_PRE>`           |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_TDD_TXANT_POST <GPIO_CONTROL_TDD_TXANT_POST>`                   | :ref:`0xf000404c <GPIO_CONTROL_TDD_TXANT_POST>`          |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_TDDCONTROLENABLE <GPIO_CONTROL_TDDCONTROLENABLE>`               | :ref:`0xf0004050 <GPIO_CONTROL_TDDCONTROLENABLE>`        |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_TDDSIGNALINVERT <GPIO_CONTROL_TDDSIGNALINVERT>`                 | :ref:`0xf0004054 <GPIO_CONTROL_TDDSIGNALINVERT>`         |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_REGISTER_CONTROL <GPIO_CONTROL_REGISTER_CONTROL>`               | :ref:`0xf0004058 <GPIO_CONTROL_REGISTER_CONTROL>`        |
+------------------------------------------------------------------------------------+----------------------------------------------------------+

GPIO_CONTROL_PORT_DIRECTION_U114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x0 = 0xf0004000`

    GPIO direction: 1: Input, 0: Output

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_DIRECTION_U114

        {
            "reg": [
                {"name": "port_direction_u114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_DIRECTION_U113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x4 = 0xf0004004`

    GPIO direction: 1: Input, 0: Output

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_DIRECTION_U113

        {
            "reg": [
                {"name": "port_direction_u113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_DIRECTION_U115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x8 = 0xf0004008`

    GPIO direction: 1: Input, 0: Output

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_DIRECTION_U115

        {
            "reg": [
                {"name": "port_direction_u115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_DIRECTION_U110
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0xc = 0xf000400c`

    GPIO direction: 1: Input, 0: Output

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_DIRECTION_U110

        {
            "reg": [
                {"name": "port_direction_u110[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x10 = 0xf0004010`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U114

        {
            "reg": [
                {"name": "polarity_inversion_u114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x14 = 0xf0004014`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U113

        {
            "reg": [
                {"name": "polarity_inversion_u113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x18 = 0xf0004018`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U115

        {
            "reg": [
                {"name": "polarity_inversion_u115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U110
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x1c = 0xf000401c`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U110

        {
            "reg": [
                {"name": "polarity_inversion_u110[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x20 = 0xf0004020`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_114

        {
            "reg": [
                {"name": "port_out_value_114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x24 = 0xf0004024`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_113

        {
            "reg": [
                {"name": "port_out_value_113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x28 = 0xf0004028`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_115

        {
            "reg": [
                {"name": "port_out_value_115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_110
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x2c = 0xf000402c`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_110

        {
            "reg": [
                {"name": "port_out_value_110[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x30 = 0xf0004030`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_114

        {
            "reg": [
                {"name": "port_in_value_114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x34 = 0xf0004034`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_113

        {
            "reg": [
                {"name": "port_in_value_113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x38 = 0xf0004038`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_115

        {
            "reg": [
                {"name": "port_in_value_115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_110
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x3c = 0xf000403c`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_110

        {
            "reg": [
                {"name": "port_in_value_110[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_GPIO
^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x40 = 0xf0004040`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_GPIO

        {
            "reg": [
                {"name": "gpio[31:0]", "bits": 32}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_GPIO2
^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x44 = 0xf0004044`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_GPIO2

        {
            "reg": [
                {"name": "gpio2[12:0]", "bits": 13},
                {"bits": 19},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_TDD_TXANT_PRE
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x48 = 0xf0004048`

    Number of cycles to delay enabling TDD signal

    .. wavedrom::
        :caption: GPIO_CONTROL_TDD_TXANT_PRE

        {
            "reg": [
                {"name": "tdd_txant_pre[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_TDD_TXANT_POST
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x4c = 0xf000404c`

    Number of cycles to delay disabling TDD signal

    .. wavedrom::
        :caption: GPIO_CONTROL_TDD_TXANT_POST

        {
            "reg": [
                {"name": "tdd_txant_post[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_TDDCONTROLENABLE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x50 = 0xf0004050`

    TDD Control Enable, bit per channel

    .. wavedrom::
        :caption: GPIO_CONTROL_TDDCONTROLENABLE

        {
            "reg": [
                {"name": "tddcontrolenable[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


GPIO_CONTROL_TDDSIGNALINVERT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x54 = 0xf0004054`

    TDD Signal Invert, bit per channel

    .. wavedrom::
        :caption: GPIO_CONTROL_TDDSIGNALINVERT

        {
            "reg": [
                {"name": "tddsignalinvert[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


GPIO_CONTROL_REGISTER_CONTROL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x58 = 0xf0004058`

    register control bitfield

    .. wavedrom::
        :caption: GPIO_CONTROL_REGISTER_CONTROL

        {
            "reg": [
                {"name": "u114_direction_write",  "bits": 1},
                {"name": "u113_direction_write",  "bits": 1},
                {"name": "u115_direction_write",  "bits": 1},
                {"name": "u110_direction_write",  "bits": 1},
                {"name": "u114_polarity_write",  "bits": 1},
                {"name": "u113_polarity_write",  "bits": 1},
                {"name": "u115_polarity_write",  "bits": 1},
                {"name": "u110_polarity_write",  "bits": 1},
                {"name": "u114_out_value_write",  "bits": 1},
                {"name": "u113_out_value_write",  "bits": 1},
                {"name": "u115_out_value_write",  "bits": 1},
                {"name": "u110_out_value_write",  "bits": 1},
                {"name": "u114_direction_read",  "bits": 1},
                {"name": "u113_direction_read",  "bits": 1},
                {"name": "u115_direction_read",  "bits": 1},
                {"name": "u110_direction_read",  "bits": 1},
                {"name": "u114_polarity_read",  "bits": 1},
                {"name": "u113_polarity_read",  "bits": 1},
                {"name": "u115_polarity_read",  "bits": 1},
                {"name": "u110_polarity_read",  "bits": 1},
                {"name": "u114_out_value_read",  "bits": 1},
                {"name": "u113_out_value_read",  "bits": 1},
                {"name": "u115_out_value_read",  "bits": 1},
                {"name": "u110_out_value_read",  "bits": 1},
                {"name": "u114_in_value_read",  "bits": 1},
                {"name": "u113_in_value_read",  "bits": 1},
                {"name": "u115_in_value_read",  "bits": 1},
                {"name": "u110_in_value_read",  "bits": 1},
                {"bits": 4}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+-------+----------------------+-------------+
| Field | Name                 | Description |
+=======+======================+=============+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+

