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
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U114 <GPIO_CONTROL_POLARITY_INVERSION_U114>` | :ref:`0xf000400c <GPIO_CONTROL_POLARITY_INVERSION_U114>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U113 <GPIO_CONTROL_POLARITY_INVERSION_U113>` | :ref:`0xf0004010 <GPIO_CONTROL_POLARITY_INVERSION_U113>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_POLARITY_INVERSION_U115 <GPIO_CONTROL_POLARITY_INVERSION_U115>` | :ref:`0xf0004014 <GPIO_CONTROL_POLARITY_INVERSION_U115>` |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_114 <GPIO_CONTROL_PORT_OUT_VALUE_114>`           | :ref:`0xf0004018 <GPIO_CONTROL_PORT_OUT_VALUE_114>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_113 <GPIO_CONTROL_PORT_OUT_VALUE_113>`           | :ref:`0xf000401c <GPIO_CONTROL_PORT_OUT_VALUE_113>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_OUT_VALUE_115 <GPIO_CONTROL_PORT_OUT_VALUE_115>`           | :ref:`0xf0004020 <GPIO_CONTROL_PORT_OUT_VALUE_115>`      |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_114 <GPIO_CONTROL_PORT_IN_VALUE_114>`             | :ref:`0xf0004024 <GPIO_CONTROL_PORT_IN_VALUE_114>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_113 <GPIO_CONTROL_PORT_IN_VALUE_113>`             | :ref:`0xf0004028 <GPIO_CONTROL_PORT_IN_VALUE_113>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_PORT_IN_VALUE_115 <GPIO_CONTROL_PORT_IN_VALUE_115>`             | :ref:`0xf000402c <GPIO_CONTROL_PORT_IN_VALUE_115>`       |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_GPIO <GPIO_CONTROL_GPIO>`                                       | :ref:`0xf0004030 <GPIO_CONTROL_GPIO>`                    |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_GPIO2 <GPIO_CONTROL_GPIO2>`                                     | :ref:`0xf0004034 <GPIO_CONTROL_GPIO2>`                   |
+------------------------------------------------------------------------------------+----------------------------------------------------------+
| :ref:`GPIO_CONTROL_REGISTER_CONTROL <GPIO_CONTROL_REGISTER_CONTROL>`               | :ref:`0xf0004038 <GPIO_CONTROL_REGISTER_CONTROL>`        |
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
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
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
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
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
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0xc = 0xf000400c`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U114

        {
            "reg": [
                {"name": "polarity_inversion_u114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x10 = 0xf0004010`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U113

        {
            "reg": [
                {"name": "polarity_inversion_u113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_POLARITY_INVERSION_U115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x14 = 0xf0004014`

    GPIO polarity: 1: Inverted, 0: Normal

    .. wavedrom::
        :caption: GPIO_CONTROL_POLARITY_INVERSION_U115

        {
            "reg": [
                {"name": "polarity_inversion_u115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x18 = 0xf0004018`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_114

        {
            "reg": [
                {"name": "port_out_value_114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x1c = 0xf000401c`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_113

        {
            "reg": [
                {"name": "port_out_value_113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_OUT_VALUE_115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x20 = 0xf0004020`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_OUT_VALUE_115

        {
            "reg": [
                {"name": "port_out_value_115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_114
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x24 = 0xf0004024`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_114

        {
            "reg": [
                {"name": "port_in_value_114[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_113
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x28 = 0xf0004028`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_113

        {
            "reg": [
                {"name": "port_in_value_113[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_PORT_IN_VALUE_115
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x2c = 0xf000402c`

    GPIO input value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_PORT_IN_VALUE_115

        {
            "reg": [
                {"name": "port_in_value_115[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_GPIO
^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x30 = 0xf0004030`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_GPIO

        {
            "reg": [
                {"name": "gpio[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_GPIO2
^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x34 = 0xf0004034`

    GPIO output value: 1: High, 0: Low

    .. wavedrom::
        :caption: GPIO_CONTROL_GPIO2

        {
            "reg": [
                {"name": "gpio2[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GPIO_CONTROL_REGISTER_CONTROL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x38 = 0xf0004038`

    register control bitfield

    .. wavedrom::
        :caption: GPIO_CONTROL_REGISTER_CONTROL

        {
            "reg": [
                {"name": "u114_direction_write",  "bits": 1},
                {"name": "u113_direction_write",  "bits": 1},
                {"name": "u115_direction_write",  "bits": 1},
                {"name": "u114_polarity_write",  "bits": 1},
                {"name": "u113_polarity_write",  "bits": 1},
                {"name": "u115_polarity_write",  "bits": 1},
                {"name": "u114_out_value_write",  "bits": 1},
                {"name": "u113_out_value_write",  "bits": 1},
                {"name": "u115_out_value_write",  "bits": 1},
                {"name": "u114_direction_read",  "bits": 1},
                {"name": "u113_direction_read",  "bits": 1},
                {"name": "u115_direction_read",  "bits": 1},
                {"name": "u114_polarity_read",  "bits": 1},
                {"name": "u113_polarity_read",  "bits": 1},
                {"name": "u115_polarity_read",  "bits": 1},
                {"name": "u114_out_value_read",  "bits": 1},
                {"name": "u113_out_value_read",  "bits": 1},
                {"name": "u115_out_value_read",  "bits": 1},
                {"name": "u114_in_value_read",  "bits": 1},
                {"name": "u113_in_value_read",  "bits": 1},
                {"name": "u115_in_value_read",  "bits": 1},
                {"bits": 11}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
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

