GPIO
====

Register Listing for GPIO
-------------------------

+--------------------------------------------------------+--------------------------------------------+
| Register                                               | Address                                    |
+========================================================+============================================+
| :ref:`GPIO_GPIO_OVERRIDE <GPIO_GPIO_OVERRIDE>`         | :ref:`0xf0003800 <GPIO_GPIO_OVERRIDE>`     |
+--------------------------------------------------------+--------------------------------------------+
| :ref:`GPIO_GPIO_OVERRIDE_DIR <GPIO_GPIO_OVERRIDE_DIR>` | :ref:`0xf0003804 <GPIO_GPIO_OVERRIDE_DIR>` |
+--------------------------------------------------------+--------------------------------------------+
| :ref:`GPIO_GPIO_OVERRIDE_VAL <GPIO_GPIO_OVERRIDE_VAL>` | :ref:`0xf0003808 <GPIO_GPIO_OVERRIDE_VAL>` |
+--------------------------------------------------------+--------------------------------------------+
| :ref:`GPIO_GPIO_VAL <GPIO_GPIO_VAL>`                   | :ref:`0xf000380c <GPIO_GPIO_VAL>`          |
+--------------------------------------------------------+--------------------------------------------+

GPIO_GPIO_OVERRIDE
^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x0 = 0xf0003800`

    GPIO Mode: 0: normal operation, 1: control is overriden.

    .. wavedrom::
        :caption: GPIO_GPIO_OVERRIDE

        {
            "reg": [
                {"name": "gpio_override[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


GPIO_GPIO_OVERRIDE_DIR
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x4 = 0xf0003804`

    GPIO override direction: 0: Output, 1: Input.

    .. wavedrom::
        :caption: GPIO_GPIO_OVERRIDE_DIR

        {
            "reg": [
                {"name": "gpio_override_dir[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


GPIO_GPIO_OVERRIDE_VAL
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x8 = 0xf0003808`

    GPIO Logic level: 0: High, 1: Low. (Dir must be set to output)

    .. wavedrom::
        :caption: GPIO_GPIO_OVERRIDE_VAL

        {
            "reg": [
                {"name": "gpio_override_val[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


GPIO_GPIO_VAL
^^^^^^^^^^^^^

`Address: 0xf0003800 + 0xc = 0xf000380c`

    GPIO current value

    .. wavedrom::
        :caption: GPIO_GPIO_VAL

        {
            "reg": [
                {"name": "gpio_val[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


