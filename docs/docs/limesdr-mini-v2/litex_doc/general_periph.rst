GENERAL_PERIPH
==============

Register Listing for GENERAL_PERIPH
-----------------------------------

+----------------------------------------------------------------------------------+---------------------------------------------------------+
| Register                                                                         | Address                                                 |
+==================================================================================+=========================================================+
| :ref:`GENERAL_PERIPH_BOARD_GPIO_OVRD <GENERAL_PERIPH_BOARD_GPIO_OVRD>`           | :ref:`0xf0001000 <GENERAL_PERIPH_BOARD_GPIO_OVRD>`      |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_BOARD_GPIO_RD <GENERAL_PERIPH_BOARD_GPIO_RD>`               | :ref:`0xf0001004 <GENERAL_PERIPH_BOARD_GPIO_RD>`        |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_BOARD_GPIO_DIR <GENERAL_PERIPH_BOARD_GPIO_DIR>`             | :ref:`0xf0001008 <GENERAL_PERIPH_BOARD_GPIO_DIR>`       |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_BOARD_GPIO_VAL <GENERAL_PERIPH_BOARD_GPIO_VAL>`             | :ref:`0xf000100c <GENERAL_PERIPH_BOARD_GPIO_VAL>`       |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_PERIPH_INPUT_RD_0 <GENERAL_PERIPH_PERIPH_INPUT_RD_0>`       | :ref:`0xf0001010 <GENERAL_PERIPH_PERIPH_INPUT_RD_0>`    |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_PERIPH_INPUT_RD_1 <GENERAL_PERIPH_PERIPH_INPUT_RD_1>`       | :ref:`0xf0001014 <GENERAL_PERIPH_PERIPH_INPUT_RD_1>`    |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_0 <GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_0>` | :ref:`0xf0001018 <GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_0>` |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_PERIPH_OUTPUT_VAL_0 <GENERAL_PERIPH_PERIPH_OUTPUT_VAL_0>`   | :ref:`0xf000101c <GENERAL_PERIPH_PERIPH_OUTPUT_VAL_0>`  |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_1 <GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_1>` | :ref:`0xf0001020 <GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_1>` |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_PERIPH_OUTPUT_VAL_1 <GENERAL_PERIPH_PERIPH_OUTPUT_VAL_1>`   | :ref:`0xf0001024 <GENERAL_PERIPH_PERIPH_OUTPUT_VAL_1>`  |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_FPGA_LED_CTRL <GENERAL_PERIPH_FPGA_LED_CTRL>`               | :ref:`0xf0001028 <GENERAL_PERIPH_FPGA_LED_CTRL>`        |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`GENERAL_PERIPH_FX3_LED_CTRL <GENERAL_PERIPH_FX3_LED_CTRL>`                 | :ref:`0xf000102c <GENERAL_PERIPH_FX3_LED_CTRL>`         |
+----------------------------------------------------------------------------------+---------------------------------------------------------+

GENERAL_PERIPH_BOARD_GPIO_OVRD
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x0 = 0xf0001000`


    .. wavedrom::
        :caption: GENERAL_PERIPH_BOARD_GPIO_OVRD

        {
            "reg": [
                {"name": "board_gpio_ovrd[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_BOARD_GPIO_RD
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x4 = 0xf0001004`


    .. wavedrom::
        :caption: GENERAL_PERIPH_BOARD_GPIO_RD

        {
            "reg": [
                {"name": "board_gpio_rd[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_BOARD_GPIO_DIR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x8 = 0xf0001008`


    .. wavedrom::
        :caption: GENERAL_PERIPH_BOARD_GPIO_DIR

        {
            "reg": [
                {"name": "board_gpio_dir[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_BOARD_GPIO_VAL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0xc = 0xf000100c`


    .. wavedrom::
        :caption: GENERAL_PERIPH_BOARD_GPIO_VAL

        {
            "reg": [
                {"name": "board_gpio_val[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_PERIPH_INPUT_RD_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x10 = 0xf0001010`


    .. wavedrom::
        :caption: GENERAL_PERIPH_PERIPH_INPUT_RD_0

        {
            "reg": [
                {"name": "periph_input_rd_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_PERIPH_INPUT_RD_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x14 = 0xf0001014`


    .. wavedrom::
        :caption: GENERAL_PERIPH_PERIPH_INPUT_RD_1

        {
            "reg": [
                {"name": "periph_input_rd_1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x18 = 0xf0001018`


    .. wavedrom::
        :caption: GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_0

        {
            "reg": [
                {"name": "periph_output_ovrd_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_PERIPH_OUTPUT_VAL_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x1c = 0xf000101c`


    .. wavedrom::
        :caption: GENERAL_PERIPH_PERIPH_OUTPUT_VAL_0

        {
            "reg": [
                {"name": "periph_output_val_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x20 = 0xf0001020`


    .. wavedrom::
        :caption: GENERAL_PERIPH_PERIPH_OUTPUT_OVRD_1

        {
            "reg": [
                {"name": "periph_output_ovrd_1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_PERIPH_OUTPUT_VAL_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x24 = 0xf0001024`


    .. wavedrom::
        :caption: GENERAL_PERIPH_PERIPH_OUTPUT_VAL_1

        {
            "reg": [
                {"name": "periph_output_val_1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GENERAL_PERIPH_FPGA_LED_CTRL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x28 = 0xf0001028`


    .. wavedrom::
        :caption: GENERAL_PERIPH_FPGA_LED_CTRL

        {
            "reg": [
                {"name": "LED1_CTRL",  "bits": 3},
                {"bits": 1},
                {"name": "LED2_CTRL",  "bits": 3},
                {"bits": 25}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-----------+-------------+
| Field | Name      | Description |
+=======+===========+=============+
+-------+-----------+-------------+
+-------+-----------+-------------+

GENERAL_PERIPH_FX3_LED_CTRL
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0001000 + 0x2c = 0xf000102c`


    .. wavedrom::
        :caption: GENERAL_PERIPH_FX3_LED_CTRL

        {
            "reg": [
                {"name": "fx3_led_ctrl[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


