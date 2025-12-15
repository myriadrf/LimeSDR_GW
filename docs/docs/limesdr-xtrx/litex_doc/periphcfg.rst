PERIPHCFG
=========

Register Listing for PERIPHCFG
------------------------------

+------------------------------------------------------------------------+----------------------------------------------------+
| Register                                                               | Address                                            |
+========================================================================+====================================================+
| :ref:`PERIPHCFG_BOARD_GPIO_OVRD <PERIPHCFG_BOARD_GPIO_OVRD>`           | :ref:`0xf000a800 <PERIPHCFG_BOARD_GPIO_OVRD>`      |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_BOARD_GPIO_RD <PERIPHCFG_BOARD_GPIO_RD>`               | :ref:`0xf000a804 <PERIPHCFG_BOARD_GPIO_RD>`        |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_BOARD_GPIO_DIR <PERIPHCFG_BOARD_GPIO_DIR>`             | :ref:`0xf000a808 <PERIPHCFG_BOARD_GPIO_DIR>`       |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_BOARD_GPIO_VAL <PERIPHCFG_BOARD_GPIO_VAL>`             | :ref:`0xf000a80c <PERIPHCFG_BOARD_GPIO_VAL>`       |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_INPUT_SEL_0 <PERIPHCFG_PERIPH_INPUT_SEL_0>`     | :ref:`0xf000a810 <PERIPHCFG_PERIPH_INPUT_SEL_0>`   |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_INPUT_RD_0 <PERIPHCFG_PERIPH_INPUT_RD_0>`       | :ref:`0xf000a814 <PERIPHCFG_PERIPH_INPUT_RD_0>`    |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_INPUT_RD_1 <PERIPHCFG_PERIPH_INPUT_RD_1>`       | :ref:`0xf000a818 <PERIPHCFG_PERIPH_INPUT_RD_1>`    |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_OVRD_0 <PERIPHCFG_PERIPH_OUTPUT_OVRD_0>` | :ref:`0xf000a81c <PERIPHCFG_PERIPH_OUTPUT_OVRD_0>` |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_VAL_0 <PERIPHCFG_PERIPH_OUTPUT_VAL_0>`   | :ref:`0xf000a820 <PERIPHCFG_PERIPH_OUTPUT_VAL_0>`  |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_OVRD_1 <PERIPHCFG_PERIPH_OUTPUT_OVRD_1>` | :ref:`0xf000a824 <PERIPHCFG_PERIPH_OUTPUT_OVRD_1>` |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_VAL_1 <PERIPHCFG_PERIPH_OUTPUT_VAL_1>`   | :ref:`0xf000a828 <PERIPHCFG_PERIPH_OUTPUT_VAL_1>`  |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_EN <PERIPHCFG_PERIPH_EN>`                       | :ref:`0xf000a82c <PERIPHCFG_PERIPH_EN>`            |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_SEL <PERIPHCFG_PERIPH_SEL>`                     | :ref:`0xf000a830 <PERIPHCFG_PERIPH_SEL>`           |
+------------------------------------------------------------------------+----------------------------------------------------+

PERIPHCFG_BOARD_GPIO_OVRD
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x0 = 0xf000a800`


    .. wavedrom::
        :caption: PERIPHCFG_BOARD_GPIO_OVRD

        {
            "reg": [
                {"name": "board_gpio_ovrd[15:0]", "attr": 'reset: 2', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_BOARD_GPIO_RD
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x4 = 0xf000a804`


    .. wavedrom::
        :caption: PERIPHCFG_BOARD_GPIO_RD

        {
            "reg": [
                {"name": "board_gpio_rd[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_BOARD_GPIO_DIR
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x8 = 0xf000a808`


    .. wavedrom::
        :caption: PERIPHCFG_BOARD_GPIO_DIR

        {
            "reg": [
                {"name": "board_gpio_dir[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_BOARD_GPIO_VAL
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0xc = 0xf000a80c`


    .. wavedrom::
        :caption: PERIPHCFG_BOARD_GPIO_VAL

        {
            "reg": [
                {"name": "board_gpio_val[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_INPUT_SEL_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x10 = 0xf000a810`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_INPUT_SEL_0

        {
            "reg": [
                {"name": "periph_input_sel_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_INPUT_RD_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x14 = 0xf000a814`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_INPUT_RD_0

        {
            "reg": [
                {"name": "periph_input_rd_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_INPUT_RD_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x18 = 0xf000a818`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_INPUT_RD_1

        {
            "reg": [
                {"name": "periph_input_rd_1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_OUTPUT_OVRD_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x1c = 0xf000a81c`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_OUTPUT_OVRD_0

        {
            "reg": [
                {"name": "periph_output_ovrd_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_OUTPUT_VAL_0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x20 = 0xf000a820`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_OUTPUT_VAL_0

        {
            "reg": [
                {"name": "periph_output_val_0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_OUTPUT_OVRD_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x24 = 0xf000a824`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_OUTPUT_OVRD_1

        {
            "reg": [
                {"name": "periph_output_ovrd_1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_OUTPUT_VAL_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x28 = 0xf000a828`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_OUTPUT_VAL_1

        {
            "reg": [
                {"name": "periph_output_val_1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_EN
^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x2c = 0xf000a82c`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_EN

        {
            "reg": [
                {"name": "periph_en[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_PERIPH_SEL
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x30 = 0xf000a830`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_SEL

        {
            "reg": [
                {"name": "periph_sel[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


