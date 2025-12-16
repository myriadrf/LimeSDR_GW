PERIPHCFG
=========

Register Listing for PERIPHCFG
------------------------------

+------------------------------------------------------------------------+----------------------------------------------------+
| Register                                                               | Address                                            |
+========================================================================+====================================================+
| :ref:`PERIPHCFG_BOARD_GPIO_OVRD <PERIPHCFG_BOARD_GPIO_OVRD>`           | :ref:`0xf000c000 <PERIPHCFG_BOARD_GPIO_OVRD>`      |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_BOARD_GPIO_RD <PERIPHCFG_BOARD_GPIO_RD>`               | :ref:`0xf000c004 <PERIPHCFG_BOARD_GPIO_RD>`        |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_BOARD_GPIO_DIR <PERIPHCFG_BOARD_GPIO_DIR>`             | :ref:`0xf000c008 <PERIPHCFG_BOARD_GPIO_DIR>`       |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_BOARD_GPIO_VAL <PERIPHCFG_BOARD_GPIO_VAL>`             | :ref:`0xf000c00c <PERIPHCFG_BOARD_GPIO_VAL>`       |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_INPUT_SEL_0 <PERIPHCFG_PERIPH_INPUT_SEL_0>`     | :ref:`0xf000c010 <PERIPHCFG_PERIPH_INPUT_SEL_0>`   |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_INPUT_RD_0 <PERIPHCFG_PERIPH_INPUT_RD_0>`       | :ref:`0xf000c014 <PERIPHCFG_PERIPH_INPUT_RD_0>`    |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_INPUT_RD_1 <PERIPHCFG_PERIPH_INPUT_RD_1>`       | :ref:`0xf000c018 <PERIPHCFG_PERIPH_INPUT_RD_1>`    |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_OVRD_0 <PERIPHCFG_PERIPH_OUTPUT_OVRD_0>` | :ref:`0xf000c01c <PERIPHCFG_PERIPH_OUTPUT_OVRD_0>` |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_VAL_0 <PERIPHCFG_PERIPH_OUTPUT_VAL_0>`   | :ref:`0xf000c020 <PERIPHCFG_PERIPH_OUTPUT_VAL_0>`  |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_OVRD_1 <PERIPHCFG_PERIPH_OUTPUT_OVRD_1>` | :ref:`0xf000c024 <PERIPHCFG_PERIPH_OUTPUT_OVRD_1>` |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_OUTPUT_VAL_1 <PERIPHCFG_PERIPH_OUTPUT_VAL_1>`   | :ref:`0xf000c028 <PERIPHCFG_PERIPH_OUTPUT_VAL_1>`  |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_EN <PERIPHCFG_PERIPH_EN>`                       | :ref:`0xf000c02c <PERIPHCFG_PERIPH_EN>`            |
+------------------------------------------------------------------------+----------------------------------------------------+
| :ref:`PERIPHCFG_PERIPH_SEL <PERIPHCFG_PERIPH_SEL>`                     | :ref:`0xf000c030 <PERIPHCFG_PERIPH_SEL>`           |
+------------------------------------------------------------------------+----------------------------------------------------+

PERIPHCFG_BOARD_GPIO_OVRD
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000c000 + 0x0 = 0xf000c000`


    .. wavedrom::
        :caption: PERIPHCFG_BOARD_GPIO_OVRD

        {
            "reg": [
                {"name": "board_gpio_ovrd[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PERIPHCFG_BOARD_GPIO_RD
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000c000 + 0x4 = 0xf000c004`


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

`Address: 0xf000c000 + 0x8 = 0xf000c008`


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

`Address: 0xf000c000 + 0xc = 0xf000c00c`


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

`Address: 0xf000c000 + 0x10 = 0xf000c010`


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

`Address: 0xf000c000 + 0x14 = 0xf000c014`


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

`Address: 0xf000c000 + 0x18 = 0xf000c018`


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

`Address: 0xf000c000 + 0x1c = 0xf000c01c`


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

`Address: 0xf000c000 + 0x20 = 0xf000c020`


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

`Address: 0xf000c000 + 0x24 = 0xf000c024`


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

`Address: 0xf000c000 + 0x28 = 0xf000c028`


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

`Address: 0xf000c000 + 0x2c = 0xf000c02c`


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

`Address: 0xf000c000 + 0x30 = 0xf000c030`


    .. wavedrom::
        :caption: PERIPHCFG_PERIPH_SEL

        {
            "reg": [
                {"name": "periph_sel[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


