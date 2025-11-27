FPGACFG
=======

Register Listing for FPGACFG
----------------------------

+------------------------------------------------------+-------------------------------------------+
| Register                                             | Address                                   |
+======================================================+===========================================+
| :ref:`FPGACFG_BOARD_ID <FPGACFG_BOARD_ID>`           | :ref:`0xf0003800 <FPGACFG_BOARD_ID>`      |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_RESERVED_03 <FPGACFG_RESERVED_03>`     | :ref:`0xf0003804 <FPGACFG_RESERVED_03>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_RESERVED_04 <FPGACFG_RESERVED_04>`     | :ref:`0xf0003808 <FPGACFG_RESERVED_04>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_RESERVED_05 <FPGACFG_RESERVED_05>`     | :ref:`0xf000380c <FPGACFG_RESERVED_05>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_RESERVED_06 <FPGACFG_RESERVED_06>`     | :ref:`0xf0003810 <FPGACFG_RESERVED_06>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_MAJOR_REV <FPGACFG_MAJOR_REV>`         | :ref:`0xf0003814 <FPGACFG_MAJOR_REV>`     |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_COMPILE_REV <FPGACFG_COMPILE_REV>`     | :ref:`0xf0003818 <FPGACFG_COMPILE_REV>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_CHANNEL_CNTRL <FPGACFG_CHANNEL_CNTRL>` | :ref:`0xf000381c <FPGACFG_CHANNEL_CNTRL>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_TCXO_EN <FPGACFG_TCXO_EN>`             | :ref:`0xf0003820 <FPGACFG_TCXO_EN>`       |
+------------------------------------------------------+-------------------------------------------+
| :ref:`FPGACFG_EXT_CLK <FPGACFG_EXT_CLK>`             | :ref:`0xf0003824 <FPGACFG_EXT_CLK>`       |
+------------------------------------------------------+-------------------------------------------+

FPGACFG_BOARD_ID
^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x0 = 0xf0003800`


    .. wavedrom::
        :caption: FPGACFG_BOARD_ID

        {
            "reg": [
                {"name": "board_id[15:0]", "attr": 'reset: 27', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_RESERVED_03
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x4 = 0xf0003804`


    .. wavedrom::
        :caption: FPGACFG_RESERVED_03

        {
            "reg": [
                {"name": "reserved_03[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_RESERVED_04
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x8 = 0xf0003808`


    .. wavedrom::
        :caption: FPGACFG_RESERVED_04

        {
            "reg": [
                {"name": "reserved_04[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_RESERVED_05
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0xc = 0xf000380c`


    .. wavedrom::
        :caption: FPGACFG_RESERVED_05

        {
            "reg": [
                {"name": "reserved_05[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_RESERVED_06
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x10 = 0xf0003810`


    .. wavedrom::
        :caption: FPGACFG_RESERVED_06

        {
            "reg": [
                {"name": "reserved_06[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_MAJOR_REV
^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x14 = 0xf0003814`


    .. wavedrom::
        :caption: FPGACFG_MAJOR_REV

        {
            "reg": [
                {"name": "major_rev[15:0]", "attr": 'reset: 2', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_COMPILE_REV
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x18 = 0xf0003818`


    .. wavedrom::
        :caption: FPGACFG_COMPILE_REV

        {
            "reg": [
                {"name": "compile_rev[15:0]", "attr": 'reset: 26', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FPGACFG_CHANNEL_CNTRL
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x1c = 0xf000381c`


    .. wavedrom::
        :caption: FPGACFG_CHANNEL_CNTRL

        {
            "reg": [
                {"name": "ch_en",  "bits": 2},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-------+-------------------------------+
| Field | Name  | Description                   |
+=======+=======+===============================+
| [1:0] | CH_EN |                               |
|       |       |                               |
|       |       | +--------+------------------+ |
|       |       | | Value  | Description      | |
|       |       | +========+==================+ |
|       |       | | ``2b01 | Channel A        | |
|       |       | +--------+------------------+ |
|       |       | | ``2b10 | Channel B        | |
|       |       | +--------+------------------+ |
|       |       | | ``2b11 | Channels A and B | |
|       |       | +--------+------------------+ |
+-------+-------+-------------------------------+

FPGACFG_TCXO_EN
^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x20 = 0xf0003820`

    TCXO Enable: 0: Disabled, 1: Enabled.

    .. wavedrom::
        :caption: FPGACFG_TCXO_EN

        {
            "reg": [
                {"name": "tcxo_en", "attr": 'reset: 1', "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


FPGACFG_EXT_CLK
^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x24 = 0xf0003824`

    CLK source select: 0: Internal, 1: External.

    .. wavedrom::
        :caption: FPGACFG_EXT_CLK

        {
            "reg": [
                {"name": "ext_clk", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


