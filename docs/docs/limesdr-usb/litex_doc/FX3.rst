FX3
===

Register Listing for FX3
------------------------

+--------------------------------------------+--------------------------------------+
| Register                                   | Address                              |
+============================================+======================================+
| :ref:`FX3_FIFO_WDATA <FX3_FIFO_WDATA>`     | :ref:`0xf0000000 <FX3_FIFO_WDATA>`   |
+--------------------------------------------+--------------------------------------+
| :ref:`FX3_FIFO_RDATA <FX3_FIFO_RDATA>`     | :ref:`0xf0000004 <FX3_FIFO_RDATA>`   |
+--------------------------------------------+--------------------------------------+
| :ref:`FX3_FIFO_STATUS <FX3_FIFO_STATUS>`   | :ref:`0xf0000008 <FX3_FIFO_STATUS>`  |
+--------------------------------------------+--------------------------------------+
| :ref:`FX3_FIFO_CONTROL <FX3_FIFO_CONTROL>` | :ref:`0xf000000c <FX3_FIFO_CONTROL>` |
+--------------------------------------------+--------------------------------------+

FX3_FIFO_WDATA
^^^^^^^^^^^^^^

`Address: 0xf0000000 + 0x0 = 0xf0000000`

    FIFO Write Register.

    .. wavedrom::
        :caption: FX3_FIFO_WDATA

        {
            "reg": [
                {"name": "fifo_wdata[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FX3_FIFO_RDATA
^^^^^^^^^^^^^^

`Address: 0xf0000000 + 0x4 = 0xf0000004`

    FIFO Read Register.

    .. wavedrom::
        :caption: FX3_FIFO_RDATA

        {
            "reg": [
                {"name": "fifo_rdata[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


FX3_FIFO_STATUS
^^^^^^^^^^^^^^^

`Address: 0xf0000000 + 0x8 = 0xf0000008`

    FIFO Status Register.

    .. wavedrom::
        :caption: FX3_FIFO_STATUS

        {
            "reg": [
                {"name": "is_rdempty",  "bits": 1},
                {"name": "is_wrfull",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------------+---------------------+
| Field | Name       | Description         |
+=======+============+=====================+
| [0]   | IS_RDEMPTY | Read FIFO is empty. |
+-------+------------+---------------------+
| [1]   | IS_WRFULL  | Write FIFO is full. |
+-------+------------+---------------------+

FX3_FIFO_CONTROL
^^^^^^^^^^^^^^^^

`Address: 0xf0000000 + 0xc = 0xf000000c`

    FIFO Control Register.

    .. wavedrom::
        :caption: FX3_FIFO_CONTROL

        {
            "reg": [
                {"name": "reset",  "bits": 1},
                {"bits": 31}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-------+------------------------------+
| Field | Name  | Description                  |
+=======+=======+==============================+
| [0]   | RESET | Reset Control (Active High). |
+-------+-------+------------------------------+

