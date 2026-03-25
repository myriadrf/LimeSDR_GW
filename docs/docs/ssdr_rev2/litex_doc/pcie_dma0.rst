PCIE_DMA0
=========

Register Listing for PCIE_DMA0
------------------------------

+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| Register                                                                                 | Address                                                     |
+==========================================================================================+=============================================================+
| :ref:`PCIE_DMA0_WRITER_ENABLE <PCIE_DMA0_WRITER_ENABLE>`                                 | :ref:`0xf0002800 <PCIE_DMA0_WRITER_ENABLE>`                 |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_VALUE1 <PCIE_DMA0_WRITER_TABLE_VALUE1>`                     | :ref:`0xf0002804 <PCIE_DMA0_WRITER_TABLE_VALUE1>`           |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_VALUE0 <PCIE_DMA0_WRITER_TABLE_VALUE0>`                     | :ref:`0xf0002808 <PCIE_DMA0_WRITER_TABLE_VALUE0>`           |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_WE <PCIE_DMA0_WRITER_TABLE_WE>`                             | :ref:`0xf000280c <PCIE_DMA0_WRITER_TABLE_WE>`               |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N <PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N>`           | :ref:`0xf0002810 <PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N>`      |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_LOOP_STATUS <PCIE_DMA0_WRITER_TABLE_LOOP_STATUS>`           | :ref:`0xf0002814 <PCIE_DMA0_WRITER_TABLE_LOOP_STATUS>`      |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_LEVEL <PCIE_DMA0_WRITER_TABLE_LEVEL>`                       | :ref:`0xf0002818 <PCIE_DMA0_WRITER_TABLE_LEVEL>`            |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_WRITER_TABLE_RESET <PCIE_DMA0_WRITER_TABLE_RESET>`                       | :ref:`0xf000281c <PCIE_DMA0_WRITER_TABLE_RESET>`            |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_ENABLE <PCIE_DMA0_READER_ENABLE>`                                 | :ref:`0xf0002820 <PCIE_DMA0_READER_ENABLE>`                 |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_VALUE1 <PCIE_DMA0_READER_TABLE_VALUE1>`                     | :ref:`0xf0002824 <PCIE_DMA0_READER_TABLE_VALUE1>`           |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_VALUE0 <PCIE_DMA0_READER_TABLE_VALUE0>`                     | :ref:`0xf0002828 <PCIE_DMA0_READER_TABLE_VALUE0>`           |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_WE <PCIE_DMA0_READER_TABLE_WE>`                             | :ref:`0xf000282c <PCIE_DMA0_READER_TABLE_WE>`               |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_LOOP_PROG_N <PCIE_DMA0_READER_TABLE_LOOP_PROG_N>`           | :ref:`0xf0002830 <PCIE_DMA0_READER_TABLE_LOOP_PROG_N>`      |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_LOOP_STATUS <PCIE_DMA0_READER_TABLE_LOOP_STATUS>`           | :ref:`0xf0002834 <PCIE_DMA0_READER_TABLE_LOOP_STATUS>`      |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_LEVEL <PCIE_DMA0_READER_TABLE_LEVEL>`                       | :ref:`0xf0002838 <PCIE_DMA0_READER_TABLE_LEVEL>`            |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_READER_TABLE_RESET <PCIE_DMA0_READER_TABLE_RESET>`                       | :ref:`0xf000283c <PCIE_DMA0_READER_TABLE_RESET>`            |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL <PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL>` | :ref:`0xf0002840 <PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL>` |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_BUFFERING_READER_FIFO_STATUS <PCIE_DMA0_BUFFERING_READER_FIFO_STATUS>`   | :ref:`0xf0002844 <PCIE_DMA0_BUFFERING_READER_FIFO_STATUS>`  |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL <PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL>` | :ref:`0xf0002848 <PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL>` |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+
| :ref:`PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS <PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS>`   | :ref:`0xf000284c <PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS>`  |
+------------------------------------------------------------------------------------------+-------------------------------------------------------------+

PCIE_DMA0_WRITER_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x0 = 0xf0002800`

    DMA Writer Control. Write ``1`` to enable DMA Writer.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_ENABLE

        {
            "reg": [
                {"name": "writer_enable[1:0]", "bits": 2},
                {"bits": 30},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_DMA0_WRITER_TABLE_VALUE1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x4 = 0xf0002804`

    Bits 32-57 of `PCIE_DMA0_WRITER_TABLE_VALUE`. 64-bit DMA descriptor to be
    written to the table.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_VALUE1

        {
            "reg": [
                {"name": "length",  "bits": 24},
                {"name": "irq_disable",  "bits": 1},
                {"name": "last_disable",  "bits": 1},
                {"bits": 6}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+--------+--------------+----------------------------------------------+
| Field  | Name         | Description                                  |
+========+==============+==============================================+
| [23:0] | LENGTH       | 24-bit Length  of the descriptor (in bytes). |
+--------+--------------+----------------------------------------------+
| [24]   | IRQ_DISABLE  | IRQ Disable Control of the descriptor.       |
+--------+--------------+----------------------------------------------+
| [25]   | LAST_DISABLE | Last Disable Control of the descriptor.      |
+--------+--------------+----------------------------------------------+

PCIE_DMA0_WRITER_TABLE_VALUE0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x8 = 0xf0002808`

    Bits 0-31 of `PCIE_DMA0_WRITER_TABLE_VALUE`.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_VALUE0

        {
            "reg": [
                {"name": "address_lsb",  "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-------------+-------------------------------------------------------+
| Field  | Name        | Description                                           |
+========+=============+=======================================================+
| [31:0] | ADDRESS_LSB | 32-bit LSB Address of the descriptor (bytes-aligned). |
+--------+-------------+-------------------------------------------------------+

PCIE_DMA0_WRITER_TABLE_WE
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0xc = 0xf000280c`

    Write and 32-bit MSB Address of the descriptor (bytes-aligned)

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_WE

        {
            "reg": [
                {"name": "address_msb",  "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-------------+-----------------------------------------------------------------------+
| Field  | Name        | Description                                                           |
+========+=============+=======================================================================+
| [31:0] | ADDRESS_MSB | 32-bit MSB Address of the descriptor (bytes-aligned), in 64-bit mode. |
+--------+-------------+-----------------------------------------------------------------------+

PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x10 = 0xf0002810`

    Mode Selection.

    ``0``: **Prog** mode / ``1``: **Loop** mode.

    **Prog** mode should be used to program the table by software and for cases
    where automatic refill of the table is not needed: A descriptor is only executed
    once and when all the descriptors have been executed (ie the table is empty),
    the DMA just stops until the next software refill.

    **Loop** mode should be used once the table has been filled by software in
    **Prog** mode and allow continuous Scatter-Gather DMA: Each descriptor sent to
    the DMA is refilled to the table.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N

        {
            "reg": [
                {"name": "writer_table_loop_prog_n", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_DMA0_WRITER_TABLE_LOOP_STATUS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x14 = 0xf0002814`

    Loop monitoring for software synchronization.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_LOOP_STATUS

        {
            "reg": [
                {"name": "index",  "bits": 16},
                {"name": "count",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+-------+--------------------------------------------------------------------+
| Field   | Name  | Description                                                        |
+=========+=======+====================================================================+
| [15:0]  | INDEX | Index of the last descriptor executed in the DMA descriptor table. |
+---------+-------+--------------------------------------------------------------------+
| [31:16] | COUNT | Loops of the DMA descriptor table since started.                   |
+---------+-------+--------------------------------------------------------------------+

PCIE_DMA0_WRITER_TABLE_LEVEL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x18 = 0xf0002818`

    Number descriptors in the table.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_LEVEL

        {
            "reg": [
                {"name": "writer_table_level[8:0]", "bits": 9},
                {"bits": 23},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_DMA0_WRITER_TABLE_RESET
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x1c = 0xf000281c`

    A write to this register resets the table.

    .. wavedrom::
        :caption: PCIE_DMA0_WRITER_TABLE_RESET

        {
            "reg": [
                {"name": "writer_table_reset", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_DMA0_READER_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x20 = 0xf0002820`

    DMA Reader Control. Write ``1`` to enable DMA Reader.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_ENABLE

        {
            "reg": [
                {"name": "reader_enable[1:0]", "bits": 2},
                {"bits": 30},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_DMA0_READER_TABLE_VALUE1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x24 = 0xf0002824`

    Bits 32-57 of `PCIE_DMA0_READER_TABLE_VALUE`. 64-bit DMA descriptor to be
    written to the table.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_VALUE1

        {
            "reg": [
                {"name": "length",  "bits": 24},
                {"name": "irq_disable",  "bits": 1},
                {"name": "last_disable",  "bits": 1},
                {"bits": 6}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+--------+--------------+----------------------------------------------+
| Field  | Name         | Description                                  |
+========+==============+==============================================+
| [23:0] | LENGTH       | 24-bit Length  of the descriptor (in bytes). |
+--------+--------------+----------------------------------------------+
| [24]   | IRQ_DISABLE  | IRQ Disable Control of the descriptor.       |
+--------+--------------+----------------------------------------------+
| [25]   | LAST_DISABLE | Last Disable Control of the descriptor.      |
+--------+--------------+----------------------------------------------+

PCIE_DMA0_READER_TABLE_VALUE0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x28 = 0xf0002828`

    Bits 0-31 of `PCIE_DMA0_READER_TABLE_VALUE`.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_VALUE0

        {
            "reg": [
                {"name": "address_lsb",  "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-------------+-------------------------------------------------------+
| Field  | Name        | Description                                           |
+========+=============+=======================================================+
| [31:0] | ADDRESS_LSB | 32-bit LSB Address of the descriptor (bytes-aligned). |
+--------+-------------+-------------------------------------------------------+

PCIE_DMA0_READER_TABLE_WE
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x2c = 0xf000282c`

    Write and 32-bit MSB Address of the descriptor (bytes-aligned)

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_WE

        {
            "reg": [
                {"name": "address_msb",  "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-------------+-----------------------------------------------------------------------+
| Field  | Name        | Description                                                           |
+========+=============+=======================================================================+
| [31:0] | ADDRESS_MSB | 32-bit MSB Address of the descriptor (bytes-aligned), in 64-bit mode. |
+--------+-------------+-----------------------------------------------------------------------+

PCIE_DMA0_READER_TABLE_LOOP_PROG_N
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x30 = 0xf0002830`

    Mode Selection.

    ``0``: **Prog** mode / ``1``: **Loop** mode.

    **Prog** mode should be used to program the table by software and for cases
    where automatic refill of the table is not needed: A descriptor is only executed
    once and when all the descriptors have been executed (ie the table is empty),
    the DMA just stops until the next software refill.

    **Loop** mode should be used once the table has been filled by software in
    **Prog** mode and allow continuous Scatter-Gather DMA: Each descriptor sent to
    the DMA is refilled to the table.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_LOOP_PROG_N

        {
            "reg": [
                {"name": "reader_table_loop_prog_n", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_DMA0_READER_TABLE_LOOP_STATUS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x34 = 0xf0002834`

    Loop monitoring for software synchronization.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_LOOP_STATUS

        {
            "reg": [
                {"name": "index",  "bits": 16},
                {"name": "count",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+-------+--------------------------------------------------------------------+
| Field   | Name  | Description                                                        |
+=========+=======+====================================================================+
| [15:0]  | INDEX | Index of the last descriptor executed in the DMA descriptor table. |
+---------+-------+--------------------------------------------------------------------+
| [31:16] | COUNT | Loops of the DMA descriptor table since started.                   |
+---------+-------+--------------------------------------------------------------------+

PCIE_DMA0_READER_TABLE_LEVEL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x38 = 0xf0002838`

    Number descriptors in the table.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_LEVEL

        {
            "reg": [
                {"name": "reader_table_level[8:0]", "bits": 9},
                {"bits": 23},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_DMA0_READER_TABLE_RESET
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x3c = 0xf000283c`

    A write to this register resets the table.

    .. wavedrom::
        :caption: PCIE_DMA0_READER_TABLE_RESET

        {
            "reg": [
                {"name": "reader_table_reset", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x40 = 0xf0002840`


    .. wavedrom::
        :caption: PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL

        {
            "reg": [
                {"name": "depth",  "attr": '8192', "bits": 24},
                {"name": "scratch",  "bits": 4},
                {"bits": 3},
                {"name": "level_mode",  "bits": 1}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+---------+------------+--------------------------------------------------------+
| Field   | Name       | Description                                            |
+=========+============+========================================================+
| [23:0]  | DEPTH      | DMA Reader FIFO depth (in 64-bit words).               |
+---------+------------+--------------------------------------------------------+
| [27:24] | SCRATCH    | Software Scratchpad.                                   |
+---------+------------+--------------------------------------------------------+
| [31]    | LEVEL_MODE |                                                        |
|         |            |                                                        |
|         |            | +---------+------------------------------------------+ |
|         |            | | Value   | Description                              | |
|         |            | +=========+==========================================+ |
|         |            | | ``0b0`` | Report Instantaneous level.              | |
|         |            | +---------+------------------------------------------+ |
|         |            | | ``0b1`` | Report `Minimal` level since last clear. | |
|         |            | +---------+------------------------------------------+ |
+---------+------------+--------------------------------------------------------+

PCIE_DMA0_BUFFERING_READER_FIFO_STATUS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x44 = 0xf0002844`


    .. wavedrom::
        :caption: PCIE_DMA0_BUFFERING_READER_FIFO_STATUS

        {
            "reg": [
                {"name": "level",  "bits": 24},
                {"bits": 8}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-------+------------------------------------------+
| Field  | Name  | Description                              |
+========+=======+==========================================+
| [23:0] | LEVEL | DMA Reader FIFO level (in 64-bit words). |
+--------+-------+------------------------------------------+

PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x48 = 0xf0002848`


    .. wavedrom::
        :caption: PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL

        {
            "reg": [
                {"name": "depth",  "attr": '8192', "bits": 24},
                {"name": "scratch",  "bits": 4},
                {"bits": 3},
                {"name": "level_mode",  "bits": 1}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+---------+------------+--------------------------------------------------------+
| Field   | Name       | Description                                            |
+=========+============+========================================================+
| [23:0]  | DEPTH      | DMA Writer FIFO depth (in 64-bit words).               |
+---------+------------+--------------------------------------------------------+
| [27:24] | SCRATCH    | Software Scratchpad.                                   |
+---------+------------+--------------------------------------------------------+
| [31]    | LEVEL_MODE |                                                        |
|         |            |                                                        |
|         |            | +---------+------------------------------------------+ |
|         |            | | Value   | Description                              | |
|         |            | +=========+==========================================+ |
|         |            | | ``0b0`` | Report Instantaneous level.              | |
|         |            | +---------+------------------------------------------+ |
|         |            | | ``0b1`` | Report `Maximal` level since last clear. | |
|         |            | +---------+------------------------------------------+ |
+---------+------------+--------------------------------------------------------+

PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002800 + 0x4c = 0xf000284c`


    .. wavedrom::
        :caption: PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS

        {
            "reg": [
                {"name": "level",  "bits": 24},
                {"bits": 8}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+-------+------------------------------------------+
| Field  | Name  | Description                              |
+========+=======+==========================================+
| [23:0] | LEVEL | DMA Writer FIFO level (in 64-bit words). |
+--------+-------+------------------------------------------+

