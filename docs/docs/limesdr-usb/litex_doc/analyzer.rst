ANALYZER
========

Register Listing for ANALYZER
-----------------------------

+----------------------------------------------------------------+------------------------------------------------+
| Register                                                       | Address                                        |
+================================================================+================================================+
| :ref:`ANALYZER_MUX_VALUE <ANALYZER_MUX_VALUE>`                 | :ref:`0xf0000800 <ANALYZER_MUX_VALUE>`         |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_TRIGGER_ENABLE <ANALYZER_TRIGGER_ENABLE>`       | :ref:`0xf0000804 <ANALYZER_TRIGGER_ENABLE>`    |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_TRIGGER_DONE <ANALYZER_TRIGGER_DONE>`           | :ref:`0xf0000808 <ANALYZER_TRIGGER_DONE>`      |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_TRIGGER_MEM_WRITE <ANALYZER_TRIGGER_MEM_WRITE>` | :ref:`0xf000080c <ANALYZER_TRIGGER_MEM_WRITE>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_TRIGGER_MEM_MASK <ANALYZER_TRIGGER_MEM_MASK>`   | :ref:`0xf0000810 <ANALYZER_TRIGGER_MEM_MASK>`  |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_TRIGGER_MEM_VALUE <ANALYZER_TRIGGER_MEM_VALUE>` | :ref:`0xf0000814 <ANALYZER_TRIGGER_MEM_VALUE>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_TRIGGER_MEM_FULL <ANALYZER_TRIGGER_MEM_FULL>`   | :ref:`0xf0000818 <ANALYZER_TRIGGER_MEM_FULL>`  |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_SUBSAMPLER_VALUE <ANALYZER_SUBSAMPLER_VALUE>`   | :ref:`0xf000081c <ANALYZER_SUBSAMPLER_VALUE>`  |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_STORAGE_ENABLE <ANALYZER_STORAGE_ENABLE>`       | :ref:`0xf0000820 <ANALYZER_STORAGE_ENABLE>`    |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_STORAGE_DONE <ANALYZER_STORAGE_DONE>`           | :ref:`0xf0000824 <ANALYZER_STORAGE_DONE>`      |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_STORAGE_LENGTH <ANALYZER_STORAGE_LENGTH>`       | :ref:`0xf0000828 <ANALYZER_STORAGE_LENGTH>`    |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_STORAGE_OFFSET <ANALYZER_STORAGE_OFFSET>`       | :ref:`0xf000082c <ANALYZER_STORAGE_OFFSET>`    |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_STORAGE_MEM_LEVEL <ANALYZER_STORAGE_MEM_LEVEL>` | :ref:`0xf0000830 <ANALYZER_STORAGE_MEM_LEVEL>` |
+----------------------------------------------------------------+------------------------------------------------+
| :ref:`ANALYZER_STORAGE_MEM_DATA <ANALYZER_STORAGE_MEM_DATA>`   | :ref:`0xf0000834 <ANALYZER_STORAGE_MEM_DATA>`  |
+----------------------------------------------------------------+------------------------------------------------+

ANALYZER_MUX_VALUE
^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x0 = 0xf0000800`


    .. wavedrom::
        :caption: ANALYZER_MUX_VALUE

        {
            "reg": [
                {"name": "mux_value", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_TRIGGER_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x4 = 0xf0000804`


    .. wavedrom::
        :caption: ANALYZER_TRIGGER_ENABLE

        {
            "reg": [
                {"name": "trigger_enable", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_TRIGGER_DONE
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x8 = 0xf0000808`


    .. wavedrom::
        :caption: ANALYZER_TRIGGER_DONE

        {
            "reg": [
                {"name": "trigger_done", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_TRIGGER_MEM_WRITE
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0xc = 0xf000080c`


    .. wavedrom::
        :caption: ANALYZER_TRIGGER_MEM_WRITE

        {
            "reg": [
                {"name": "trigger_mem_write", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_TRIGGER_MEM_MASK
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x10 = 0xf0000810`


    .. wavedrom::
        :caption: ANALYZER_TRIGGER_MEM_MASK

        {
            "reg": [
                {"name": "trigger_mem_mask[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_TRIGGER_MEM_VALUE
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x14 = 0xf0000814`


    .. wavedrom::
        :caption: ANALYZER_TRIGGER_MEM_VALUE

        {
            "reg": [
                {"name": "trigger_mem_value[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_TRIGGER_MEM_FULL
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x18 = 0xf0000818`


    .. wavedrom::
        :caption: ANALYZER_TRIGGER_MEM_FULL

        {
            "reg": [
                {"name": "trigger_mem_full", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_SUBSAMPLER_VALUE
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x1c = 0xf000081c`


    .. wavedrom::
        :caption: ANALYZER_SUBSAMPLER_VALUE

        {
            "reg": [
                {"name": "subsampler_value[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


ANALYZER_STORAGE_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x20 = 0xf0000820`


    .. wavedrom::
        :caption: ANALYZER_STORAGE_ENABLE

        {
            "reg": [
                {"name": "storage_enable", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_STORAGE_DONE
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x24 = 0xf0000824`


    .. wavedrom::
        :caption: ANALYZER_STORAGE_DONE

        {
            "reg": [
                {"name": "storage_done", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ANALYZER_STORAGE_LENGTH
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x28 = 0xf0000828`


    .. wavedrom::
        :caption: ANALYZER_STORAGE_LENGTH

        {
            "reg": [
                {"name": "storage_length[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


ANALYZER_STORAGE_OFFSET
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x2c = 0xf000082c`


    .. wavedrom::
        :caption: ANALYZER_STORAGE_OFFSET

        {
            "reg": [
                {"name": "storage_offset[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


ANALYZER_STORAGE_MEM_LEVEL
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x30 = 0xf0000830`


    .. wavedrom::
        :caption: ANALYZER_STORAGE_MEM_LEVEL

        {
            "reg": [
                {"name": "storage_mem_level[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


ANALYZER_STORAGE_MEM_DATA
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0000800 + 0x34 = 0xf0000834`


    .. wavedrom::
        :caption: ANALYZER_STORAGE_MEM_DATA

        {
            "reg": [
                {"name": "storage_mem_data[3:0]", "bits": 4},
                {"bits": 28},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


