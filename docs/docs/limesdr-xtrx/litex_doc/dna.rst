DNA
===

Register Listing for DNA
------------------------

+--------------------------+-----------------------------+
| Register                 | Address                     |
+==========================+=============================+
| :ref:`DNA_ID1 <DNA_ID1>` | :ref:`0xf0008800 <DNA_ID1>` |
+--------------------------+-----------------------------+
| :ref:`DNA_ID0 <DNA_ID0>` | :ref:`0xf0008804 <DNA_ID0>` |
+--------------------------+-----------------------------+

DNA_ID1
^^^^^^^

`Address: 0xf0008800 + 0x0 = 0xf0008800`

    Bits 32-56 of `DNA_ID`.

    .. wavedrom::
        :caption: DNA_ID1

        {
            "reg": [
                {"name": "id[63:32]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


DNA_ID0
^^^^^^^

`Address: 0xf0008800 + 0x4 = 0xf0008804`

    Bits 0-31 of `DNA_ID`.

    .. wavedrom::
        :caption: DNA_ID0

        {
            "reg": [
                {"name": "id[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


