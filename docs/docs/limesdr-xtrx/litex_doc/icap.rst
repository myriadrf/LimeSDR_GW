ICAP
====

Register Listing for ICAP
-------------------------

+--------------------------------+--------------------------------+
| Register                       | Address                        |
+================================+================================+
| :ref:`ICAP_ADDR <ICAP_ADDR>`   | :ref:`0xf0000800 <ICAP_ADDR>`  |
+--------------------------------+--------------------------------+
| :ref:`ICAP_DATA <ICAP_DATA>`   | :ref:`0xf0000804 <ICAP_DATA>`  |
+--------------------------------+--------------------------------+
| :ref:`ICAP_WRITE <ICAP_WRITE>` | :ref:`0xf0000808 <ICAP_WRITE>` |
+--------------------------------+--------------------------------+
| :ref:`ICAP_DONE <ICAP_DONE>`   | :ref:`0xf000080c <ICAP_DONE>`  |
+--------------------------------+--------------------------------+
| :ref:`ICAP_READ <ICAP_READ>`   | :ref:`0xf0000810 <ICAP_READ>`  |
+--------------------------------+--------------------------------+

ICAP_ADDR
^^^^^^^^^

`Address: 0xf0000800 + 0x0 = 0xf0000800`

    ICAP Address.

    .. wavedrom::
        :caption: ICAP_ADDR

        {
            "reg": [
                {"name": "addr[4:0]", "bits": 5},
                {"bits": 27},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ICAP_DATA
^^^^^^^^^

`Address: 0xf0000800 + 0x4 = 0xf0000804`

    ICAP Write/Read Data.

    .. wavedrom::
        :caption: ICAP_DATA

        {
            "reg": [
                {"name": "data[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


ICAP_WRITE
^^^^^^^^^^

`Address: 0xf0000800 + 0x8 = 0xf0000808`

    ICAP Control.

    Write ``1`` send a write to the ICAP.

    .. wavedrom::
        :caption: ICAP_WRITE

        {
            "reg": [
                {"name": "write", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ICAP_DONE
^^^^^^^^^

`Address: 0xf0000800 + 0xc = 0xf000080c`

    ICAP Status.

    Write command done when read as ``1``.

    .. wavedrom::
        :caption: ICAP_DONE

        {
            "reg": [
                {"name": "done", "attr": 'reset: 1', "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


ICAP_READ
^^^^^^^^^

`Address: 0xf0000800 + 0x10 = 0xf0000810`

    ICAP Control.

    Read ``1`` send a read from the ICAP.

    .. wavedrom::
        :caption: ICAP_READ

        {
            "reg": [
                {"name": "read", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


