PWR_CTRL
========

Register Listing for PWR_CTRL
-----------------------------

+----------------------------------------+------------------------------------+
| Register                               | Address                            |
+========================================+====================================+
| :ref:`PWR_CTRL_LDOEN <PWR_CTRL_LDOEN>` | :ref:`0xf0009800 <PWR_CTRL_LDOEN>` |
+----------------------------------------+------------------------------------+

PWR_CTRL_LDOEN
^^^^^^^^^^^^^^

`Address: 0xf0009800 + 0x0 = 0xf0009800`

    LDO enable

    .. wavedrom::
        :caption: PWR_CTRL_LDOEN

        {
            "reg": [
                {"name": "ldoen", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


