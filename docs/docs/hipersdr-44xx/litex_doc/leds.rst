LEDS
====

Register Listing for LEDS
-------------------------

+----------------------------+------------------------------+
| Register                   | Address                      |
+============================+==============================+
| :ref:`LEDS_OUT <LEDS_OUT>` | :ref:`0xf0005800 <LEDS_OUT>` |
+----------------------------+------------------------------+

LEDS_OUT
^^^^^^^^

`Address: 0xf0005800 + 0x0 = 0xf0005800`

    Led Output(s) Control.

    .. wavedrom::
        :caption: LEDS_OUT

        {
            "reg": [
                {"name": "out[2:0]", "bits": 3},
                {"bits": 29},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


