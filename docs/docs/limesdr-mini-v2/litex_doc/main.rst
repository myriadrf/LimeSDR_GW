MAIN
====

Register Listing for MAIN
-------------------------

+------------------------------+-------------------------------+
| Register                     | Address                       |
+==============================+===============================+
| :ref:`MAIN_GPIO <MAIN_GPIO>` | :ref:`0xf0003800 <MAIN_GPIO>` |
+------------------------------+-------------------------------+
| :ref:`MAIN_GPO <MAIN_GPO>`   | :ref:`0xf0003804 <MAIN_GPO>`  |
+------------------------------+-------------------------------+

MAIN_GPIO
^^^^^^^^^

`Address: 0xf0003800 + 0x0 = 0xf0003800`


    .. wavedrom::
        :caption: MAIN_GPIO

        {
            "reg": [
                {"name": "gpio[15:0]", "attr": 'reset: 4420', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MAIN_GPO
^^^^^^^^

`Address: 0xf0003800 + 0x4 = 0xf0003804`

    GPO interface

    .. wavedrom::
        :caption: MAIN_GPO

        {
            "reg": [
                {"name": "cpu_busy",  "bits": 1},
                {"bits": 31}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+----------+---------------------------+
| Field | Name     | Description               |
+=======+==========+===========================+
| [0]   | CPU_BUSY | CPU state.                |
|       |          |                           |
|       |          | +---------+-------------+ |
|       |          | | Value   | Description | |
|       |          | +=========+=============+ |
|       |          | | ``0b0`` | IDLE.       | |
|       |          | +---------+-------------+ |
|       |          | | ``0b1`` | BUSY.       | |
|       |          | +---------+-------------+ |
+-------+----------+---------------------------+

