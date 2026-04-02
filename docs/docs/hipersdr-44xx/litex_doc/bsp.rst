BSP
===

Register Listing for BSP
------------------------

+----------------------------------------+------------------------------------+
| Register                               | Address                            |
+========================================+====================================+
| :ref:`BSP_SW_TRIGGER <BSP_SW_TRIGGER>` | :ref:`0xf0002000 <BSP_SW_TRIGGER>` |
+----------------------------------------+------------------------------------+
| :ref:`BSP_EV_STATUS <BSP_EV_STATUS>`   | :ref:`0xf0002004 <BSP_EV_STATUS>`  |
+----------------------------------------+------------------------------------+
| :ref:`BSP_EV_PENDING <BSP_EV_PENDING>` | :ref:`0xf0002008 <BSP_EV_PENDING>` |
+----------------------------------------+------------------------------------+
| :ref:`BSP_EV_ENABLE <BSP_EV_ENABLE>`   | :ref:`0xf000200c <BSP_EV_ENABLE>`  |
+----------------------------------------+------------------------------------+

BSP_SW_TRIGGER
^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x0 = 0xf0002000`

    Software Trigger

    .. wavedrom::
        :caption: BSP_SW_TRIGGER

        {
            "reg": [
                {"name": "start",  "type": 4, "bits": 1},
                {"bits": 31}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+-------+-------+--------------------+
| Field | Name  | Description        |
+=======+=======+====================+
| [0]   | START | Write 1 to trigger |
+-------+-------+--------------------+

BSP_EV_STATUS
^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x4 = 0xf0002004`

    This register contains the current raw level of the sw event trigger.  Writes to
    this register have no effect.

    .. wavedrom::
        :caption: BSP_EV_STATUS

        {
            "reg": [
                {"name": "event0",  "bits": 1},
                {"name": "event1",  "bits": 1},
                {"name": "sw",  "bits": 1},
                {"bits": 29}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+-------+--------+-------------------------------+
| Field | Name   | Description                   |
+=======+========+===============================+
| [0]   | EVENT0 | Level of the ``event0`` event |
+-------+--------+-------------------------------+
| [1]   | EVENT1 | Level of the ``event1`` event |
+-------+--------+-------------------------------+
| [2]   | SW     | Level of the ``sw`` event     |
+-------+--------+-------------------------------+

BSP_EV_PENDING
^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x8 = 0xf0002008`

    When a  sw event occurs, the corresponding bit will be set in this register.  To
    clear the Event, set the corresponding bit in this register.

    .. wavedrom::
        :caption: BSP_EV_PENDING

        {
            "reg": [
                {"name": "event0",  "bits": 1},
                {"name": "event1",  "bits": 1},
                {"name": "sw",  "bits": 1},
                {"bits": 29}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+-------+--------+--------------------------------------------------------------------------------+
| Field | Name   | Description                                                                    |
+=======+========+================================================================================+
| [0]   | EVENT0 | `1` if a this particular event occurred. This Event is triggered on **rising** |
|       |        | edge.                                                                          |
+-------+--------+--------------------------------------------------------------------------------+
| [1]   | EVENT1 | `1` if a this particular event occurred. This Event is triggered on **rising** |
|       |        | edge.                                                                          |
+-------+--------+--------------------------------------------------------------------------------+
| [2]   | SW     | `1` if a `sw` event occurred. This Event is triggered on a **rising** edge.    |
+-------+--------+--------------------------------------------------------------------------------+

BSP_EV_ENABLE
^^^^^^^^^^^^^

`Address: 0xf0002000 + 0xc = 0xf000200c`

    This register enables the corresponding sw events.  Write a ``0`` to this
    register to disable individual events.

    .. wavedrom::
        :caption: BSP_EV_ENABLE

        {
            "reg": [
                {"name": "event0",  "bits": 1},
                {"name": "event1",  "bits": 1},
                {"name": "sw",  "bits": 1},
                {"bits": 29}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+-------+--------+----------------------------------------------+
| Field | Name   | Description                                  |
+=======+========+==============================================+
| [0]   | EVENT0 | Write a ``1`` to enable the ``event0`` Event |
+-------+--------+----------------------------------------------+
| [1]   | EVENT1 | Write a ``1`` to enable the ``event1`` Event |
+-------+--------+----------------------------------------------+
| [2]   | SW     | Write a ``1`` to enable the ``sw`` Event     |
+-------+--------+----------------------------------------------+

