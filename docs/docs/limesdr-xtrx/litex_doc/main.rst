MAIN
====

Register Listing for MAIN
-------------------------

+------------------------------------------------------+-------------------------------------------+
| Register                                             | Address                                   |
+======================================================+===========================================+
| :ref:`MAIN_TIME_MIN_SEC <MAIN_TIME_MIN_SEC>`         | :ref:`0xf0006000 <MAIN_TIME_MIN_SEC>`     |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MAIN_TIME_MON_DAY_HRS <MAIN_TIME_MON_DAY_HRS>` | :ref:`0xf0006004 <MAIN_TIME_MON_DAY_HRS>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MAIN_TIME_YRS <MAIN_TIME_YRS>`                 | :ref:`0xf0006008 <MAIN_TIME_YRS>`         |
+------------------------------------------------------+-------------------------------------------+

MAIN_TIME_MIN_SEC
^^^^^^^^^^^^^^^^^

`Address: 0xf0006000 + 0x0 = 0xf0006000`

    Time in minutes and seconds, current

    .. wavedrom::
        :caption: MAIN_TIME_MIN_SEC

        {
            "reg": [
                {"name": "sec",  "bits": 6},
                {"name": "min",  "bits": 6},
                {"bits": 20}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+--------+------+------------------------+
| Field  | Name | Description            |
+========+======+========================+
| [5:0]  | SEC  | Current time, seconds  |
+--------+------+------------------------+
| [11:6] | MIN  | Current  time, minutes |
+--------+------+------------------------+

MAIN_TIME_MON_DAY_HRS
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006000 + 0x4 = 0xf0006004`

    Time in months, days and hours, current

    .. wavedrom::
        :caption: MAIN_TIME_MON_DAY_HRS

        {
            "reg": [
                {"name": "hrs",  "bits": 5},
                {"name": "day",  "bits": 5},
                {"name": "mon",  "bits": 4},
                {"bits": 18}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+---------+------+----------------------------+
| Field   | Name | Description                |
+=========+======+============================+
| [4:0]   | HRS  | Current time, hours        |
+---------+------+----------------------------+
| [9:5]   | DAY  | Current start time, days   |
+---------+------+----------------------------+
| [13:10] | MON  | Current start time, months |
+---------+------+----------------------------+

MAIN_TIME_YRS
^^^^^^^^^^^^^

`Address: 0xf0006000 + 0x8 = 0xf0006008`

    Time in years, current

    .. wavedrom::
        :caption: MAIN_TIME_YRS

        {
            "reg": [
                {"name": "yrs",  "bits": 12},
                {"bits": 20}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+--------+------+---------------------+
| Field  | Name | Description         |
+========+======+=====================+
| [11:0] | YRS  | Current time, years |
+--------+------+---------------------+

