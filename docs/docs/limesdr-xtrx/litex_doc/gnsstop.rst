GNSSTOP
=======

Register Listing for GNSSTOP
----------------------------

+------------------------------------------------------------+----------------------------------------------+
| Register                                                   | Address                                      |
+============================================================+==============================================+
| :ref:`GNSSTOP_TIME_MIN_SEC <GNSSTOP_TIME_MIN_SEC>`         | :ref:`0xf0003800 <GNSSTOP_TIME_MIN_SEC>`     |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_TIME_MON_DAY_HRS <GNSSTOP_TIME_MON_DAY_HRS>` | :ref:`0xf0003804 <GNSSTOP_TIME_MON_DAY_HRS>` |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_TIME_YRS <GNSSTOP_TIME_YRS>`                 | :ref:`0xf0003808 <GNSSTOP_TIME_YRS>`         |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_UTC_SSS0 <GNSSTOP_GNSS_UTC_SSS0>`       | :ref:`0xf000380c <GNSSTOP_GNSS_UTC_SSS0>`    |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_UTC_MM_SS1 <GNSSTOP_GNSS_UTC_MM_SS1>`   | :ref:`0xf0003810 <GNSSTOP_GNSS_UTC_MM_SS1>`  |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_UTC_HH <GNSSTOP_GNSS_UTC_HH>`           | :ref:`0xf0003814 <GNSSTOP_GNSS_UTC_HH>`      |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_DATE_MM_YY <GNSSTOP_GNSS_DATE_MM_YY>`   | :ref:`0xf0003818 <GNSSTOP_GNSS_DATE_MM_YY>`  |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_DATE_DD <GNSSTOP_GNSS_DATE_DD>`         | :ref:`0xf000381c <GNSSTOP_GNSS_DATE_DD>`     |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_STATUS <GNSSTOP_GNSS_STATUS>`           | :ref:`0xf0003820 <GNSSTOP_GNSS_STATUS>`      |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_LAT <GNSSTOP_GNSS_LAT>`                 | :ref:`0xf0003824 <GNSSTOP_GNSS_LAT>`         |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_LONG <GNSSTOP_GNSS_LONG>`               | :ref:`0xf0003828 <GNSSTOP_GNSS_LONG>`        |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_LONG_EXT <GNSSTOP_GNSS_LONG_EXT>`       | :ref:`0xf000382c <GNSSTOP_GNSS_LONG_EXT>`    |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_SPEED <GNSSTOP_GNSS_SPEED>`             | :ref:`0xf0003830 <GNSSTOP_GNSS_SPEED>`       |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_COURSE <GNSSTOP_GNSS_COURSE>`           | :ref:`0xf0003834 <GNSSTOP_GNSS_COURSE>`      |
+------------------------------------------------------------+----------------------------------------------+
| :ref:`GNSSTOP_GNSS_FIX <GNSSTOP_GNSS_FIX>`                 | :ref:`0xf0003838 <GNSSTOP_GNSS_FIX>`         |
+------------------------------------------------------------+----------------------------------------------+

GNSSTOP_TIME_MIN_SEC
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x0 = 0xf0003800`

    Time in minutes and seconds, current

    .. wavedrom::
        :caption: GNSSTOP_TIME_MIN_SEC

        {
            "reg": [
                {"name": "sec",  "bits": 6},
                {"name": "min",  "bits": 6},
                {"bits": 20}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+--------+------+-----------------------+
| Field  | Name | Description           |
+========+======+=======================+
| [5:0]  | SEC  | Current time, seconds |
+--------+------+-----------------------+
| [11:6] | MIN  | Current time, minutes |
+--------+------+-----------------------+

GNSSTOP_TIME_MON_DAY_HRS
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x4 = 0xf0003804`

    Time in months, days and hours, current

    .. wavedrom::
        :caption: GNSSTOP_TIME_MON_DAY_HRS

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

GNSSTOP_TIME_YRS
^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x8 = 0xf0003808`

    Time in years, current

    .. wavedrom::
        :caption: GNSSTOP_TIME_YRS

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

GNSSTOP_GNSS_UTC_SSS0
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0xc = 0xf000380c`

    GNSS UTC sub-seconds

    .. wavedrom::
        :caption: GNSSTOP_GNSS_UTC_SSS0

        {
            "reg": [
                {"name": "gnss_utc_sss0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_UTC_MM_SS1
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x10 = 0xf0003810`

    GNSS UTC minutes and seconds

    .. wavedrom::
        :caption: GNSSTOP_GNSS_UTC_MM_SS1

        {
            "reg": [
                {"name": "gnss_utc_mm_ss1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_UTC_HH
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x14 = 0xf0003814`

    GNSS UTC hours

    .. wavedrom::
        :caption: GNSSTOP_GNSS_UTC_HH

        {
            "reg": [
                {"name": "gnss_utc_hh[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_DATE_MM_YY
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x18 = 0xf0003818`

    GNSS Date month and year

    .. wavedrom::
        :caption: GNSSTOP_GNSS_DATE_MM_YY

        {
            "reg": [
                {"name": "gnss_date_mm_yy[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_DATE_DD
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x1c = 0xf000381c`

    GNSS Date day

    .. wavedrom::
        :caption: GNSSTOP_GNSS_DATE_DD

        {
            "reg": [
                {"name": "gnss_date_dd[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_STATUS
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x20 = 0xf0003820`

    GNSS Status (bit 0: RMC status, 1: Lat N/S, 2: Long E/W)

    .. wavedrom::
        :caption: GNSSTOP_GNSS_STATUS

        {
            "reg": [
                {"name": "gnss_status[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_LAT
^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x24 = 0xf0003824`

    GNSS Latitude (BCD digits)

    .. wavedrom::
        :caption: GNSSTOP_GNSS_LAT

        {
            "reg": [
                {"name": "gnss_lat[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_LONG
^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x28 = 0xf0003828`

    GNSS Longitude (BCD digits)

    .. wavedrom::
        :caption: GNSSTOP_GNSS_LONG

        {
            "reg": [
                {"name": "gnss_long[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_LONG_EXT
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x2c = 0xf000382c`

    GNSS Longitude digit Y4

    .. wavedrom::
        :caption: GNSSTOP_GNSS_LONG_EXT

        {
            "reg": [
                {"name": "gnss_long_ext[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_SPEED
^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x30 = 0xf0003830`

    GNSS Speed (BCD digits)

    .. wavedrom::
        :caption: GNSSTOP_GNSS_SPEED

        {
            "reg": [
                {"name": "gnss_speed[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_COURSE
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x34 = 0xf0003834`

    GNSS Course (BCD digits)

    .. wavedrom::
        :caption: GNSSTOP_GNSS_COURSE

        {
            "reg": [
                {"name": "gnss_course[23:0]", "bits": 24},
                {"bits": 8},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


GNSSTOP_GNSS_FIX
^^^^^^^^^^^^^^^^

`Address: 0xf0003800 + 0x38 = 0xf0003838`

    GNSS Fix Status (4 bits per constellation: GL, GB, GP, GA)

    .. wavedrom::
        :caption: GNSSTOP_GNSS_FIX

        {
            "reg": [
                {"name": "gnss_fix[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


