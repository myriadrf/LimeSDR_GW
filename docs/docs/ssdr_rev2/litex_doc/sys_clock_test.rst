SYS_CLOCK_TEST
==============

Register Listing for SYS_CLOCK_TEST
-----------------------------------

+--------------------------------------------------------------------+--------------------------------------------------+
| Register                                                           | Address                                          |
+====================================================================+==================================================+
| :ref:`SYS_CLOCK_TEST_TEST_EN <SYS_CLOCK_TEST_TEST_EN>`             | :ref:`0xf000a800 <SYS_CLOCK_TEST_TEST_EN>`       |
+--------------------------------------------------------------------+--------------------------------------------------+
| :ref:`SYS_CLOCK_TEST_TEST_CNT <SYS_CLOCK_TEST_TEST_CNT>`           | :ref:`0xf000a804 <SYS_CLOCK_TEST_TEST_CNT>`      |
+--------------------------------------------------------------------+--------------------------------------------------+
| :ref:`SYS_CLOCK_TEST_TEST_COMPLETE <SYS_CLOCK_TEST_TEST_COMPLETE>` | :ref:`0xf000a808 <SYS_CLOCK_TEST_TEST_COMPLETE>` |
+--------------------------------------------------------------------+--------------------------------------------------+

SYS_CLOCK_TEST_TEST_EN
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x0 = 0xf000a800`

    1 - enable test, 0 - disable test

    .. wavedrom::
        :caption: SYS_CLOCK_TEST_TEST_EN

        {
            "reg": [
                {"name": "test_en", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


SYS_CLOCK_TEST_TEST_CNT
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x4 = 0xf000a804`

    Number of cycles counted during test

    .. wavedrom::
        :caption: SYS_CLOCK_TEST_TEST_CNT

        {
            "reg": [
                {"name": "test_cnt[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


SYS_CLOCK_TEST_TEST_COMPLETE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a800 + 0x8 = 0xf000a808`

    1 - test complete, 0 - test not complete

    .. wavedrom::
        :caption: SYS_CLOCK_TEST_TEST_COMPLETE

        {
            "reg": [
                {"name": "test_complete", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


