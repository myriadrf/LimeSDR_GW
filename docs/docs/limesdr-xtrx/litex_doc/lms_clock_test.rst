LMS_CLOCK_TEST
==============

Register Listing for LMS_CLOCK_TEST
-----------------------------------

+--------------------------------------------------------------------+--------------------------------------------------+
| Register                                                           | Address                                          |
+====================================================================+==================================================+
| :ref:`LMS_CLOCK_TEST_TEST_EN <LMS_CLOCK_TEST_TEST_EN>`             | :ref:`0xf0006000 <LMS_CLOCK_TEST_TEST_EN>`       |
+--------------------------------------------------------------------+--------------------------------------------------+
| :ref:`LMS_CLOCK_TEST_TEST_CNT <LMS_CLOCK_TEST_TEST_CNT>`           | :ref:`0xf0006004 <LMS_CLOCK_TEST_TEST_CNT>`      |
+--------------------------------------------------------------------+--------------------------------------------------+
| :ref:`LMS_CLOCK_TEST_TEST_COMPLETE <LMS_CLOCK_TEST_TEST_COMPLETE>` | :ref:`0xf0006008 <LMS_CLOCK_TEST_TEST_COMPLETE>` |
+--------------------------------------------------------------------+--------------------------------------------------+

LMS_CLOCK_TEST_TEST_EN
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006000 + 0x0 = 0xf0006000`

    1 - enable test, 0 - disable test

    .. wavedrom::
        :caption: LMS_CLOCK_TEST_TEST_EN

        {
            "reg": [
                {"name": "test_en", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


LMS_CLOCK_TEST_TEST_CNT
^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006000 + 0x4 = 0xf0006004`

    Number of cycles counted during test

    .. wavedrom::
        :caption: LMS_CLOCK_TEST_TEST_CNT

        {
            "reg": [
                {"name": "test_cnt[22:0]", "bits": 23},
                {"bits": 9},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


LMS_CLOCK_TEST_TEST_COMPLETE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006000 + 0x8 = 0xf0006008`

    1 - test complete, 0 - test not complete

    .. wavedrom::
        :caption: LMS_CLOCK_TEST_TEST_COMPLETE

        {
            "reg": [
                {"name": "test_complete", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


