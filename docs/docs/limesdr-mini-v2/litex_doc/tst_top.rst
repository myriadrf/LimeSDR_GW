TST_TOP
=======

Register Listing for TST_TOP
----------------------------

+----------------------------------------------------+------------------------------------------+
| Register                                           | Address                                  |
+====================================================+==========================================+
| :ref:`TST_TOP_TEST_EN <TST_TOP_TEST_EN>`           | :ref:`0xf0005800 <TST_TOP_TEST_EN>`      |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_TEST_FRC_ERR <TST_TOP_TEST_FRC_ERR>` | :ref:`0xf0005804 <TST_TOP_TEST_FRC_ERR>` |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_TEST_CMPLT <TST_TOP_TEST_CMPLT>`     | :ref:`0xf0005808 <TST_TOP_TEST_CMPLT>`   |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_TEST_REZ <TST_TOP_TEST_REZ>`         | :ref:`0xf000580c <TST_TOP_TEST_REZ>`     |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_FX3_CLK_CNT <TST_TOP_FX3_CLK_CNT>`   | :ref:`0xf0005810 <TST_TOP_FX3_CLK_CNT>`  |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_LMK_CLK_CNT0 <TST_TOP_LMK_CLK_CNT0>` | :ref:`0xf0005814 <TST_TOP_LMK_CLK_CNT0>` |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_LMK_CLK_CNT1 <TST_TOP_LMK_CLK_CNT1>` | :ref:`0xf0005818 <TST_TOP_LMK_CLK_CNT1>` |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_ADF_CNT <TST_TOP_ADF_CNT>`           | :ref:`0xf000581c <TST_TOP_ADF_CNT>`      |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_TX_TST_I <TST_TOP_TX_TST_I>`         | :ref:`0xf0005820 <TST_TOP_TX_TST_I>`     |
+----------------------------------------------------+------------------------------------------+
| :ref:`TST_TOP_TX_TST_Q <TST_TOP_TX_TST_Q>`         | :ref:`0xf0005824 <TST_TOP_TX_TST_Q>`     |
+----------------------------------------------------+------------------------------------------+

TST_TOP_TEST_EN
^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x0 = 0xf0005800`


    .. wavedrom::
        :caption: TST_TOP_TEST_EN

        {
            "reg": [
                {"name": "fx3_pclk_tst_en",  "bits": 1},
                {"bits": 1},
                {"name": "vctcxo_tst_en",  "bits": 1},
                {"name": "adf_tst_en",  "bits": 1},
                {"name": "ddr2_1_tst_en",  "bits": 1},
                {"name": "ddr2_2_tst_en",  "bits": 1},
                {"bits": 26}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+-----------------+-------------+
| Field | Name            | Description |
+=======+=================+=============+
+-------+-----------------+-------------+
+-------+-----------------+-------------+
+-------+-----------------+-------------+
+-------+-----------------+-------------+
+-------+-----------------+-------------+

TST_TOP_TEST_FRC_ERR
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x4 = 0xf0005804`


    .. wavedrom::
        :caption: TST_TOP_TEST_FRC_ERR

        {
            "reg": [
                {"name": "fx3_pclk_tst_frc_err",  "bits": 1},
                {"bits": 1},
                {"name": "vctco_tst_frc_err",  "bits": 1},
                {"name": "adf_tst_frc_err",  "bits": 1},
                {"name": "ddr2_1_tst_frc_err",  "bits": 1},
                {"name": "ddr2_2_tst_frc_err",  "bits": 1},
                {"bits": 26}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+----------------------+-------------+
| Field | Name                 | Description |
+=======+======================+=============+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+
+-------+----------------------+-------------+

TST_TOP_TEST_CMPLT
^^^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x8 = 0xf0005808`


    .. wavedrom::
        :caption: TST_TOP_TEST_CMPLT

        {
            "reg": [
                {"name": "test_cmplt[5:0]", "bits": 6},
                {"bits": 26},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


TST_TOP_TEST_REZ
^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0xc = 0xf000580c`


    .. wavedrom::
        :caption: TST_TOP_TEST_REZ

        {
            "reg": [
                {"name": "test_rez[5:0]", "bits": 6},
                {"bits": 26},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


TST_TOP_FX3_CLK_CNT
^^^^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x10 = 0xf0005810`


    .. wavedrom::
        :caption: TST_TOP_FX3_CLK_CNT

        {
            "reg": [
                {"name": "fx3_clk_cnt[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


TST_TOP_LMK_CLK_CNT0
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x14 = 0xf0005814`


    .. wavedrom::
        :caption: TST_TOP_LMK_CLK_CNT0

        {
            "reg": [
                {"name": "lmk_clk_cnt0[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


TST_TOP_LMK_CLK_CNT1
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x18 = 0xf0005818`


    .. wavedrom::
        :caption: TST_TOP_LMK_CLK_CNT1

        {
            "reg": [
                {"name": "lmk_clk_cnt1[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


TST_TOP_ADF_CNT
^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x1c = 0xf000581c`


    .. wavedrom::
        :caption: TST_TOP_ADF_CNT

        {
            "reg": [
                {"name": "adf_cnt[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


TST_TOP_TX_TST_I
^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x20 = 0xf0005820`


    .. wavedrom::
        :caption: TST_TOP_TX_TST_I

        {
            "reg": [
                {"name": "tx_tst_i[15:0]", "attr": 'reset: 43690', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


TST_TOP_TX_TST_Q
^^^^^^^^^^^^^^^^

`Address: 0xf0005800 + 0x24 = 0xf0005824`


    .. wavedrom::
        :caption: TST_TOP_TX_TST_Q

        {
            "reg": [
                {"name": "tx_tst_q[15:0]", "attr": 'reset: 21845', "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


