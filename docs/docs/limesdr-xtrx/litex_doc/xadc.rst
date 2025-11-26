XADC
====

Register Listing for XADC
-------------------------

+--------------------------------------------+--------------------------------------+
| Register                                   | Address                              |
+============================================+======================================+
| :ref:`XADC_TEMPERATURE <XADC_TEMPERATURE>` | :ref:`0xf0008000 <XADC_TEMPERATURE>` |
+--------------------------------------------+--------------------------------------+
| :ref:`XADC_VCCINT <XADC_VCCINT>`           | :ref:`0xf0008004 <XADC_VCCINT>`      |
+--------------------------------------------+--------------------------------------+
| :ref:`XADC_VCCAUX <XADC_VCCAUX>`           | :ref:`0xf0008008 <XADC_VCCAUX>`      |
+--------------------------------------------+--------------------------------------+
| :ref:`XADC_VCCBRAM <XADC_VCCBRAM>`         | :ref:`0xf000800c <XADC_VCCBRAM>`     |
+--------------------------------------------+--------------------------------------+
| :ref:`XADC_EOC <XADC_EOC>`                 | :ref:`0xf0008010 <XADC_EOC>`         |
+--------------------------------------------+--------------------------------------+
| :ref:`XADC_EOS <XADC_EOS>`                 | :ref:`0xf0008014 <XADC_EOS>`         |
+--------------------------------------------+--------------------------------------+

XADC_TEMPERATURE
^^^^^^^^^^^^^^^^

`Address: 0xf0008000 + 0x0 = 0xf0008000`

    Raw Temperature value from XADC. Temperature (°C) = ``Value`` x 503.975 / 4096 -
    273.15.

    .. wavedrom::
        :caption: XADC_TEMPERATURE

        {
            "reg": [
                {"name": "temperature[11:0]", "bits": 12},
                {"bits": 20},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


XADC_VCCINT
^^^^^^^^^^^

`Address: 0xf0008000 + 0x4 = 0xf0008004`

    Raw VCCINT value from XADC. VCCINT (V) = ``Value`` x 3 / 4096.

    .. wavedrom::
        :caption: XADC_VCCINT

        {
            "reg": [
                {"name": "vccint[11:0]", "bits": 12},
                {"bits": 20},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


XADC_VCCAUX
^^^^^^^^^^^

`Address: 0xf0008000 + 0x8 = 0xf0008008`

    Raw VCCAUX value from XADC. VCCAUX (V) = ``Value`` x 3 / 4096.

    .. wavedrom::
        :caption: XADC_VCCAUX

        {
            "reg": [
                {"name": "vccaux[11:0]", "bits": 12},
                {"bits": 20},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


XADC_VCCBRAM
^^^^^^^^^^^^

`Address: 0xf0008000 + 0xc = 0xf000800c`

    Raw VCCBRAM value from XADC. VCCBRAM (V) = ``Value`` x 3 / 4096.

    .. wavedrom::
        :caption: XADC_VCCBRAM

        {
            "reg": [
                {"name": "vccbram[11:0]", "bits": 12},
                {"bits": 20},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


XADC_EOC
^^^^^^^^

`Address: 0xf0008000 + 0x10 = 0xf0008010`

    End of Convertion Status, ``1``: Convertion Done.

    .. wavedrom::
        :caption: XADC_EOC

        {
            "reg": [
                {"name": "eoc", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


XADC_EOS
^^^^^^^^

`Address: 0xf0008000 + 0x14 = 0xf0008014`

    End of Sequence Status,   ``1``: Sequence Done.

    .. wavedrom::
        :caption: XADC_EOS

        {
            "reg": [
                {"name": "eos", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


