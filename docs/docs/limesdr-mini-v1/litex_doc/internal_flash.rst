INTERNAL_FLASH
==============

Register Listing for INTERNAL_FLASH
-----------------------------------

+--------------------------------------------------------------------------+-----------------------------------------------------+
| Register                                                                 | Address                                             |
+==========================================================================+=====================================================+
| :ref:`INTERNAL_FLASH_STATUS_REGISTER <INTERNAL_FLASH_STATUS_REGISTER>`   | :ref:`0xf0002000 <INTERNAL_FLASH_STATUS_REGISTER>`  |
+--------------------------------------------------------------------------+-----------------------------------------------------+
| :ref:`INTERNAL_FLASH_CONTROL_REGISTER <INTERNAL_FLASH_CONTROL_REGISTER>` | :ref:`0xf0002004 <INTERNAL_FLASH_CONTROL_REGISTER>` |
+--------------------------------------------------------------------------+-----------------------------------------------------+

INTERNAL_FLASH_STATUS_REGISTER
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x0 = 0xf0002000`

    On-Chip Flash Status Register.

    .. wavedrom::
        :caption: INTERNAL_FLASH_STATUS_REGISTER

        {
            "reg": [
                {"name": "status_register[31:0]", "bits": 32}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


INTERNAL_FLASH_CONTROL_REGISTER
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0002000 + 0x4 = 0xf0002004`

    On-Chip Flash Control Register.

    .. wavedrom::
        :caption: INTERNAL_FLASH_CONTROL_REGISTER

        {
            "reg": [
                {"name": "control_register[31:0]", "bits": 32}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


