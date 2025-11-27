PCIE_MSI
========

Register Listing for PCIE_MSI
-----------------------------

+------------------------------------------+-------------------------------------+
| Register                                 | Address                             |
+==========================================+=====================================+
| :ref:`PCIE_MSI_ENABLE <PCIE_MSI_ENABLE>` | :ref:`0xf0001800 <PCIE_MSI_ENABLE>` |
+------------------------------------------+-------------------------------------+
| :ref:`PCIE_MSI_CLEAR <PCIE_MSI_CLEAR>`   | :ref:`0xf0001804 <PCIE_MSI_CLEAR>`  |
+------------------------------------------+-------------------------------------+
| :ref:`PCIE_MSI_VECTOR <PCIE_MSI_VECTOR>` | :ref:`0xf0001808 <PCIE_MSI_VECTOR>` |
+------------------------------------------+-------------------------------------+

PCIE_MSI_ENABLE
^^^^^^^^^^^^^^^

`Address: 0xf0001800 + 0x0 = 0xf0001800`

    MSI Enable Control.

    Write bit(s) to ``1`` to enable corresponding MSI IRQ(s).

    .. wavedrom::
        :caption: PCIE_MSI_ENABLE

        {
            "reg": [
                {"name": "enable[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_MSI_CLEAR
^^^^^^^^^^^^^^

`Address: 0xf0001800 + 0x4 = 0xf0001804`

    MSI Clear Control.

    Write bit(s) to ``1`` to clear corresponding MSI IRQ(s).

    .. wavedrom::
        :caption: PCIE_MSI_CLEAR

        {
            "reg": [
                {"name": "clear[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_MSI_VECTOR
^^^^^^^^^^^^^^^

`Address: 0xf0001800 + 0x8 = 0xf0001808`

    MSI Vector Status.

    Current MSI IRQs vector value.

    .. wavedrom::
        :caption: PCIE_MSI_VECTOR

        {
            "reg": [
                {"name": "vector[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


