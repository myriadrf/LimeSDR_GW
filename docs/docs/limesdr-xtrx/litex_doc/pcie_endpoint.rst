PCIE_ENDPOINT
=============

Register Listing for PCIE_ENDPOINT
----------------------------------

+----------------------------------------------------------------------------------+---------------------------------------------------------+
| Register                                                                         | Address                                                 |
+==================================================================================+=========================================================+
| :ref:`PCIE_ENDPOINT_PHY_LINK_STATUS <PCIE_ENDPOINT_PHY_LINK_STATUS>`             | :ref:`0xf000a000 <PCIE_ENDPOINT_PHY_LINK_STATUS>`       |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`PCIE_ENDPOINT_PHY_MSI_ENABLE <PCIE_ENDPOINT_PHY_MSI_ENABLE>`               | :ref:`0xf000a004 <PCIE_ENDPOINT_PHY_MSI_ENABLE>`        |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`PCIE_ENDPOINT_PHY_MSIX_ENABLE <PCIE_ENDPOINT_PHY_MSIX_ENABLE>`             | :ref:`0xf000a008 <PCIE_ENDPOINT_PHY_MSIX_ENABLE>`       |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE <PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE>` | :ref:`0xf000a00c <PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE>` |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE <PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE>`   | :ref:`0xf000a010 <PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE>`  |
+----------------------------------------------------------------------------------+---------------------------------------------------------+
| :ref:`PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE <PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE>`   | :ref:`0xf000a014 <PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE>`  |
+----------------------------------------------------------------------------------+---------------------------------------------------------+

PCIE_ENDPOINT_PHY_LINK_STATUS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x0 = 0xf000a000`


    .. wavedrom::
        :caption: PCIE_ENDPOINT_PHY_LINK_STATUS

        {
            "reg": [
                {"name": "status",  "bits": 1},
                {"name": "rate",  "bits": 1},
                {"name": "width",  "bits": 2},
                {"name": "ltssm",  "bits": 6},
                {"bits": 22}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+--------+-----------------------------+
| Field | Name   | Description                 |
+=======+========+=============================+
| [0]   | STATUS |                             |
|       |        |                             |
|       |        | +---------+-------------+   |
|       |        | | Value   | Description |   |
|       |        | +=========+=============+   |
|       |        | | ``0b0`` | Link Down.  |   |
|       |        | +---------+-------------+   |
|       |        | | ``0b1`` | Link Up.    |   |
|       |        | +---------+-------------+   |
+-------+--------+-----------------------------+
| [1]   | RATE   |                             |
|       |        |                             |
|       |        | +---------+-------------+   |
|       |        | | Value   | Description |   |
|       |        | +=========+=============+   |
|       |        | | ``0b0`` | 2.5 Gb/s.   |   |
|       |        | +---------+-------------+   |
|       |        | | ``0b1`` | 5.0 Gb/s.   |   |
|       |        | +---------+-------------+   |
+-------+--------+-----------------------------+
| [3:2] | WIDTH  |                             |
|       |        |                             |
|       |        | +----------+--------------+ |
|       |        | | Value    | Description  | |
|       |        | +==========+==============+ |
|       |        | | ``0b00`` | 1-Lane link. | |
|       |        | +----------+--------------+ |
|       |        | | ``0b01`` | 2-Lane link. | |
|       |        | +----------+--------------+ |
|       |        | | ``0b10`` | 4-Lane link. | |
|       |        | +----------+--------------+ |
|       |        | | ``0b11`` | 8-Lane link. | |
|       |        | +----------+--------------+ |
+-------+--------+-----------------------------+
| [9:4] | LTSSM  | LTSSM State                 |
+-------+--------+-----------------------------+

PCIE_ENDPOINT_PHY_MSI_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x4 = 0xf000a004`

    MSI Enable Status. ``1``: MSI is enabled.

    .. wavedrom::
        :caption: PCIE_ENDPOINT_PHY_MSI_ENABLE

        {
            "reg": [
                {"name": "phy_msi_enable", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_ENDPOINT_PHY_MSIX_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x8 = 0xf000a008`

    MSI-X Enable Status. ``1``: MSI-X is enabled.

    .. wavedrom::
        :caption: PCIE_ENDPOINT_PHY_MSIX_ENABLE

        {
            "reg": [
                {"name": "phy_msix_enable", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0xc = 0xf000a00c`

    Bus Mastering Status. ``1``: Bus Mastering enabled.

    .. wavedrom::
        :caption: PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE

        {
            "reg": [
                {"name": "phy_bus_master_enable", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x10 = 0xf000a010`

    Negiotiated Max Request Size (in bytes).

    .. wavedrom::
        :caption: PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE

        {
            "reg": [
                {"name": "phy_max_request_size[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf000a000 + 0x14 = 0xf000a014`

    Negiotiated Max Payload Size (in bytes).

    .. wavedrom::
        :caption: PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE

        {
            "reg": [
                {"name": "phy_max_payload_size[15:0]", "bits": 16},
                {"bits": 16},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


