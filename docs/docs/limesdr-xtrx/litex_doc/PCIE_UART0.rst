PCIE_UART0
==========

Register Listing for PCIE_UART0
-------------------------------

+------------------------------------------------------+-------------------------------------------+
| Register                                             | Address                                   |
+======================================================+===========================================+
| :ref:`PCIE_UART0_RXTX <PCIE_UART0_RXTX>`             | :ref:`0xf0006800 <PCIE_UART0_RXTX>`       |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_TXFULL <PCIE_UART0_TXFULL>`         | :ref:`0xf0006804 <PCIE_UART0_TXFULL>`     |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_RXEMPTY <PCIE_UART0_RXEMPTY>`       | :ref:`0xf0006808 <PCIE_UART0_RXEMPTY>`    |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_EV_STATUS <PCIE_UART0_EV_STATUS>`   | :ref:`0xf000680c <PCIE_UART0_EV_STATUS>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_EV_PENDING <PCIE_UART0_EV_PENDING>` | :ref:`0xf0006810 <PCIE_UART0_EV_PENDING>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_EV_ENABLE <PCIE_UART0_EV_ENABLE>`   | :ref:`0xf0006814 <PCIE_UART0_EV_ENABLE>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_TXEMPTY <PCIE_UART0_TXEMPTY>`       | :ref:`0xf0006818 <PCIE_UART0_TXEMPTY>`    |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART0_RXFULL <PCIE_UART0_RXFULL>`         | :ref:`0xf000681c <PCIE_UART0_RXFULL>`     |
+------------------------------------------------------+-------------------------------------------+

PCIE_UART0_RXTX
^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x0 = 0xf0006800`


    .. wavedrom::
        :caption: PCIE_UART0_RXTX

        {
            "reg": [
                {"name": "rxtx[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_UART0_TXFULL
^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x4 = 0xf0006804`

    TX FIFO Full.

    .. wavedrom::
        :caption: PCIE_UART0_TXFULL

        {
            "reg": [
                {"name": "txfull", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_UART0_RXEMPTY
^^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x8 = 0xf0006808`

    RX FIFO Empty.

    .. wavedrom::
        :caption: PCIE_UART0_RXEMPTY

        {
            "reg": [
                {"name": "rxempty", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_UART0_EV_STATUS
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0xc = 0xf000680c`

    This register contains the current raw level of the rx event trigger.  Writes to
    this register have no effect.

    .. wavedrom::
        :caption: PCIE_UART0_EV_STATUS

        {
            "reg": [
                {"name": "tx",  "bits": 1},
                {"name": "rx",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------+---------------------------+
| Field | Name | Description               |
+=======+======+===========================+
| [0]   | TX   | Level of the ``tx`` event |
+-------+------+---------------------------+
| [1]   | RX   | Level of the ``rx`` event |
+-------+------+---------------------------+

PCIE_UART0_EV_PENDING
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x10 = 0xf0006810`

    When a  rx event occurs, the corresponding bit will be set in this register.  To
    clear the Event, set the corresponding bit in this register.

    .. wavedrom::
        :caption: PCIE_UART0_EV_PENDING

        {
            "reg": [
                {"name": "tx",  "bits": 1},
                {"name": "rx",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------+---------------------------------------------------------------------------------+
| Field | Name | Description                                                                     |
+=======+======+=================================================================================+
| [0]   | TX   | `1` if a `tx` event occurred. This Event is **level triggered** when the signal |
|       |      | is **high**.                                                                    |
+-------+------+---------------------------------------------------------------------------------+
| [1]   | RX   | `1` if a `rx` event occurred. This Event is **level triggered** when the signal |
|       |      | is **high**.                                                                    |
+-------+------+---------------------------------------------------------------------------------+

PCIE_UART0_EV_ENABLE
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x14 = 0xf0006814`

    This register enables the corresponding rx events.  Write a ``0`` to this
    register to disable individual events.

    .. wavedrom::
        :caption: PCIE_UART0_EV_ENABLE

        {
            "reg": [
                {"name": "tx",  "bits": 1},
                {"name": "rx",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+------+------------------------------------------+
| Field | Name | Description                              |
+=======+======+==========================================+
| [0]   | TX   | Write a ``1`` to enable the ``tx`` Event |
+-------+------+------------------------------------------+
| [1]   | RX   | Write a ``1`` to enable the ``rx`` Event |
+-------+------+------------------------------------------+

PCIE_UART0_TXEMPTY
^^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x18 = 0xf0006818`

    TX FIFO Empty.

    .. wavedrom::
        :caption: PCIE_UART0_TXEMPTY

        {
            "reg": [
                {"name": "txempty", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_UART0_RXFULL
^^^^^^^^^^^^^^^^^

`Address: 0xf0006800 + 0x1c = 0xf000681c`

    RX FIFO Full.

    .. wavedrom::
        :caption: PCIE_UART0_RXFULL

        {
            "reg": [
                {"name": "rxfull", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


