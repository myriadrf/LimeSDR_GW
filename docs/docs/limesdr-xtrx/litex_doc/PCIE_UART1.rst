PCIE_UART1
==========

Register Listing for PCIE_UART1
-------------------------------

+------------------------------------------------------+-------------------------------------------+
| Register                                             | Address                                   |
+======================================================+===========================================+
| :ref:`PCIE_UART1_RXTX <PCIE_UART1_RXTX>`             | :ref:`0xf0007000 <PCIE_UART1_RXTX>`       |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_TXFULL <PCIE_UART1_TXFULL>`         | :ref:`0xf0007004 <PCIE_UART1_TXFULL>`     |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_RXEMPTY <PCIE_UART1_RXEMPTY>`       | :ref:`0xf0007008 <PCIE_UART1_RXEMPTY>`    |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_EV_STATUS <PCIE_UART1_EV_STATUS>`   | :ref:`0xf000700c <PCIE_UART1_EV_STATUS>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_EV_PENDING <PCIE_UART1_EV_PENDING>` | :ref:`0xf0007010 <PCIE_UART1_EV_PENDING>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_EV_ENABLE <PCIE_UART1_EV_ENABLE>`   | :ref:`0xf0007014 <PCIE_UART1_EV_ENABLE>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_TXEMPTY <PCIE_UART1_TXEMPTY>`       | :ref:`0xf0007018 <PCIE_UART1_TXEMPTY>`    |
+------------------------------------------------------+-------------------------------------------+
| :ref:`PCIE_UART1_RXFULL <PCIE_UART1_RXFULL>`         | :ref:`0xf000701c <PCIE_UART1_RXFULL>`     |
+------------------------------------------------------+-------------------------------------------+

PCIE_UART1_RXTX
^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x0 = 0xf0007000`


    .. wavedrom::
        :caption: PCIE_UART1_RXTX

        {
            "reg": [
                {"name": "rxtx[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


PCIE_UART1_TXFULL
^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x4 = 0xf0007004`

    TX FIFO Full.

    .. wavedrom::
        :caption: PCIE_UART1_TXFULL

        {
            "reg": [
                {"name": "txfull", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_UART1_RXEMPTY
^^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x8 = 0xf0007008`

    RX FIFO Empty.

    .. wavedrom::
        :caption: PCIE_UART1_RXEMPTY

        {
            "reg": [
                {"name": "rxempty", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_UART1_EV_STATUS
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0xc = 0xf000700c`

    This register contains the current raw level of the rx event trigger.  Writes to
    this register have no effect.

    .. wavedrom::
        :caption: PCIE_UART1_EV_STATUS

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

PCIE_UART1_EV_PENDING
^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x10 = 0xf0007010`

    When a  rx event occurs, the corresponding bit will be set in this register.  To
    clear the Event, set the corresponding bit in this register.

    .. wavedrom::
        :caption: PCIE_UART1_EV_PENDING

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

PCIE_UART1_EV_ENABLE
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x14 = 0xf0007014`

    This register enables the corresponding rx events.  Write a ``0`` to this
    register to disable individual events.

    .. wavedrom::
        :caption: PCIE_UART1_EV_ENABLE

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

PCIE_UART1_TXEMPTY
^^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x18 = 0xf0007018`

    TX FIFO Empty.

    .. wavedrom::
        :caption: PCIE_UART1_TXEMPTY

        {
            "reg": [
                {"name": "txempty", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


PCIE_UART1_RXFULL
^^^^^^^^^^^^^^^^^

`Address: 0xf0007000 + 0x1c = 0xf000701c`

    RX FIFO Full.

    .. wavedrom::
        :caption: PCIE_UART1_RXFULL

        {
            "reg": [
                {"name": "rxfull", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


