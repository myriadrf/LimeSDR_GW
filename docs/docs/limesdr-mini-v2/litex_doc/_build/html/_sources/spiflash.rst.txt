SPIFLASH
========

Register Listing for SPIFLASH
-----------------------------

+--------------------------------------------------------------+-----------------------------------------------+
| Register                                                     | Address                                       |
+==============================================================+===============================================+
| :ref:`SPIFLASH_PHY_CLK_DIVISOR <SPIFLASH_PHY_CLK_DIVISOR>`   | :ref:`0xf0003000 <SPIFLASH_PHY_CLK_DIVISOR>`  |
+--------------------------------------------------------------+-----------------------------------------------+
| :ref:`SPIFLASH_MMAP_DUMMY_BITS <SPIFLASH_MMAP_DUMMY_BITS>`   | :ref:`0xf0003004 <SPIFLASH_MMAP_DUMMY_BITS>`  |
+--------------------------------------------------------------+-----------------------------------------------+
| :ref:`SPIFLASH_MASTER_CS <SPIFLASH_MASTER_CS>`               | :ref:`0xf0003008 <SPIFLASH_MASTER_CS>`        |
+--------------------------------------------------------------+-----------------------------------------------+
| :ref:`SPIFLASH_MASTER_PHYCONFIG <SPIFLASH_MASTER_PHYCONFIG>` | :ref:`0xf000300c <SPIFLASH_MASTER_PHYCONFIG>` |
+--------------------------------------------------------------+-----------------------------------------------+
| :ref:`SPIFLASH_MASTER_RXTX <SPIFLASH_MASTER_RXTX>`           | :ref:`0xf0003010 <SPIFLASH_MASTER_RXTX>`      |
+--------------------------------------------------------------+-----------------------------------------------+
| :ref:`SPIFLASH_MASTER_STATUS <SPIFLASH_MASTER_STATUS>`       | :ref:`0xf0003014 <SPIFLASH_MASTER_STATUS>`    |
+--------------------------------------------------------------+-----------------------------------------------+

SPIFLASH_PHY_CLK_DIVISOR
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003000 + 0x0 = 0xf0003000`


    .. wavedrom::
        :caption: SPIFLASH_PHY_CLK_DIVISOR

        {
            "reg": [
                {"name": "phy_clk_divisor[7:0]", "attr": 'reset: 387', "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


SPIFLASH_MMAP_DUMMY_BITS
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003000 + 0x4 = 0xf0003004`


    .. wavedrom::
        :caption: SPIFLASH_MMAP_DUMMY_BITS

        {
            "reg": [
                {"name": "mmap_dummy_bits[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


SPIFLASH_MASTER_CS
^^^^^^^^^^^^^^^^^^

`Address: 0xf0003000 + 0x8 = 0xf0003008`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_CS

        {
            "reg": [
                {"name": "master_cs", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


SPIFLASH_MASTER_PHYCONFIG
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003000 + 0xc = 0xf000300c`

    SPI PHY settings.

    .. wavedrom::
        :caption: SPIFLASH_MASTER_PHYCONFIG

        {
            "reg": [
                {"name": "len",  "bits": 8},
                {"name": "width",  "bits": 4},
                {"bits": 4},
                {"name": "mask",  "bits": 8},
                {"bits": 8}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+---------+-------+-----------------------------------------------------------------------------+
| Field   | Name  | Description                                                                 |
+=========+=======+=============================================================================+
| [7:0]   | LEN   | SPI Xfer length (in bits).                                                  |
+---------+-------+-----------------------------------------------------------------------------+
| [11:8]  | WIDTH | SPI Xfer width (1/2/4/8).                                                   |
+---------+-------+-----------------------------------------------------------------------------+
| [23:16] | MASK  | SPI DQ output enable mask (set bits to ``1`` to enable output drivers on DQ |
|         |       | lines).                                                                     |
+---------+-------+-----------------------------------------------------------------------------+

SPIFLASH_MASTER_RXTX
^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003000 + 0x10 = 0xf0003010`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_RXTX

        {
            "reg": [
                {"name": "master_rxtx[31:0]", "bits": 32}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 1 }, "options": {"hspace": 900, "bits": 32, "lanes": 1}
        }


SPIFLASH_MASTER_STATUS
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0003000 + 0x14 = 0xf0003014`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_STATUS

        {
            "reg": [
                {"name": "tx_ready",  "bits": 1},
                {"name": "rx_ready",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 900, "bits": 32, "lanes": 4 }, "options": {"hspace": 900, "bits": 32, "lanes": 4}
        }


+-------+----------+-----------------------+
| Field | Name     | Description           |
+=======+==========+=======================+
| [0]   | TX_READY | TX FIFO is not full.  |
+-------+----------+-----------------------+
| [1]   | RX_READY | RX FIFO is not empty. |
+-------+----------+-----------------------+

