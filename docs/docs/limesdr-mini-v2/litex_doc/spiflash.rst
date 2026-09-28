SPIFLASH
========

Register Listing for SPIFLASH
-----------------------------

+------------------------------------------------------------------+-------------------------------------------------+
| Register                                                         | Address                                         |
+==================================================================+=================================================+
| :ref:`SPIFLASH_PHY_CLK_DIVISOR <SPIFLASH_PHY_CLK_DIVISOR>`       | :ref:`0xf0004000 <SPIFLASH_PHY_CLK_DIVISOR>`    |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_PHY_MODE <SPIFLASH_PHY_MODE>`                     | :ref:`0xf0004004 <SPIFLASH_PHY_MODE>`           |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MMAP_CLK_DIVISOR <SPIFLASH_MMAP_CLK_DIVISOR>`     | :ref:`0xf0004008 <SPIFLASH_MMAP_CLK_DIVISOR>`   |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MMAP_DUMMY_BITS <SPIFLASH_MMAP_DUMMY_BITS>`       | :ref:`0xf000400c <SPIFLASH_MMAP_DUMMY_BITS>`    |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MASTER_CS <SPIFLASH_MASTER_CS>`                   | :ref:`0xf0004010 <SPIFLASH_MASTER_CS>`          |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MASTER_PHYCONFIG <SPIFLASH_MASTER_PHYCONFIG>`     | :ref:`0xf0004014 <SPIFLASH_MASTER_PHYCONFIG>`   |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MASTER_RXTX <SPIFLASH_MASTER_RXTX>`               | :ref:`0xf0004018 <SPIFLASH_MASTER_RXTX>`        |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MASTER_STATUS <SPIFLASH_MASTER_STATUS>`           | :ref:`0xf000401c <SPIFLASH_MASTER_STATUS>`      |
+------------------------------------------------------------------+-------------------------------------------------+
| :ref:`SPIFLASH_MASTER_CLK_DIVISOR <SPIFLASH_MASTER_CLK_DIVISOR>` | :ref:`0xf0004020 <SPIFLASH_MASTER_CLK_DIVISOR>` |
+------------------------------------------------------------------+-------------------------------------------------+

SPIFLASH_PHY_CLK_DIVISOR
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x0 = 0xf0004000`


    .. wavedrom::
        :caption: SPIFLASH_PHY_CLK_DIVISOR

        {
            "reg": [
                {"name": "phy_clk_divisor[9:0]", "attr": 'reset: 776', "bits": 10},
                {"bits": 22},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


SPIFLASH_PHY_MODE
^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x4 = 0xf0004004`

    SPI mode (CPOL/CPHA). Curently only mode 0 and 3 are supported.

    .. wavedrom::
        :caption: SPIFLASH_PHY_MODE

        {
            "reg": [
                {"name": "phy_mode[1:0]", "bits": 2},
                {"bits": 30},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


SPIFLASH_MMAP_CLK_DIVISOR
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x8 = 0xf0004008`


    .. wavedrom::
        :caption: SPIFLASH_MMAP_CLK_DIVISOR

        {
            "reg": [
                {"name": "mmap_clk_divisor[9:0]", "bits": 10},
                {"bits": 22},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


SPIFLASH_MMAP_DUMMY_BITS
^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0xc = 0xf000400c`


    .. wavedrom::
        :caption: SPIFLASH_MMAP_DUMMY_BITS

        {
            "reg": [
                {"name": "mmap_dummy_bits[7:0]", "bits": 8},
                {"bits": 24},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


SPIFLASH_MASTER_CS
^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x10 = 0xf0004010`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_CS

        {
            "reg": [
                {"name": "master_cs", "bits": 1},
                {"bits": 31},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


SPIFLASH_MASTER_PHYCONFIG
^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x14 = 0xf0004014`

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
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
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

`Address: 0xf0004000 + 0x18 = 0xf0004018`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_RXTX

        {
            "reg": [
                {"name": "master_rxtx[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


SPIFLASH_MASTER_STATUS
^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x1c = 0xf000401c`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_STATUS

        {
            "reg": [
                {"name": "tx_ready",  "bits": 1},
                {"name": "rx_ready",  "bits": 1},
                {"bits": 30}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 4 }, "options": {"hspace": 400, "bits": 32, "lanes": 4}
        }


+-------+----------+-----------------------+
| Field | Name     | Description           |
+=======+==========+=======================+
| [0]   | TX_READY | TX FIFO is not full.  |
+-------+----------+-----------------------+
| [1]   | RX_READY | RX FIFO is not empty. |
+-------+----------+-----------------------+

SPIFLASH_MASTER_CLK_DIVISOR
^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Address: 0xf0004000 + 0x20 = 0xf0004020`


    .. wavedrom::
        :caption: SPIFLASH_MASTER_CLK_DIVISOR

        {
            "reg": [
                {"name": "master_clk_divisor[9:0]", "bits": 10},
                {"bits": 22},
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


