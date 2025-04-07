LimeSDR Mini V2 build instructions 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To build the gateware for **limesdr_mini_v2** target, use:

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v2 --build
   

.. note::

   - Ensure required toolchain is installed and configured before building. See `Requirements <https://limesdrgw.myriadrf.org/docs/build_project#requirements>`_ section for respective board.  
   
   - Make sure to run build command from **project root directory**.

Available build options
-----------------------

**Comand:**

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v2 --build [--with-bios] [--without-spi-flash] [--load] [--flash] [--flash-user] [--flash-golden] [--toolchain=TOOLCHAIN] [--cable <cable>]

**Options:**

- ``--toolchain=TOOLCHAIN``: Specify **diamond** or **trellis** (default: **trellis**).
- ``--with-bios``: Enables LiteX BIOS.
- ``--without-spi-flash``: Disables SPI Flash support (*only for trellis toolchain*).
- ``--load``: Loads the bitstream into SRAM.
- ``--flash``: Writes MCS combo bitstream (User + Golden) to SPI Flash.
- ``--flash-user``: Writes the User bitstream to ``0x00000000``.
- ``--flash-golden``: Writes the Golden bitstream to ``0x00140000``.
- ``--cable``: Specifies the JTAG cable (default: *ft2232*). Use ``openFPGALoader --list-cables`` for options.


User/Golden Bitstreams
-----------------------------

- The **User bitstream** is built using the commands above.
- The **Golden bitstream** is provided at: ``tools/limesdr_mini_v2_golden.bit``


After generating the User bitstream, the root directory contains:

- **limesdr_mini_v2.bin**: Composite image with both Golden and User bitstreams.
- **tools/limesdr_mini_v2.mcs**: Equivalent to ``limesdr_mini_v2.bin``, but in Intel Hex (iHex) format.

.. note::

	Due to limitations in **prjtrellis**, Lattice Diamond must be installed and in ``$PATH`` to generate ``tools/limesdr_mini_v2.mcs``.

Flashing Instructions
---------------------

To write bitstreams to SPI Flash:

- **Full Flash Image (User + Golden):**

  .. code:: bash
     
     python -m boards.targets.limesdr_mini_v2 --flash

- **User Bitstream Only:**

  .. code:: bash
     
     python -m boards.targets.limesdr_mini_v2 --flash-user

- **Golden Bitstream Only:**

  .. code:: bash
     
     python -m boards.targets.limesdr_mini_v2 --flash-golden


