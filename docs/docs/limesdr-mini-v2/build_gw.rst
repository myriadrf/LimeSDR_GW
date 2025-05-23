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

   python3 -m boards.targets.limesdr_mini_v2 --build [--load] [--flash] [--flash-user] [--flash-golden] [--toolchain=TOOLCHAIN] [--cable <cable>]

**Options:**

- ``--load``: Loads the bitstream into SRAM.
- ``--flash``: Writes MCS combo bitstream (User + Golden) to SPI Flash.
- ``--flash-user``: Writes the User bitstream to ``0x00000000``.
- ``--flash-golden``: Writes the Golden bitstream to ``0x00140000``.
- ``--toolchain=TOOLCHAIN``: Specify **trellis** or **diamond**  (default: **trellis**).
- ``--cable``: Specifies the JTAG cable (default: *ft2232*). Use ``openFPGALoader --list-cables`` for options.


User/Golden Bitstreams
-----------------------------

- The **User bitstream** is built using the commands above.
- The **Golden bitstream** is provided at: ``bitstream/LimeSDR_Mini_V2/``


After generating the User bitstream, the root directory contains:

- **limesdr_mini_v2.bin**: Composite image with both Golden and User bitstreams.
- **tools/limesdr_mini_v2.mcs**: Equivalent to ``limesdr_mini_v2.bin``, but in Intel Hex (iHex) format.

.. note::

	Due to limitations in **prjtrellis**, Lattice Diamond must be installed and in ``$PATH`` to generate ``tools/limesdr_mini_v2.mcs``.

Programming cables
------------------

For information about supported programming cables and required hardware connections, refer to the specific linked chapter in hardware documentation:

   - `Connecting LimeSDR Mini V2 board to FT2232H Mini Module <https://limesdr-mini.myriadrf.org/documentation/jtag-programming#connecting-limesdr-mini-v2-board-to-ft2232h-mini-module>`_

.. note::
   Only the specified chapter related to the FT2232H Mini Module and hardware connections is applicable to board programming with this project. Other parts of the documentation are not relevant.

  

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


