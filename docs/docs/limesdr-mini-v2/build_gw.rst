LimeSDR Mini V2 Build Instructions
==================================

To build gateware for the **limesdr_mini_v2** target, first activate the virtual environment and then run the build command:

.. code:: bash

   source .venv/bin/activate
   python3 -m boards.targets.limesdr_mini_v2 --build

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the board-specific requirements.

   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

**Command:**

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v2 --build [--load] [--flash] [--flash-user] [--flash-golden] [--toolchain=TOOLCHAIN] [--cable <cable>]

**Options:**

- ``--load``: Load the bitstream into SRAM.
- ``--flash``: Program the combined user and golden image into SPI flash.
- ``--flash-user``: Program the user bitstream at address ``0x00000000``.
- ``--flash-golden``: Program the golden bitstream at address ``0x00140000``.
- ``--toolchain=TOOLCHAIN``: Select ``trellis`` or ``diamond``. The default is ``trellis``.
- ``--cable <cable>``: Specify the JTAG cable. The default is ``ft2232``.
  Use ``openFPGALoader --list-cables`` to list supported cable names.

User and Golden Bitstreams
--------------------------

- The **user bitstream** is built using the commands above.
- The **golden bitstream** is located in ``bitstream/LimeSDR_Mini_V2/``.

After generating the user bitstream, the following files are available:

- **limesdr_mini_v2.bin**: Combined image containing both the golden and user bitstreams.
- **tools/limesdr_mini_v2.mcs**: The same image in Intel Hex (iHex) format.

.. note::

   Due to limitations in **prjtrellis**, Lattice Diamond must be installed and available in
   ``$PATH`` to generate ``tools/limesdr_mini_v2.mcs``.

Programming Cables
------------------

For supported programming cables and required hardware connections, refer to the following hardware
documentation section:

- `Connecting LimeSDR Mini V2 board to FT2232H Mini Module <https://limesdr-mini.myriadrf.org/documentation/jtag-programming#connecting-limesdr-mini-v2-board-to-ft2232h-mini-module>`_

.. note::

   Only the section covering the FT2232H Mini Module and the required hardware connections is
   relevant for board programming with this project.

Flashing Instructions
---------------------

To write bitstreams to SPI flash:

- **Full flash image (user + golden):**

  .. code:: bash

     python3 -m boards.targets.limesdr_mini_v2 --flash

- **User bitstream only:**

  .. code:: bash

     python3 -m boards.targets.limesdr_mini_v2 --flash-user

- **Golden bitstream only:**

  .. code:: bash

     python3 -m boards.targets.limesdr_mini_v2 --flash-golden