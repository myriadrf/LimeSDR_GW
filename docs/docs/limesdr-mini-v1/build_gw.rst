LimeSDR Mini V1 Build Instructions
==================================

To build gateware for the **limesdr_mini_v1** target, run:

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v1 --build

.. note::

   - Ensure that the required toolchain is installed and configured before building. See
     :ref:`Requirements <requirements>` for the board-specific requirements.

   - Run the build command from the **project root directory**.

Available Build Options
-----------------------

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v1 --build [--golden] [--flash] [--with-uartbone] [--cable <cable>]

**Options:**

- ``--golden``: Build the golden bitstream. This image excludes the RX/TX path and LMS7002-related
  modules to reduce size.
- ``--flash``: Program both the golden and user bitstreams. Both bitstreams must be built first.
- ``--with-uartbone``: Enable UARTBone in the design.
- ``--cable <cable>``: Specify the programming cable, if required by the selected flash flow.

.. note::

   As of release 3.0:

   - The golden image must be built at least once before building the user image.
   - The ``--flash`` option works only when ``--golden`` is not used.

User and Golden Bitstreams
--------------------------

The ``--load`` option is not supported, because the user bitstream executes firmware from internal
flash.

To update the LimeSDR Mini V1, use the following sequence:

1. Build the golden bitstream, if it has not already been generated, using the ``--golden`` option.
2. Build the user bitstream.
3. After the build completes, the generated output files include ``.rpd``, ``.pof``, and ``.svf``.

Programming Cables
------------------

For supported programming cables and hardware connection details, refer to the following hardware
documentation section:

- `Connecting LimeSDR Mini V2 board to FT2232H Mini Module <https://limesdr-mini.myriadrf.org/documentation/jtag-programming#connecting-limesdr-mini-v2-board-to-ft2232h-mini-module>`_

.. note::

   - The programming-cable guidance is the same for **LimeSDR Mini V1** and **LimeSDR Mini V2**.
   - For this project, only the FT2232H Mini Module connection information from the linked hardware
     documentation is relevant.

Flashing Instructions
---------------------

- **Full flash image (user + golden):**

  .. code:: bash

     python3 -m boards.targets.limesdr_mini_v1 --flash

  Or:

  .. code:: bash

     openFPGALoader -c <cable> LimeSDR-Mini_lms7_trx_HW_1.2.svf