LimeSDR Mini V1 build instructions 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To build the gateware for **limesdr_mini_v1** target, use:

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v1 --build
   
.. note::

   - Ensure required toolchain is installed and configured before building. See `Requirements <https://limesdrgw.myriadrf.org/docs/build_project#requirements>`_ section for respective board.  
   
   - Make sure to run build command from **project root directory**.

Available build options
-----------------------

.. code:: bash

   python3 -m boards.targets.limesdr_mini_v1 --build [--golden] [--flash] [--with-uartbone] [--cable <cable>]

**Options:**

- ``--golden``: to create the Golden bitstream (without RX/TX Path nor LMS7002 Modules to reduces size since it uses ROM)
- ``--flash``: to write Golden and User Bitstreams (requires to have both bitstreams build).
- ``--with-uartbone``: to enable uartbone in design.


User/Golden Bitstreams
----------------------

Since the Operational bitstream executes firmware from internal flash, the --load option is not supported.

To update the LimeSDR Mini v1, follow this sequence:

    1. Build the Golden bitstream (if not already done) using the ``--golden`` option.
    2. Build the Operational bitstream.
    3. After the previous step, the .rpd, .pof, and .svf files will be available.
    

Flashing Instructions
---------------------
- **Full Flash Image (User + Golden):**

  .. code:: bash
     
     python -m boards.targets.limesdr_mini_v1 --flash
     # Or
     openFPGALoader -c [cable] LimeSDR-Mini_lms7_trx_HW_1.2.svf





