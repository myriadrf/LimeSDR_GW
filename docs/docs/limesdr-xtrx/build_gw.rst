LimeSDR XTRX build instructions 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


To build the gateware for **limesdr_xtrx** target, use command:

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build
   
.. note::

   - Ensure required toolchain is installed and configured before building. See `Requirements <https://limesdrgw.myriadrf.org/docs/build_project#requirements>`_ section for respective board.  
   
   - Make sure to run build command from **project root directory**.
  
Available build options
-----------------------

**Comand:**

.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build [--load] [--flash] [--gold] [--cable <cable>]

**Options:**

- ``--load``: Loads the bitstream into SRAM (volatile memory).
- ``--flash``: Programs the bitstream into SPI FLASH memory.
- ``--gold``: Build/Flash golden image instead of user.
- ``--cable``: Specifies the JTAG cable (default: *ft2232*). Use ``openFPGALoader --list-cables`` for options.
 

User/Golden Bitstreams
----------------------

- The **User bitstream** is built using the default command above.
- The **Golden bitstream** is built using the ``--gold`` option:
.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build  --gold


Programming cables
------------------

For information about supported programming cables and required hardware connections, refer to the specific linked chapter in hardware documentation:

   - `FT2232H Mini Module JTAG adapter <https://limesdr-xtrx.myriadrf.org/documentation/jtag-programming#jtag-programming-openfpgaloader>`_

.. note::
   Only the specified chapter related to the FT2232H Mini Module and hardware connections is applicable to board programming with this project. Other parts of the documentation are not relevant.

   
Flashing Instructions
---------------------
- **User Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.llimesdr_xtrx --flash

- **Golden Bitstream Only:**

  .. code:: bash
     
     python3 -m boards.targets.llimesdr_xtrx --flash --gold
     
     
.. note::

	User/Gold Bitstreams has to be present/generated first before flashing
