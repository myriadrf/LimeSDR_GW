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

   python3 -m boards.targets.limesdr_xtrx --build [--with-bios] [--load] [--flash] [--gold] [--cable <cable>]

**Options:**

- ``--with-bios``: Enables LiteX BIOS (requires additional resources).
- ``--load``: Loads the bitstream into SRAM (volatile memory).
- ``--flash``: Programs the bitstream into SPI FLASH memory.
- ``--cable``: Specifies the JTAG cable (default: *digilent_hs2*). Use ``openFPGALoader --list-cables`` for options.
- ``--gold``: Build/Flash golden image instead of user. 


User/Golden Bitstreams
----------------------

- The **User bitstream** is built using the default command above.
- The **Golden bitstream** is built using the ``--gold`` option:
.. code:: bash

   python3 -m boards.targets.limesdr_xtrx --build  --gold


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
