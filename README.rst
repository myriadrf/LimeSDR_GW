LimeSDR Gateware
================

This project focuses on the development and customization of FPGA
gateware for LimeSDR boards using the LiteX framework.

Tagged Releases
---------------
All stable and fully tested work is available as tagged releases. If you need a reliable version, always refer to the latest release tag.

Available branches
------------------

-  **master** - Most features are tested, but some bugs may still be present. Used as the main integration branch before creating releases.
-  **develop** - Active development branch. Only the functionality currently under development is tested. May be unstable and is updated frequently.
-  **other** - Development sandboxes for experiments or feature work. No guarantees of testing or stability.


Documentation
-------------

More details can be found in:

-  https://limesdrgw.myriadrf.org/

Local documentation can be build (Sphinx documentation generator required):

.. code:: bash

	cd docs/
	python3.8 -m venv venv
	source venv/bin/activate
	pip install -r requirements.txt
	make html
