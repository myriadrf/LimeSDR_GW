LimeSDR Gateware
================

This project focuses on the development and customization of FPGA
gateware for LimeSDR boards using the LiteX framework.

Available branches
------------------

-  **master** - most recent stable and tested work


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
