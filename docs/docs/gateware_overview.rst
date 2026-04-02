Gateware Overview
=================

This page is a quick entry point to LimeSDR gateware documentation. Use it to navigate architecture,
host interfaces, toolchains, and update workflow topics, then follow the linked pages for full
detail.

Recommended Reading Order
-------------------------

1. :doc:`Gateware Architecture <gateware_architecture>`: top-level subsystem layout (HIF/CPU/LimeTOP/PSS), control vs data plane, and interconnect model.
   
   Board-specific architecture/implementation pages:

   - :doc:`limesdr_xtrx`
   - :doc:`limesdr_mini_v1`
   - :doc:`limesdr_mini_v2`
   - :doc:`HiperSDR_44xx`
   - :doc:`ssdr_rev2`
2. :doc:`Gateware Toolchains <gateware_toolchains>` and :doc:`Firmware Toolchains <firmware_toolchains>`: required build environments for FPGA synthesis and CPU firmware compilation.
3. :doc:`Update and Recovery <update_and_recovery>`: flash update flow, multiboot behavior, and practical recovery/verification steps.

.. toctree::
   :maxdepth: 3
   :hidden:

   gateware_architecture
   gateware_toolchains
   firmware_toolchains
   update_and_recovery
   limesdr_xtrx
   limesdr_mini_v1
   limesdr_mini_v2
   HiperSDR_44xx
   ssdr_rev2
   addfeatures

Additional Features
-------------------

Some LimeSDR boards offer auxiliary features. See :doc:`addfeatures`.
