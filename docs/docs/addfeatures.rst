Additional features
===================

Beyond standard data streaming, specific LimeSDR boards offer a range of auxiliary features. This page serves as a central reference for these capabilities, including a hardware compatibility table and links to detailed technical documentation.

Compatibility table
-------------------

.. toctree::
   :maxdepth: 3
   :hidden:

   UTC timestamping <utc_timestamps>
   Synchronized stream start <en_delay>
   PPS Disciplined Oscillator (PPSDO) <ppsdo>

**Note:** This table specifies the *earliest* gw version that implements the feature for a specific board.
Later GW versions might have improvements and/or bug fixes, it is recommended to use the latest version, if possible.


+------------------------------------+---------------------------------------------------------------------------------------------+
| **Feature**                        | **Supported Boards and GW versions**                                                        |
+                                    +--------------+-----------------+-----------------+--------------+---------------+-----------+
|                                    | LimeSDR XTRX | LimeSDR Mini V1 | LimeSDR Mini V2 | LimeSDR USB  | HiperSDR 44xx | sSDR rev2 |
+====================================+==============+=================+=================+==============+===============+===========+
| :doc:`UTC timestamping             | v3.1         | --              | --              | --           | --            | --        |
| <utc_timestamps>`                  |              |                 |                 |              |               |           |
+------------------------------------+--------------+-----------------+-----------------+--------------+---------------+-----------+
| :doc:`Synchronized stream start    | v3.1         | --              | --              | --           | v3.10         | v3.3      |
| <en_delay>`                        |              |                 |                 |              |               |           |
+------------------------------------+--------------+-----------------+-----------------+--------------+---------------+-----------+
| :doc:`PPS Disciplined Oscillator   | v3.4         | --              | v3.4            | v3.13        | v3.6          | --        |
| (PPSDO) <ppsdo>`                   |              |                 |                 |              |               |           |
+------------------------------------+--------------+-----------------+-----------------+--------------+---------------+-----------+
