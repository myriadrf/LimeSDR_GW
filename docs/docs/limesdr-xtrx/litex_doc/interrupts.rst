Interrupt Controller
====================

This device has an ``EventManager``-based interrupt system.  Individual modules
generate `events` which are wired into a central interrupt controller.

When an interrupt occurs, you should look the interrupt number up in the CPU-
specific interrupt table and then call the relevant module.

Assigned Interrupts
-------------------

The following interrupts are assigned on this system:

+-----------+----------------------------+
| Interrupt | Module                     |
+===========+============================+
| 3         | :doc:`CNTRL <CNTRL>`       |
+-----------+----------------------------+
| 4         | :doc:`LIME_TOP <lime_top>` |
+-----------+----------------------------+
| 0         | :doc:`NOIRQ <noirq>`       |
+-----------+----------------------------+
| 2         | :doc:`TIMER0 <timer0>`     |
+-----------+----------------------------+
| 1         | :doc:`UART <uart>`         |
+-----------+----------------------------+

