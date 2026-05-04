Firmware Toolchains
===================

RISC-V Firmware Toolchains
--------------------------

For soft RISC-V CPUs (e.g., VexRiscv or PicoRV32), a RISC-V toolchain compiles the firmware (BIOS
or application). LiteX simplifies installation and management.

The recommended toolchain is **riscv64-unknown-elf-gcc** with newlib (no OS), targeting `rv32im` or
`rv32ima` based on CPU features.

Installation
------------

The RISC-V toolchain should be installed following the official `LiteX installation instructions <https://github.com/enjoy-digital/litex>`_ as mentioned in the :ref:`quick_start` guide.

.. note::
   While LiteX libraries are managed via `./setup_litex.sh`, the RISC-V compiler (GCC) must be installed separately and available in your ``$PATH``.

LiteX handles firmware compilation, linker scripts, and Board Support Packages (BSPs) for the
selected CPU, embedding the binary into the FPGA bitstream. Override defaults with `--riscv-cpu`
and `--cpu-variant` options if needed.
