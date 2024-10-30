=========================
Modifying the project
=========================

Gateware
----------------------
Gateware sources are split among multiple folders. The folders are:

* **boards/targets** - python LiteX files containing board specific top level gateware descriptions
* **boards/platforms** - python LiteX files containing platform constraints, such as pin locations and IO standards
* **gateware** - module descriptions, hdl files

Example
^^^^^^^^^^^^^^^^^^^^^^
To make the process of adding a custom module to the process easier to understand, an example is provided for LimeSDR XTRX.
This example will add a fixed point FFT module in the data receive path, so that results of the fourier transform are packed into packets
instead of RF samples.

All sources required for the example can be found in **gateware/examples/fft**. The files contained there are:

* **fixedpointfft.py** - modified version of a fixed point FFT module from `amlib`_ repository.
* **fft.v** - verilog source file, pregenerated from **fixedpointfft.py**.
* **fft_wrap.vhd** - VHDL wrapper for **fft.v** with a basic AXI-STREAM interface.
* **LimeTop_fft.py** - modified version of regular **LimeTop.py** used in the project. Contains all changes discussed in this example.
* **limesdr_fft_samples.grc** - gnu radio file containg blocks that properly scale, shift and display FFT data received from the board.

If you are not interested in modifying the code yourself, but want to try out the FFT module, you can modify the **boards/targets/limesdr_xtrx.py** file to use the **LimeTop_fft** 
file instead of the regular **LimeTop** file like so:

.. code-block:: python

        # Lime Top Level -------------------------------------------------------------------
        # from gateware.LimeTop import LimeTop
        # fft example
        from gateware.examples.fft.LimeTop_fft import LimeTop

In order to calculate an FFT from received samples and pack resulting data into packets we must first identify the right location for the FFT module.
According to information found in `LimeSDR XTRX gateware description`_, raw samples are received by **lms7002_top** and passed to **rx_path_top** for packing.
That means that in order to reuse sample packing logic, the FFT module has to be inserted between **lms7002_top** and **rx_path_top**.

To avoid conflicting assignments, we must disconnect the **lms7002_top** master interface from the **rx_path_top** slave interface.
This is done by commenting the relevant *connect* command as seen in a code snippet below:

.. code-block:: python

        # RX Path
        self.rx_path = rx_path_top(platform)
        self.comb += self.rx_path.RESET_N.eq(self.lms7002.tx_en.storage)

        # Connect RX path AXIS slave to lms7002 AXIS master
        # The line below is commented to disconnect the RX path from the LMS7002
        # self.comb += self.lms7002.axis_m.connect(self.rx_path.s_axis_iqsmpls)
        self.comb += self.rx_path.s_axis_iqsmpls.areset_n.eq(self.lms7002.tx_en.storage)

The next step is to instantiate the fft wrapper and create two new AXI Stream interfaces for it.
The interface declarations can be copy-pasted from any other module. In this case it can be done like this:

.. code-block:: python

        # import AXIStreamInterface description
        from litex.soc.interconnect.axi import AXIStreamInterface
        # describe layouts for s_axis and m_axis interfaces for fft wrapper
        # definitions copied from rx_path_top to ensure same layout
        s_axis_layout = [("data", max(1, 64))]
        s_axis_layout += [("areset_n", 1)]
        s_axis_layout += [("keep", max(1, 64//8))]
        #
        m_axis_layout = [("data", max(1, 64))]
        m_axis_layout += [("areset_n", 1)]
        m_axis_layout += [("keep", max(1, 64//8))]
        # declare fft interfaces
        self.fft_s_axis = AXIStreamInterface(data_width=64, layout=s_axis_layout, clock_domain=self.lms7002.axis_m.clock_domain)
        self.fft_m_axis = AXIStreamInterface(data_width=64, layout=m_axis_layout, clock_domain=self.lms7002.axis_m.clock_domain)

Detailed instructions on how to instantiate a non-LiteX module in a LiteX project can be found in the `Litex documentation`_, 
in this example it is done like this:

.. code-block:: python

        # assign fft wrapper ports to appropriate interfaces
        self.fft_params = dict()
        self.fft_params.update(
            i_CLK = ClockSignal(self.lms7002.axis_m.clock_domain),
            i_RESET_N = self.lms7002.tx_en.storage,
            i_S_AXIS_TVALID = self.fft_s_axis.valid,
            i_S_AXIS_TDATA = self.fft_s_axis.data,
            o_S_AXIS_TREADY = self.fft_s_axis.ready,
            i_S_AXIS_TLAST = self.fft_s_axis.last,
            i_S_AXIS_TKEEP = self.fft_s_axis.keep,
            #
            o_M_AXIS_TDATA = self.fft_m_axis.data,
            o_M_AXIS_TVALID = self.fft_m_axis.valid,
            i_M_AXIS_TREADY = self.fft_m_axis.ready,
            o_M_AXIS_TLAST = self.fft_m_axis.last,
            o_M_AXIS_TKEEP = self.fft_m_axis.keep
        )
        # instantiate fft wrapper
        self.specials += Instance("fft_wrap", **self.fft_params)

Finally, the newly instantiated module needs to connected both to **lms7002_top** and **rx_path_top** modules. The syntax for that is 
the same as the connection between **lms7002_top** and **rx_path_top** that was commented out at the beginning of the example, except for the added *omit={"areset_n"}*,
because the fft wrapper does not have specified ports. The code can be seen below:

.. code-block:: python

        # connect the lms7002 master interface to the fft wrapper slave interface
        self.comb += self.lms7002.axis_m.connect(self.fft_s_axis,omit={"areset_n"})
        # connect the fft wrapper master interface to the rx_path slave interface
        self.comb += self.fft_m_axis.connect(self.rx_path.s_axis_iqsmpls,omit={"areset_n"})

After performing these modifications, build the project, and program the board, as described in :ref:`Building the project<docs/build_project:building and loading the gateware>`.

The FFT calculated by the module can be seen using the **limesdr_fft_samples.grc** file provided with the example.
To be able to use the file please make sure you have up to date versions of GNU Radio and LimeSuiteNG installed.


.. _amlib: https://github.com/amaranth-farm/amlib
.. _LimeSDR XTRX gateware description: https://limesdrgw.myriadrf.org/docs/limesdr_xtrx
.. _LiteX documentation: https://github.com/enjoy-digital/litex/wiki/Reuse-a-(System)Verilog,-VHDL,-Amaranth,-Spinal-HDL,-Chisel-core

Firmware
----------------------

The firmware sources can be found in the ``firmware`` folder. The firmware can be built
using the ``Makefile`` provided in the same folder.

In order to successfully compile, the gateware project needs to be built at least once to generate
required sources and headers.

When building gateware, the firmware gets compiled automatically, it is not required to compile it manually.

Debug tools
----------------------

**Firmware Debug through GDB over JTAG**

To build and load a gateware with a debug interface:

.. code:: bash

   ./limesdr_xtrx.py --with-bscan --build --load --flash

   # Load firmware through serial:
   litex_term /dev/ttyUSBx --kernel firmware/firmware.bin

   # Run OpenOCD with one of the specified configurations:
   openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl
   or
   openocd -f ./openocd_xc7_ft2232.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

   # Connect GDB for debugging:
   gdb-multiarch -q firmware/firmware.elf -ex "target extended-remote localhost:3333"

Note that instead of using GDB directly, Eclipse IDE can be configured
to debug code in a more user-friendly way. Follow this guide to
configure Eclipse IDE: `Using Eclipse to run and debug the
software <https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software>`__