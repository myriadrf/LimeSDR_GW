lms7_trx_files = [
    # SPI / CFG.
    # ----------
    "gateware/hdl/spi/fpgacfg_pkg.vhd",
    "gateware/hdl/spi/mcfg_components.vhd",
    "gateware/hdl/spi/periphcfg_pkg.vhd",
    "gateware/hdl/spi/tstcfg_pkg.vhd",

    # Packages.
    # ---------
    "gateware/hdl/packages/synth/FIFO_PACK.vhd",

    # General.
    # --------
    "gateware/hdl/general/sync_reg.vhd",
    "gateware/hdl/general/bus_sync_reg.vhd",
    "gateware/hdl/general/busy_delay.vhd",

    # Self-Test.
    # ----------
    "gateware/hdl/self_test/transition_count.vhd",
    "gateware/hdl/self_test/singl_clk_with_ref_test.vhd",
    "gateware/hdl/self_test/clk_with_ref_test.vhd",
    "gateware/hdl/self_test/clk_no_ref_test.vhd",

    # Altera-Inst.
    # ------------
    "gateware/hdl/altera_inst/lpm_cnt_inst.vhd",
    "gateware/hdl/altera_inst/fifo_inst.vhd",
    "gateware/hdl/altera_inst/lpm_compare_inst.vhd",

    # TX-IQ-Mux.
    # ----------
    "gateware/hdl/txiqmux/synth/txiqmux.vhd",
    "gateware/hdl/txiqmux/synth/txiq_tst_ptrn.vhd",
]
