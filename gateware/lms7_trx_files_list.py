lms7_trx_files = [
    # SPI / CFG.
    # ----------
    "LimeSDR-Mini_lms7_trx/src/spi/cfg_top.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/fpgacfg.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/fpgacfg_pkg.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/mcfg_components.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/mcfg32wm_fsm.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/mem_package.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/periphcfg.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/periphcfg_pkg.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/pllcfg_pkg.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/tstcfg.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/tstcfg_pkg.vhd",

    # Packages.
    # ---------
    "LimeSDR-Mini_lms7_trx/src/packages/synth/FIFO_PACK.vhd",

    # Revision.
    # ---------
    "LimeSDR-Mini_lms7_trx/src/revision/revision.vhd",

    # PLL.
    # ----
    "LimeSDR-Mini_lms7_trx/src/pll_top/synth/pll_top.vhd",
    "LimeSDR-Mini_lms7_trx/src/spi/pllcfg.vhd",

    # General.
    # --------
    "LimeSDR-Mini_lms7_trx/src/general/sync_reg.vhd",
    "LimeSDR-Mini_lms7_trx/src/general/bus_sync_reg.vhd",
    "LimeSDR-Mini_lms7_trx/src/general/busy_delay.vhd",

    # Self-Test.
    # ----------
    "LimeSDR-Mini_lms7_trx/src/self_test/transition_count.vhd",
    "LimeSDR-Mini_lms7_trx/src/self_test/singl_clk_with_ref_test.vhd",
    "LimeSDR-Mini_lms7_trx/src/self_test/clk_with_ref_test.vhd",
    "LimeSDR-Mini_lms7_trx/src/self_test/clk_no_ref_test.vhd",

    # Altera-Inst.
    # ------------
    "LimeSDR-Mini_lms7_trx/src/altera_inst/lpm_cnt_inst.vhd",
    "LimeSDR-Mini_lms7_trx/src/altera_inst/fifo_inst.vhd",
    "LimeSDR-Mini_lms7_trx/src/altera_inst/lpm_compare_inst.vhd",

    # TX-IQ-Mux.
    # ----------
    "LimeSDR-Mini_lms7_trx/src/txiqmux/synth/txiqmux.vhd",
    "LimeSDR-Mini_lms7_trx/src/txiqmux/synth/txiq_tst_ptrn.vhd",

    # Delay-Ctrl.
    # ----------
    "LimeSDR-Mini_lms7_trx/src/delayf_ctrl/delay_ctrl_fsm.vhd",
    "LimeSDR-Mini_lms7_trx/src/delayf_ctrl/delay_ctrl_top.vhd",
    "LimeSDR-Mini_lms7_trx/src/delayf_ctrl/delayf_ctrl.vhd",

]

lms7_trx_ips = [
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w64x256_r64/fifodc_w64x256_r64.sbx",     # sync_fifo_rw.vhd.
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w32x1024_r128/fifodc_w32x1024_r128.sbx", # FT601_top.vhd.
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w128x256_r128/fifodc_w128x256_r128.sbx", # one_pct_fifo.vhd.
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w128x256_r64/fifodc_w128x256_r64.sbx",   # packets2data.vhd.
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w48x1024_r48/fifodc_w48x1024_r48.sbx",   # rx_path_top.vhd.
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w64x2024_r32/fifodc_w64x2024_r32.sbx",   # FT601_top.vhd.
    "LimeSDR-Mini_lms7_trx/proj/ip/fifodc_w32x256_r32/fifodc_w32x256_r32.sbx",     # FT601_top.vhd.
]
