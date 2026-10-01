################################################################################
# Asynchronous clocks
################################################################################

set_clock_groups -asynchronous \
	-group [get_clocks -nowarn {SI_CLK0}] \
	-group [get_clocks -nowarn {SI_CLK1}] \
	-group [get_clocks -nowarn {SI_CLK2}] \
	-group [get_clocks -nowarn {SI_CLK3}] \
	-group [get_clocks -nowarn {SI_CLK5}] \
	-group [get_clocks -nowarn {SI_CLK6}] \
	-group [get_clocks -nowarn {SI_CLK7}] \
	-group [get_clocks -nowarn {LMK_CLK}] \
	-group [get_clocks -nowarn {altera_reserved_tck}] \
	-group [get_clocks -nowarn {FPGA_GPIO_7}] \
	-group [get_clocks -nowarn {TX_PLLCLK_C0 TX_PLLCLK_C1 LMS_FCLK1_PLL *tx_pll_top*|*|pll1|clk[0] *tx_pll_top*|*|pll1|clk[1] *tx_pll_top*clk[0]* *tx_pll_top*clk[1]*}] \
	-group [get_clocks -nowarn {LMS_MCLK1 LMS_MCLK1_5MHZ LMS_MCLK1_VIRT LMS_MCLK1_VIRT_5MHz LMS_FCLK1_DRCT}] \
	-group [get_clocks -nowarn {RX_PLLCLK_C0 RX_PLLCLK_C1 LMS_FCLK2_PLL *rx_pll_top*|*|pll1|clk[0] *rx_pll_top*|*|pll1|clk[1] *rx_pll_top*clk[0]* *rx_pll_top*clk[1]*}] \
	-group [get_clocks -nowarn {LMS_MCLK2 LMS_MCLK2_5MHZ LMS_MCLK2_VIRT LMS_MCLK2_VIRT_5MHz LMS_FCLK2_DRCT}] \
	-group [get_clocks -nowarn {FX3_PCLK FX3_PCLK_VIRT FX3_PCLK_VIRT_OUT FPGA_SPI0_SCLK_reg FPGA_SPI0_SCLK_out FPGA_SPI1_SCLK BRDG_SPI_clk}] \
	-group [get_clocks -nowarn {*wfm_player_top* *wfm_player* *DDR2_ctrl* *ddram0*}] \
	-group [get_clocks -nowarn {*ddr2_tester* *ddram1*}]
											