################################################################################
# Asynchronous clocks
################################################################################

set group_count 0
set cmd "set_clock_groups -asynchronous"

proc add_clock_group {pattern_list} {
	global cmd group_count
	set clk_col [get_clocks -nowarn $pattern_list]
	if {[get_collection_size $clk_col] > 0} {
		set clk_names [list]
		foreach_in_collection c $clk_col {
			set cname [get_clock_info -name $c]
			if {[lsearch -exact $clk_names $cname] == -1} {
				lappend clk_names $cname
			}
		}
		if {[llength $clk_names] > 0} {
			append cmd " -group {" [join $clk_names " "] "}"
			incr group_count
		}
	}
}

# Base clocks
add_clock_group {SI_CLK0}
add_clock_group {SI_CLK1}
add_clock_group {SI_CLK2}
add_clock_group {SI_CLK3}
add_clock_group {SI_CLK5}
add_clock_group {SI_CLK6}
add_clock_group {SI_CLK7}
add_clock_group {LMK_CLK}

# LMS RF Transceiver clock domains
add_clock_group {LMS_MCLK1 LMS_MCLK1_VIRT}
add_clock_group {LMS_MCLK1_5MHZ LMS_MCLK1_VIRT_5MHz}
add_clock_group {TX_PLLCLK_C0 *tx_pll_top*|*|pll1|clk[0] *tx_pll_top*clk[0]*}
add_clock_group {TX_PLLCLK_C1 LMS_FCLK1_PLL *tx_pll_top*|*|pll1|clk[1] *tx_pll_top*clk[1]*}
add_clock_group {LMS_FCLK1_DRCT}

add_clock_group {LMS_MCLK2 LMS_MCLK2_VIRT}
add_clock_group {LMS_MCLK2_5MHZ LMS_MCLK2_VIRT_5MHz}
add_clock_group {RX_PLLCLK_C0 *rx_pll_top*|*|pll1|clk[0] *rx_pll_top*clk[0]*}
add_clock_group {RX_PLLCLK_C1 *rx_pll_top*|*|pll1|clk[1] *rx_pll_top*clk[1]*}
add_clock_group {LMS_FCLK2_PLL}
add_clock_group {LMS_FCLK2_DRCT}

# System / USB domain
add_clock_group {FX3_PCLK FX3_PCLK_VIRT FX3_PCLK_VIRT_OUT FPGA_SPI0_SCLK_reg FPGA_SPI0_SCLK_out FPGA_SPI1_SCLK BRDG_SPI_clk}

# DDR2 ALTMEMPHY memory controller domains
add_clock_group {*wfm_player_top* *wfm_player*}
add_clock_group {*ddr2_tester*}

if {$group_count > 1} {
	eval $cmd
}
											