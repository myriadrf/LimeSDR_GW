################################################################################
#Time settings
################################################################################
set_time_format -unit ns -decimal_places 3

################################################################################
#Timing parameters
################################################################################

################################################################################
#Base clocks
################################################################################
#Si5351C clocks
create_clock -period "27MHz" 			-name SI_CLK0			[get_ports SI_CLK0]
create_clock -period "27MHz" 			-name SI_CLK1			[get_ports SI_CLK1]
create_clock -period "27MHz" 			-name SI_CLK2			[get_ports SI_CLK2]
create_clock -period "27MHz" 			-name SI_CLK3			[get_ports SI_CLK3]
create_clock -period "27MHz" 			-name SI_CLK5			[get_ports SI_CLK5]
create_clock -period "27MHz" 			-name SI_CLK6			[get_ports SI_CLK6]
create_clock -period "27MHz" 			-name SI_CLK7			[get_ports SI_CLK7]
#LMK clock buffer clock
create_clock -period "30.72MHz"		-name LMK_CLK			[get_ports LMK_CLK]
#FX3 spi clock (if port exists)
set brdg_spi_port [get_ports -nowarn BRDG_SPI_clk]
if {[get_collection_size $brdg_spi_port] > 0} {
	create_clock -period "1MHz" 			-name BRDG_SPI_clk	$brdg_spi_port
}


################################################################################
#Virtual clocks
################################################################################
  

################################################################################
#Generated clocks
################################################################################

#NIOS / SPI
set spi0_sclk_reg [get_registers -nowarn {*|lms_ctr_spi_lms:spi_lms|SCLK_reg}]
if {[get_collection_size $spi0_sclk_reg] > 0} {
	create_generated_clock 	-name FPGA_SPI0_SCLK_reg \
									-source [get_ports {FX3_PCLK}] \
									-divide_by 6 \
									$spi0_sclk_reg
									
	create_generated_clock 	-name FPGA_SPI0_SCLK_out \
									-source $spi0_sclk_reg \
									[get_ports FPGA_SPI0_clk]
									
	set_false_path				-to [get_ports FPGA_SPI0_clk]
}

set spi1_sclk_reg [get_registers -nowarn {*|lms_ctr_spi_1_ADF:spi_1_adf|SCLK_reg}]
if {[get_collection_size $spi1_sclk_reg] > 0} {
	create_generated_clock -name FPGA_SPI1_SCLK \
									-source [get_ports FX3_PCLK] \
									-divide_by 6 \
									$spi1_sclk_reg
}


################################################################################
#Clock outputs
################################################################################


################################################################################
#Other clock constraints
################################################################################								
derive_clock_uncertainty


################################################################################
#Input constraints
################################################################################


#NIOS SPI0
set spi0_out_clk [get_clocks -nowarn FPGA_SPI0_SCLK_out]
if {[get_collection_size $spi0_out_clk] > 0} {
	if {$::quartus(nameofexecutable) ne "quartus_sta"} {
		set_input_delay -clock $spi0_out_clk -max 20.9 [get_ports {FPGA_SPI0_miso}] -clock_fall
		set_input_delay -clock $spi0_out_clk -min 16.2 [get_ports {FPGA_SPI0_miso}] -clock_fall
	} else {
		set_input_delay -clock $spi0_out_clk -max 19.0 [get_ports {FPGA_SPI0_miso}] -clock_fall
		set_input_delay -clock $spi0_out_clk -min 16.2 [get_ports {FPGA_SPI0_miso}] -clock_fall
	}

	set_output_delay -clock $spi0_out_clk -max 15 [get_ports {FPGA_SPI0_mosi}] 
	set_output_delay -clock $spi0_out_clk -min -15 [get_ports {FPGA_SPI0_mosi}]

	set_multicycle_path -setup -end -from $spi0_out_clk -to [get_clocks {FX3_PCLK}] [expr 3]
	set_multicycle_path -hold -end -from $spi0_out_clk -to [get_clocks {FX3_PCLK}] [expr 5]

	set_multicycle_path -setup -start -from [get_clocks FX3_PCLK] -to $spi0_out_clk 3
	set_multicycle_path -hold -start -from [get_clocks FX3_PCLK] -to $spi0_out_clk 5
}


################################################################################
#Output constraints
################################################################################						


################################################################################
#NIOS constraints
################################################################################
# JTAG Signal Constraints
set tck_port [get_ports -nowarn altera_reserved_tck]
if {[get_collection_size $tck_port] > 0} {
	create_clock -period 10MHz $tck_port
	set_clock_groups -asynchronous -group {altera_reserved_tck}
	set_input_delay -clock altera_reserved_tck -clock_fall .1 [get_ports -nowarn altera_reserved_tdi]
	set_input_delay -clock altera_reserved_tck -clock_fall .1 [get_ports -nowarn altera_reserved_tms]
	set_output_delay -clock altera_reserved_tck -clock_fall .1 [get_ports -nowarn altera_reserved_tdo]
}


################################################################################
#Timing exceptions
################################################################################
	
proc add_fp_from_ports {pattern} {
	set p [get_ports -nowarn $pattern]
	if {[get_collection_size $p] > 0} {
		set_false_path -from $p
	}
}

proc add_fp_to_ports {pattern} {
	set p [get_ports -nowarn $pattern]
	if {[get_collection_size $p] > 0} {
		set_false_path -to $p
	}
}

proc add_fp_from_regs {pattern} {
	set r [get_registers -nowarn $pattern]
	if {[get_collection_size $r] > 0} {
		set_false_path -from $r
	}
}

proc add_fp_to_regs {pattern} {
	set r [get_registers -nowarn $pattern]
	if {[get_collection_size $r] > 0} {
		set_false_path -to $r
	}
}

#set false paths between low speed signals
add_fp_to_ports {FPGA_LED*}
add_fp_to_ports {FX3_LED*}
add_fp_to_ports {FPGA_GPIO*}
add_fp_to_ports {LB_TX2*}
add_fp_to_ports {LB_TX1*}
add_fp_to_ports {LMS_CORE_LDO_EN}
add_fp_to_ports {LMS_RXEN}
add_fp_to_ports {LMS_TXEN}
add_fp_to_ports {LMS_TXNRX1}
add_fp_to_ports {LMS_TXNRX2}
add_fp_to_ports {FPGA_I2C_scl}
add_fp_to_ports {FPGA_I2C_sda}

add_fp_from_ports {EXT_GND*}
add_fp_from_ports {revision_HW_VER*}
add_fp_from_ports {revision_BOM_VER*}
add_fp_from_ports {ADF_MUXOUT*}
add_fp_from_ports {BRDG_SPI*}
add_fp_from_ports {FPGA_I2C_scl}
add_fp_from_ports {FPGA_I2C_sda}
add_fp_from_ports {FPGA_GPIO*}
add_fp_from_ports {LM75_OS*}

add_fp_to_ports {LMS_RESET*}
add_fp_to_ports {BRDG_SPI*}
add_fp_to_ports {FPGA_SPI1*}
add_fp_to_ports {FAN_CTRL*}
add_fp_to_ports {FPGA_SPI0_cs_n*}

set smpl_cnt3 [get_registers -nowarn {*rx_path_top*|smpl_cnt:smpl_cnt_inst3|lpm_cnt_inst:lpm_cnt_inst_inst0|lpm_counter:LPM_COUNTER_component|cntr_f5l:auto_generated|counter_reg_bit[*]}]
if {[get_collection_size $smpl_cnt3] > 0} {
	set_multicycle_path -from $smpl_cnt3 -to $smpl_cnt3 -setup -end 2
	set_multicycle_path -from $smpl_cnt3 -to $smpl_cnt3 -hold -end 1
}

set smpl_cnt4 [get_registers -nowarn {*rx_path_top*|smpl_cnt:smpl_cnt_inst4|lpm_cnt_inst:lpm_cnt_inst_inst0|lpm_counter:LPM_COUNTER_component|cntr_f5l:auto_generated|counter_reg_bit[*]}]
if {[get_collection_size $smpl_cnt4] > 0} {
	set_multicycle_path -from $smpl_cnt4 -to $smpl_cnt4 -setup -end 2
	set_multicycle_path -from $smpl_cnt4 -to $smpl_cnt4 -hold -end 1
}

#set false paths to output clocks 
add_fp_to_ports {LMS_FCLK1}
add_fp_to_ports {LMS_FCLK2}
add_fp_to_ports {FPGA_SPI1_clk}

add_fp_to_regs {*tstcfg*|dout_reg[*]}
add_fp_from_regs {*tstcfg*|mem[3][5]}
add_fp_from_regs {*fpgacfg*|mem[13][2]}

# DDR2 cross-domain signals (init_done, test flags, status registers)
add_fp_to_regs {*DDR2_ctrl_init_done_wcmd0*}
add_fp_to_regs {*DDR2_ctrl_init_done_rcmd0*}
add_fp_to_regs {*wfm_load_reg_pll_refclk*}

add_fp_from_regs {*ddr2_tester*|ddr2_traffic_gen:traffic_gen_inst|ddr2_traffic_gen_mm_traffic_generator_0:mm_traffic_generator_0|driver_avl_use_be_avl_use_burstbegin:traffic_generator_0|pnf_per_bit_persist[*]}
add_fp_from_regs {*wfm_player_top*|DDR2_ctrl_top:DDR2_ctrl_top_inst|ddr2_traffic_gen:traffic_gen_inst|ddr2_traffic_gen_mm_traffic_generator_0:mm_traffic_generator_0|driver_avl_use_be_avl_use_burstbegin:traffic_generator_0|pnf_per_bit_persist[*]}
add_fp_from_regs {*ddr2_tester*|ddr2_traffic_gen:traffic_gen_inst|ddr2_traffic_gen_mm_traffic_generator_0:mm_traffic_generator_0|driver_avl_use_be_avl_use_burstbegin:traffic_generator_0|driver_fsm_avl_use_be_avl_use_burstbegin:real_driver.driver_fsm_inst|stage.TIMEOUT}
add_fp_from_regs {*ddr2_tester*|ddr2_traffic_gen:traffic_gen_inst|ddr2_traffic_gen_mm_traffic_generator_0:mm_traffic_generator_0|driver_avl_use_be_avl_use_burstbegin:traffic_generator_0|driver_fsm_avl_use_be_avl_use_burstbegin:real_driver.driver_fsm_inst|stage.TEST_COMPLETE}
add_fp_from_regs {*sync_reg0\[*\]}




