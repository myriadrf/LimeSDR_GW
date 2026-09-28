################################################################################
#Time settings
################################################################################
set_time_format -unit ns -decimal_places 3

################################################################################
#Timing parameters
################################################################################

	#FX3
set FX3_period		10

set FX3_tDS 2 
set FX3_tDH 0.50

set FX3_tDS 2 
set FX3_tDH 0.50

	#FX3 tRDS tWRS tAS tPES combined to FX3_tSU
set FX3_tSU		2
	#FX3 tRDH tWRH tAH tPEH combined to FX3_tH
set FX3_tH 		.5

#set FX3_tCO_max 	7
#set FX3_tCO_min 	0

set FX3_tCO_max 	8
set FX3_tCO_min 	1

#set FX3_tCFLG_max 	8
#set FX3_tCFLG_min 	0
set FX3_tCFLG_max 	8
set FX3_tCFLG_min 	1

set FX3_d_in_max_dly [expr $FX3_tCO_max]
set FX3_d_in_min_dly [expr $FX3_tCO_min]

set FX3_ctl_in_max_dly [expr $FX3_tCFLG_max]
set FX3_ctl_in_min_dly [expr $FX3_tCFLG_min]

set FX3_d_out_max_dly [expr $FX3_tDS]
set FX3_d_out_min_dly [expr -$FX3_tDH]

set FX3_ctl_out_max_dly [expr $FX3_tSU]
set FX3_ctl_out_min_dly [expr -$FX3_tH]

################################################################################
#Base clocks
################################################################################

#FX3 spi clock (if port exists)
set brdg_spi_port [get_ports -nowarn BRDG_SPI_clk]
if {[get_collection_size $brdg_spi_port] > 0} {
	create_clock -period "1MHz" 			-name BRDG_SPI_clk	$brdg_spi_port
}
#FX3 GPIF clock
create_clock -period $FX3_period 	-name FX3_PCLK			[get_ports FX3_PCLK]

################################################################################
#Virtual clocks
################################################################################
create_clock -name FX3_PCLK_VIRT				-period $FX3_period	
create_clock -name FX3_PCLK_VIRT_OUT				-period $FX3_period

################################################################################
#Input constraints
################################################################################
#FX3
set_input_delay -clock [get_clocks FX3_PCLK_VIRT] \
-max $FX3_ctl_in_max_dly [get_ports {FX3_ctl4 FX3_ctl5 FX3_ctl8}]
set_input_delay -clock [get_clocks FX3_PCLK_VIRT] \
-min $FX3_ctl_in_min_dly [get_ports {FX3_ctl4 FX3_ctl5 FX3_ctl8}]

set_input_delay -clock [get_clocks FX3_PCLK_VIRT] \
-max $FX3_d_in_max_dly [get_ports {FX3_dq[*]}]
set_input_delay -clock [get_clocks FX3_PCLK_VIRT] \
-min $FX3_d_in_min_dly [get_ports {FX3_dq[*]}]

################################################################################
#Output constraints
################################################################################		
#FX3
set_output_delay -clock [get_clocks FX3_PCLK_VIRT_OUT] -max $FX3_ctl_out_max_dly \
								[get_ports {FX3_ctl0 FX3_ctl1 FX3_ctl2 \
												FX3_ctl3 FX3_ctl7 FX3_ctl11 FX3_ctl12}]
set_output_delay -clock [get_clocks FX3_PCLK_VIRT_OUT] -min $FX3_ctl_out_min_dly \
								[get_ports {FX3_ctl0 FX3_ctl1 FX3_ctl2 \
								FX3_ctl3 FX3_ctl7 FX3_ctl11 FX3_ctl12}]
                        
set_output_delay -clock [get_clocks FX3_PCLK_VIRT_OUT] -max $FX3_d_out_max_dly \
								[get_ports {FX3_dq[*]}]
set_output_delay -clock [get_clocks FX3_PCLK_VIRT_OUT] -min $FX3_d_out_min_dly \
								[get_ports {FX3_dq[*]}]
                        
#there is extra clock cycle for FX3_CTL2, should be fine                        
set_false_path -to [get_ports FX3_ctl2]