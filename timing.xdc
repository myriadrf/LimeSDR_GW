# Renaming generated clocks
create_generated_clock -name sys -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT0]

create_generated_clock -name idelaye -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT1]

create_generated_clock -name afe -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT2]

set_clock_groups -name sys_async1 -asynchronous -group [get_clocks sys]

set_clock_groups -name sys_async2 -asynchronous -group [get_clocks afe]

set_clock_groups -name 1pps -asynchronous -group [get_clocks fpga_1pps_clk]

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets pps_IBUF_inst/O]

# FPGA_1PPS 245.76Mhz
create_clock -period 4.06901 -name fpga_1pps_clk [get_ports FPGA_1PPS_p]

# Rename auto-derived pll_afe output clocks
create_generated_clock -name afe_sys [get_pins -hierarchical "*PLLE2_ADV_1/CLKOUT0"]

create_generated_clock -name afe_sys_2x [get_pins -hierarchical "*PLLE2_ADV_1/CLKOUT1"]

# Add AFE sys clocks to same clock group
set_clock_groups -name afe_sys_async_group -asynchronous -group [get_clocks {fpga_1pps_clk afe_sys afe_sys_2x}]

