# Renaming generated clocks
create_generated_clock -name sys -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT0]

create_generated_clock -name idelaye -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT1]

create_generated_clock -name afe -source [get_pins PLLE2_ADV/CLKIN1] -master_clock [get_clocks pcie_clk] [get_pins PLLE2_ADV/CLKOUT2]

set_clock_groups -name sys_async1 -asynchronous -group [get_clocks sys]

set_clock_groups -name sys_async2 -asynchronous -group [get_clocks afe]

set_clock_groups -name 1pps -asynchronous -group [get_clocks fpga_1pps_clk]

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets pps_IBUF_inst/O]

