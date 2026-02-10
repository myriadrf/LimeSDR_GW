# FPGA_GT_AFEREF 245.76Mhz
create_clock -period 4.069 -name fpga_gt_aferef_clk [get_ports afe79xx_serdes_x4_fpga_gt_aferef_p]

# FPGA_1PPS 245.76Mhz
create_clock -period 4.069 -name fpga_1pps_clk [get_ports FPGA_1PPS_p]

# FPGA_SYSREF 3.84Mhz
create_clock -period 260.416 -name fpga_sysref_clk [get_ports FPGA_SYSREF_p]

set_clock_groups -name afe_async1 -asynchronous -group [get_clocks fpga_1pps_clk]

set_clock_groups -name afe_async2 -asynchronous -group [get_clocks xcvr_top_inst_n_0]

set_clock_groups -name afe_async3 -asynchronous -group [get_clocks xcvr_top_inst_n_1]

