create_clock -period 100 -name GPS_PPS [get_ports gps_pps]

set_clock_groups -group [get_clocks -include_generated_clocks lms7002m_mclk1]  -group [get_clocks -include_generated_clocks lms7002m_mclk2] -group [get_clocks -include_generated_clocks jtag_clk] -group [get_clocks -include_generated_clocks icap_clk] -group [get_clocks -include_generated_clocks dna_clk] -group [get_clocks -include_generated_clocks *] -group [get_clocks -include_generated_clocks clk26] -asynchronous
