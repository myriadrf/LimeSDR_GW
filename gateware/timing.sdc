create_clock -name FT_CLK  -period 10.000 [get_ports FT_CLK]
create_clock -name LMK_CLK -period 25.000 [get_ports LMK_CLK]
create_clock -name OSC_CLK -period 12.903 [get_ports OSC_CLK]