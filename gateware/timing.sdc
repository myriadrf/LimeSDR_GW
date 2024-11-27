# FT601 / 100MHz.
create_clock -name FT_CLK  -period 10.000 [get_ports FT_CLK]

# Sys Clk / 77.5MHz.
create_clock -name SYS_CLK -period 12.903 [get_pins {OSCG.OSC}]

# LMS7002M / 40MHz & 125MHz.
create_clock -name LMK_CLK   -period 25.000 [get_ports LMK_CLK]
create_clock -name LMS_MCLK1 -period 8.000  [get_ports LMS_MCLK1]
create_clock -name LMS_MCLK2 -period 8.000  [get_ports LMS_MCLK2]
