# ----------------------------------------------------------------------------
# FILE:          CDC_constraints.sdc
# DESCRIPTION:   Gray-pointer constraints for the LiteX AsyncFIFOs (LimeSDR-USB).
#
# All inter-domain paths are cut by set_clock_groups -asynchronous
# (Clock_groups.sdc), so without these the routing of the gray-coded FIFO
# pointers into their first MultiReg stage is unconstrained and placement
# dependent. A gray-pointer CDC is only safe while the bit-to-bit skew of the
# pointer bus stays below the source clock period (10 ns for sys/FX3_PCLK,
# 8.1 ns for lms_tx/lms_rx at 122.88 MHz). set_net_delay is applied across the
# clock groups (set_max_skew is not: Quartus Standard drops cut paths from it,
# warning 332182), so a max net delay well below the period bounds the skew on
# every seed. Measured without it: <= 2.4 ns max, <= 1.7 ns spread per bus.
#
# The paths from a gray counter go only to its own synchroniser
# (impl_multiregimplN), so each assignment covers exactly that pointer bus.
# ----------------------------------------------------------------------------

set cdc_gray_counters {
   fx3_limestreamasyncfifo_graycounter0_q
   fx3_limestreamasyncfifo_graycounter1_q
   limetop_tx_path_input_buff_cdc_graycounter0_q
   limetop_tx_path_input_buff_cdc_graycounter1_q
   limetop_tx_path_smpl_nr_fifo_cdc_graycounter0_q
   limetop_tx_path_smpl_nr_fifo_cdc_graycounter1_q
   limetop_rx_path_source_ep_cdc_graycounter0_q
   limetop_rx_path_source_ep_cdc_graycounter1_q
   limetop_lms7002_top_tx_cdc_graycounter0_q
   limetop_lms7002_top_tx_cdc_graycounter1_q
}

foreach gc $cdc_gray_counters {
   set src [get_keepers -nowarn "${gc}*"]
   if {[get_collection_size $src] == 0} { continue }
   set dst [get_keepers -nowarn {*multiregimpl*}]
   set_net_delay -from $src -to $dst -max 4.0
}
