set pnf_ports [get_ports -nowarn "pnf"]
if {[get_collection_size $pnf_ports] > 0} {
	set_false_path -from * -to $pnf_ports
}

set tc_ports [get_ports -nowarn "test_complete"]
if {[get_collection_size $tc_ports] > 0} {
	set_false_path -from * -to $tc_ports
}

set pnf_byte_ports [get_ports -nowarn "pnf_per_byte\[*\]"]
if {[get_collection_size $pnf_byte_ports] > 0} {
	set_false_path -from * -to $pnf_byte_ports
}
