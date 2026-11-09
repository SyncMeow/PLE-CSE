# Post-route phys_opt (AggressiveExplore, as in the SoC impl_1 flow) on a routed OOC checkpoint.
#   vivado -mode batch -source scripts/ooc_postroute_physopt.tcl -tclargs <routed.dcp> <out_prefix>
set dcp [lindex $argv 0]
set out [lindex $argv 1]
open_checkpoint $dcp
phys_opt_design -directive AggressiveExplore
report_timing_summary -max_paths 10 -file "${out}_timing.rpt"
report_utilization -file "${out}_util.rpt"
write_checkpoint -force "${out}.dcp"
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "PHYSOPT,wns_ns=$wns,whs_ns=$whs"
set p [get_timing_paths -max_paths 1 -nworst 1 -setup]
puts "WORST,[get_property STARTPOINT_PIN $p],[get_property ENDPOINT_PIN $p],levels=[get_property LOGIC_LEVELS $p]"
