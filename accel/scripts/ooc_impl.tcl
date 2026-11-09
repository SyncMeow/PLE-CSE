# Out-of-context synthesis + place + route of one PLE top (post-route resources and timing).
# Non-project mode, no bitstream, nothing touches Platform/build/hw/SoC. Only reg-to-reg paths are timed:
# ports have no I/O delay (in the SoC they are registered by the bus / CSE side).
#   vivado -mode batch -source scripts/ooc_impl.tcl -tclargs <out_dir> <top> [period_ns]
set out    [lindex $argv 0]
set top    [lindex $argv 1]
set period [expr {[llength $argv] > 2 ? [lindex $argv 2] : 10.0}]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize "$here/../rtl"]
file mkdir $out
set pk  [list "$rtl/common/ple_cse_pkg.sv" "$rtl/ple/ple_pkg.sv" "$rtl/ple/ple_robot_pkg.sv"]
set src [lsearch -all -inline -not -glob [lsort [glob "$rtl/ple/*.sv"]] *_pkg.sv]
read_verilog -sv [concat $pk $src]
synth_design -top $top -part xc7a100tcsg324-1 -mode out_of_context -flatten_hierarchy rebuilt
create_clock -name clk -period $period [get_ports clk]
report_utilization -file "$out/${top}_synth_util.rpt"
set wns_s [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
opt_design
place_design
phys_opt_design
route_design
report_utilization -file "$out/${top}_route_util.rpt"
report_utilization -hierarchical -hierarchical_depth 2 -file "$out/${top}_route_util_hier.rpt"
report_timing_summary -max_paths 10 -file "$out/${top}_route_timing.rpt"
report_route_status -file "$out/${top}_route_status.rpt"
write_checkpoint -force "$out/${top}_routed.dcp"
set wns  [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs  [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
set lut  [llength [get_cells -hier -filter {PRIMITIVE_GROUP == LUT}]]
set ff   [llength [get_cells -hier -filter {PRIMITIVE_GROUP == FLOP_LATCH}]]
set bram [llength [get_cells -hier -filter {REF_NAME =~ RAMB*}]]
set dsp  [llength [get_cells -hier -filter {REF_NAME =~ DSP*}]]
puts "OOCIMPL,$top,period_ns=$period,wns_synth_ns=$wns_s,wns_route_ns=$wns,whs_route_ns=$whs,lut_cells=$lut,ff=$ff,ramb=$bram,dsp=$dsp"
set p [get_timing_paths -max_paths 1 -nworst 1 -setup]
puts "WORST,[get_property STARTPOINT_PIN $p],[get_property ENDPOINT_PIN $p],levels=[get_property LOGIC_LEVELS $p],datapath_ns=[get_property DATAPATH_DELAY $p]"
