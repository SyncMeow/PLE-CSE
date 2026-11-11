# Out-of-context synthesis + place + route of PLE + CSE (ple_cse_top) or of the CSE alone (cse_system).
# Same flow as ooc_impl.tcl (non-project, no bitstream, never touches Platform/build/hw/SoC).
# If place/route fails (e.g. the environment memory does not fit), the synthesis reports are still written.
#   vivado -mode batch -source scripts/ooc_impl_plecse.tcl -tclargs <out_dir> <top> <period_ns> <n_cu> [synth]
set out    [lindex $argv 0]
set top    [lindex $argv 1]
set period [lindex $argv 2]
set ncu    [lindex $argv 3]
set only_synth [expr {[llength $argv] > 4 && [lindex $argv 4] eq "synth"}]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize "$here/../rtl"]
file mkdir $out
set pk  [list "$rtl/common/ple_cse_pkg.sv" "$rtl/ple/ple_pkg.sv" "$rtl/ple/ple_robot_pkg.sv" "$rtl/cse/cse_cu_pkg.sv"]
set ple [lsearch -all -inline -not -glob [lsort [glob "$rtl/ple/*.sv"]] *_pkg.sv]
set cse [lsearch -all -inline -not -glob [lsort [glob "$rtl/cse/*.sv"]] *_pkg.sv]
read_verilog -sv [concat $pk $ple $cse "$rtl/ple_cse_top.sv"]
set tag "${top}_ncu${ncu}"
synth_design -top $top -part xc7a100tcsg324-1 -mode out_of_context -flatten_hierarchy [expr {[info exists ::env(FLATTEN)] ? $::env(FLATTEN) : "rebuilt"}] -generic N_CU=$ncu
create_clock -name clk -period $period [get_ports clk]
report_utilization -file "$out/${tag}_synth_util.rpt"
report_utilization -hierarchical -hierarchical_depth 3 -file "$out/${tag}_synth_util_hier.rpt"
set wns_s [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
proc counts {} {
  set lut  [llength [get_cells -hier -filter {PRIMITIVE_GROUP == LUT}]]
  set ff   [llength [get_cells -hier -filter {PRIMITIVE_GROUP == FLOP_LATCH}]]
  set b36  [llength [get_cells -hier -filter {REF_NAME =~ RAMB36*}]]
  set b18  [llength [get_cells -hier -filter {REF_NAME =~ RAMB18*}]]
  set lram [llength [get_cells -hier -filter {PRIMITIVE_GROUP == DMEM}]]
  set dsp  [llength [get_cells -hier -filter {REF_NAME =~ DSP*}]]
  return "lut_cells=$lut,ff=$ff,ramb36=$b36,ramb18=$b18,lutram_cells=$lram,dsp=$dsp"
}
puts "OOCSYNTH,$tag,period_ns=$period,wns_synth_ns=$wns_s,[counts]"
if {$only_synth} { exit }
if {[catch {
  opt_design
  place_design
  phys_opt_design
  route_design
} err]} {
  puts "OOCIMPL_FAIL,$tag,$err"
  exit 1
}
report_utilization -file "$out/${tag}_route_util.rpt"
report_utilization -hierarchical -hierarchical_depth 3 -file "$out/${tag}_route_util_hier.rpt"
report_timing_summary -max_paths 10 -file "$out/${tag}_route_timing.rpt"
report_route_status -file "$out/${tag}_route_status.rpt"
write_checkpoint -force "$out/${tag}_routed.dcp"
set wns  [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs  [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "OOCIMPL,$tag,period_ns=$period,wns_synth_ns=$wns_s,wns_route_ns=$wns,whs_route_ns=$whs,[counts]"
set p [get_timing_paths -max_paths 1 -nworst 1 -setup]
puts "WORST,[get_property STARTPOINT_PIN $p],[get_property ENDPOINT_PIN $p],levels=[get_property LOGIC_LEVELS $p],datapath_ns=[get_property DATAPATH_DELAY $p]"
