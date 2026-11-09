# OOC synthesis of ple_top for several tree capacities (resources only; post-synth estimate).
#   vivado -mode batch -source scripts/ooc_synth_cap.tcl -tclargs <out_dir> <cap> [<cap> ...]
set out  [lindex $argv 0]
set caps [lrange $argv 1 end]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize "$here/../rtl"]
file mkdir $out
foreach cap $caps {
  set pk  [list "$rtl/common/ple_cse_pkg.sv" "$rtl/ple/ple_pkg.sv" "$rtl/ple/ple_robot_pkg.sv"]
  set src [lsearch -all -inline -not -glob [lsort [glob "$rtl/ple/*.sv"]] *_pkg.sv]
  read_verilog -sv [concat $pk $src]
  synth_design -top ple_top -part xc7a100tcsg324-1 -mode out_of_context -flatten_hierarchy rebuilt -generic CAP=$cap
  create_clock -name clk -period 10.000 [get_ports clk]
  report_utilization -file "$out/cap${cap}_util.rpt"
  report_utilization -hierarchical -hierarchical_depth 3 -file "$out/cap${cap}_util_hier.rpt"
  set wns  [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
  set lut  [llength [get_cells -hier -filter {PRIMITIVE_GROUP == LUT}]]
  set ff   [llength [get_cells -hier -filter {PRIMITIVE_GROUP == FLOP_LATCH}]]
  set b36  [llength [get_cells -hier -filter {REF_NAME =~ RAMB36*}]]
  set b18  [llength [get_cells -hier -filter {REF_NAME =~ RAMB18*}]]
  set dsp  [llength [get_cells -hier -filter {REF_NAME =~ DSP*}]]
  puts "OOCCAP,cap=$cap,wns_synth_ns=$wns,lut_cells=$lut,ff=$ff,ramb36=$b36,ramb18=$b18,dsp=$dsp"
  close_design
}
