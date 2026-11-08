# Out-of-context synthesis of PLE modules (resource / RAM inference / post-synth timing estimate).
# Non-project mode, no bitstream, nothing touches Platform/build/hw/SoC.
#   vivado -mode batch -source scripts/ooc_synth.tcl -tclargs <out_dir> <top> [<top> ...]
set out  [lindex $argv 0]
set tops [lrange $argv 1 end]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize "$here/../rtl"]
file mkdir $out
foreach top $tops {
  read_verilog -sv [list "$rtl/common/ple_cse_pkg.sv" "$rtl/ple/ple_pkg.sv" "$rtl/ple/$top.sv"]
  synth_design -top $top -part xc7a100tcsg324-1 -mode out_of_context -flatten_hierarchy rebuilt
  create_clock -name clk -period 10.000 [get_ports clk]
  report_utilization -file "$out/${top}_util.rpt"
  report_timing_summary -max_paths 5 -file "$out/${top}_timing.rpt"
  set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
  set bram [llength [get_cells -hier -filter {REF_NAME =~ RAMB*}]]
  set lutram [llength [get_cells -hier -filter {REF_NAME =~ RAM32* || REF_NAME =~ RAM64* || REF_NAME =~ RAM128* || REF_NAME =~ RAM256*}]]
  set ff [llength [get_cells -hier -filter {REF_NAME =~ FD*}]]
  set dsp [llength [get_cells -hier -filter {REF_NAME =~ DSP*}]]
  puts "OOC,$top,wns_ns=$wns,ramb=$bram,lutram_cells=$lutram,ff=$ff,dsp=$dsp"
  close_design
}
