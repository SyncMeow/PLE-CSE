# Out-of-context synthesis of PLE modules (resource / RAM inference / post-synth timing estimate).
# Non-project mode, no bitstream, nothing touches Platform/build/hw/SoC.
#   vivado -mode batch -source scripts/ooc_synth.tcl -tclargs <out_dir> <top> [<top> ...]
set out  [lindex $argv 0]
set tops [lrange $argv 1 end]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize "$here/../rtl"]
file mkdir $out
foreach top $tops {
  set pk [list "$rtl/common/ple_cse_pkg.sv" "$rtl/ple/ple_pkg.sv" "$rtl/ple/ple_robot_pkg.sv"]
  set src [lsearch -all -inline -not -glob [lsort [glob "$rtl/ple/*.sv"]] *_pkg.sv]
  read_verilog -sv [concat $pk $src]                ;# every PLE module, so integrated tops (ple_evu, ple_top) build
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
  # worst slack per endpoint register (bits merged), from the 2000 worst endpoints
  set fam [dict create]
  foreach p [get_timing_paths -max_paths 2000 -nworst 1 -setup -slack_lesser_than 0] {
    set e [get_property NAME [get_property ENDPOINT_PIN $p]]
    regsub -all {_reg(\[[0-9]+\])*(/[A-Z]+)?$} [regsub -all {\[[0-9]+\]} $e {}] {} k   ;# endpoint register, bits merged
    set s [get_property SLACK $p]
    if {![dict exists $fam $k] || $s < [lindex [dict get $fam $k] 0]} { dict set fam $k [list $s [get_property LOGIC_LEVELS $p] [get_property NAME [get_property STARTPOINT_PIN $p]] $e] }
  }
  dict for {k v} $fam { puts "FAM,$top,$k,[join $v ,]" }
  close_design
}
