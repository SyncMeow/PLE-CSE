#!/bin/bash
# CPU baseline for the PLE: fx_solve on the RTL CPU model, one image per tb_core problem, all in parallel.
#   tools/run_fx_soc.sh <profile 0|1> [first] [last]
# Logs: build/sw_rrtc_fx_soc/p<k>_<k>_prof<profile>_blob/sim.log (FXSOC lines)
cd "$(dirname "$0")/.." || exit 1
prof=${1:-0}; first=${2:-0}; last=${3:-13}
for k in $(seq "$first" "$last"); do
  make --no-print-directory fx-soc FX_BLOB=1 FX_PROFILE="$prof" FX_PROBS=$k:$k > /dev/null || exit 1
done
make --no-print-directory sim-model > /dev/null || exit 1
for k in $(seq "$first" "$last"); do
  make --no-print-directory fx-sim FX_BLOB=1 FX_PROFILE="$prof" FX_PROBS=$k:$k > ../build/sw_rrtc_fx_soc/run_p${k}_prof$prof.out 2>&1 &
done
wait
grep -h '^.*FXSOC,' ../build/sw_rrtc_fx_soc/p*_prof${prof}_blob/sim.log | sed 's/^.*FXSOC,/FXSOC,/'
