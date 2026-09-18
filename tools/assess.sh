#!/bin/sh
# usage: assess.sh [ADC_MINUTES] [INTERVAL_MINUTES]  -- capture raw ADC ('d')
# and interval ('r') samples for the given minutes (defaults 6 and 45; 0
# skips a source), run the SP 800-90B non-IID assessment on each and print
# health test cutoffs. Everything is kept in captures/<date-time>/. Leaves
# the board in 'x' mode.
# Rates: ~3500 ADC samples/s (1M in ~5 min), ~50 intervals/s (1M in ~6 h).
# Long runs: use nohup and check summary.txt afterwards.
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
ADC_MINUTES=${1:-6}
INTERVAL_MINUTES=${2:-45}
DIR=$ROOT/captures/$(date +%Y%m%d-%H%M%S)
mkdir -p "$DIR"
export OMP_NUM_THREADS=2  # ea_non_iid otherwise uses every core; keep the laptop cool

run() {  # kind minutes name
  [ "$(python3 -c "print(int($2 > 0))")" = 1 ] || return 0
  python3 "$ROOT/tools/stump.py" capture $1 "$(python3 -c "print($2 * 60)")" "$DIR/raw_$1.bin"
  python3 "$ROOT/tools/stump.py" capture x 3 /dev/null
  python3 "$ROOT/sketch/rawdump.py" symbols "$DIR/raw_$1.bin" $1 "$DIR/$1.sym"
  "$ROOT/tools/ea_non_iid" -v "$DIR/$1.sym" 8 > "$DIR/ea_non_iid_$1.txt" 2>&1
  h=$(sed -n 's/^min(H_original, 8 X H_bitstring): //p' "$DIR/ea_non_iid_$1.txt")
  {
    echo "$3: $(wc -c < "$DIR/$1.sym") samples, assessed min-entropy $h bits/sample"
    python3 "$ROOT/sketch/rawdump.py" cutoffs "$h"
  } | tee -a "$DIR/summary.txt"
}

run d "$ADC_MINUTES" ADC
run r "$INTERVAL_MINUTES" intervals
echo "results in $DIR"
