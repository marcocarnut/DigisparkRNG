#!/bin/sh
# usage: flash.sh [HEX]  -- reboot the running sketch into the bootloader
# (Ctrl-C x3, or 134 bps for bridges), flash HEX (default: the sketch's last build), wait for the port.
# If the running sketch can't jump to the bootloader, replug the board when
# micronucleus asks (it waits 60 s).
ROOT=$(cd "$(dirname "$0")/.." && pwd)
NAME=$(basename "$(cd "$ROOT/sketch" && pwd -P)")
HEX=${1:-$ROOT/build/$NAME/$NAME.ino.hex}
# Ctrl-C x3 for sketches that read commands; 134 bps for bridges, which pass
# every byte through
if [ -e /dev/ttyACM0 ]; then
  python3 "$ROOT/tools/stump.py" bootloader
  sleep 1
  # 134 bps three times: the bridges ask for that much before believing it,
  # since a corrupted line-coding request used to be enough to reboot them.
  # (The rate has to change in between or the host does not send it again.)
  for i in 1 2 3; do
    [ -e /dev/ttyACM0 ] || break
    stty -F /dev/ttyACM0 134 2>/dev/null
    stty -F /dev/ttyACM0 9600 2>/dev/null
  done
fi
LOG=$(mktemp)
"$HOME/.arduino15/packages/digistump/tools/micronucleus/2.6/micronucleus" \
  --no-ansi --run --timeout 60 "$HEX" > "$LOG" 2>&1
grep -v '% complete' "$LOG"
if ! grep -q 'Micronucleus done' "$LOG"; then
  rm -f "$LOG"
  echo "flash.sh: flashing FAILED" >&2
  exit 1
fi
rm -f "$LOG"
python3 "$ROOT/tools/stump.py" wait 60
