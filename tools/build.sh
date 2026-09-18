#!/bin/sh
# usage: [BOARD=pro] [EXTRA_FLAGS=-D...] build.sh [SKETCH_DIR]  -- compile for the Digispark
# (ATtiny85 at 16.5 MHz, default) or the Digispark Pro (ATtiny167, BOARD=pro).
# Output: build/<sketch name>[-pro]/<sketch name>.ino.hex
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SKETCH=$(cd "${1:-$ROOT/sketch}" && pwd -P)
NAME=$(basename "$SKETCH")
case "${BOARD:-tiny}" in
  tiny) FQBN=digistump:avr:digispark-tiny:clock=clock165; OUT=$ROOT/build/$NAME ;;
  pro)  FQBN=digistump:avr:digispark-pro;                 OUT=$ROOT/build/$NAME-pro ;;
  *)    echo "build.sh: BOARD must be tiny or pro" >&2; exit 1 ;;
esac
"$ROOT/tools/arduino-cli" --config-file "$HOME/.arduinoIDE/arduino-cli.yaml" compile \
  --fqbn "$FQBN" --warnings default --output-dir "$OUT" \
  --build-property "build.extra_flags=$EXTRA_FLAGS" "$SKETCH"
echo "$OUT/$NAME.ino.hex"
