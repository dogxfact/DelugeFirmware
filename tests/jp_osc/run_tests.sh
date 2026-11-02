#!/bin/sh
# Host tests for the JP-mode oscillators. Usage: tests/jp_osc/run_tests.sh [path/to/simde] [path/to/jp-osc-data/include]
# Without simde the scalar path is tested; with it the NEON path runs through simde. With the data package the
# traced constants are used.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src="$here/../../src/deluge"
flags="-std=c++20 -O2 -Wall -I$src"
[ -n "$1" ] && flags="$flags -DJP_OSC_SIMDE -I$1"
[ -n "$2" ] && flags="$flags -I$2"
${CXX:-c++} $flags "$here/jp_osc_test.cpp" "$src/dsp/oscillators/jp_supersaw.cpp" "$src/dsp/oscillators/jp_feedback.cpp" -o "${TMPDIR:-/tmp}/jp_osc_test"
"${TMPDIR:-/tmp}/jp_osc_test"
