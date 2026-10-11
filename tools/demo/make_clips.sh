#!/usr/bin/env bash
# Flies the demo scenarios in the co-simulation and renders each as a ground-station
# video clip, for the demo video (docs/demo.md).
#
# Run from the repository root on Linux (WSL), with the firmware built
# (cmake --preset debug && cmake --build --preset debug) and a Python environment
# that has the project's dev extras (pip install -e ".[dev]"):
#
#     bash tools/demo/make_clips.sh [--reuse] [OUT_DIR]
#
# Writes OUT_DIR/<scenario>.tlm (the telemetry), OUT_DIR/<scenario>/ (the nodes' logs
# and ground truth) and OUT_DIR/<scenario>.mp4, or .gif without ffmpeg. The
# co-simulation runs at about 0.15x real time, so the five flights take about 20 minutes.
# --reuse renders from the telemetry of an earlier run instead of flying again.

set -euo pipefail

REUSE=false
if [[ "${1:-}" == "--reuse" ]]; then
  REUSE=true
  shift
fi
OUT="${1:-build/demo}"
mkdir -p "$OUT"
if command -v ffmpeg >/dev/null; then EXT=mp4; else EXT=gif; fi

# name | mission options | caption
CLIPS=(
  "close-pass|--scenario nominal --start 150 --duration 45|Close pass: Node A keeps the vehicle more than 10 m from the line (HLR-001, HLR-002)"
  "no-avoidance|--scenario no-avoidance --start 150 --duration 45|Autopilot ignores avoidance: corrupted magnetometer detected, GPS course used instead (HLR-008)"
  "link-loss|--scenario link-loss --duration 25|Ground link lost at 12 s: return to home 3 s later (HLR-003)"
  "battery|--scenario battery --duration 30|Battery draining from 22 %: return home below 20 %, land below 10 % (HLR-004)"
  "prop-damage|--scenario prop-damage --duration 25|Damaged propeller at 15 s: on-board INT8 classifier raises the alarm (HLR-009)"
)

for clip in "${CLIPS[@]}"; do
  IFS="|" read -r name options caption <<< "$clip"
  if [[ "$REUSE" == true && -s "$OUT/$name.tlm" ]]; then
    echo "=== $name: reusing $OUT/$name.tlm"
  else
    echo "=== $name: flying"
    # shellcheck disable=SC2086  # the options are meant to split into arguments
    python3 -m sim.mission $options --out "$OUT/$name" --record "$OUT/$name.tlm"
  fi
  echo "=== $name: rendering $OUT/$name.$EXT"
  python3 -m ground_station.display --replay "$OUT/$name.tlm" --video "$OUT/$name.$EXT" \
    --follow 250 --title "$caption"
done

echo "done: clips in $OUT"
