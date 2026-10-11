#!/usr/bin/env bash
# Turns demo clips (tools/demo/make_clips.sh) into the small, sped-up GIFs the
# README shows: docs/images/<clip>.gif.
#
#     bash tools/demo/readme_gifs.sh [CLIPS_DIR]
#
# Needs ffmpeg. Each GIF plays its flight at 3x, 5 frames per second, 900 pixels
# wide, with a 32-colour palette computed for that clip: 1 to 2 MB each.

set -euo pipefail

CLIPS="${1:-build/demo}"
SPEED=3
WIDTH=900
FPS=5

for name in no-avoidance close-pass link-loss battery prop-damage; do
  src="$CLIPS/$name.mp4"
  out="docs/images/$name.gif"
  [[ -s "$src" ]] || { echo "skip $name: no $src"; continue; }
  ffmpeg -v error -y -i "$src" -filter_complex \
    "setpts=PTS/$SPEED,fps=$FPS,scale=$WIDTH:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=32:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle" \
    "$out"
  echo "$out: $(( $(stat -c %s "$out") / 1024 )) KB"
done
