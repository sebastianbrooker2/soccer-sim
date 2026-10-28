#!/usr/bin/env bash
# Record N headless self-play matches -> recordings/<id>/data, converted to eval/gt/real_<id>.csv
# usage: N=10 SPEED=3 DURATION=200 ./record_matches.sh
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$HERE/../.."
N="${N:-10}"; SPEED="${SPEED:-3}"; DURATION="${DURATION:-200}"; PFX="${PFX:-real}"
for i in $(seq 1 "$N"); do
  id=$(printf "%s%02d" "$PFX" "$i")
  MATCH_ID=$id SOCCER_MATCH_DURATION=$DURATION SOCCER_MATCH_SPEED=$SPEED \
    SOCCER_MATCH_RESULT="$ROOT/recordings/$id.result.json" "$HERE/record_match.sh" || echo "match $id failed"
  [ -f "$ROOT/recordings/$id/data/manifest.json" ] && \
    python3 "$ROOT/tools/replay_to_csv.py" "$ROOT/recordings/$id/data" "$ROOT/eval/gt/${id}.csv" --dt 0.05
done
