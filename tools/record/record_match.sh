#!/usr/bin/env bash
# SOCCER_MATCH_CMD for tools/tune.py: run ONE headless self-play match and write a result JSON.
#
# team1 (home) = candidate knobs from $SOCCER_TUNING_FILE ; team2 (away) = frozen baseline
# ($SOCCER_FROZEN_TUNING, default none). Boots the MuJoCo sim + motion, runs both agents inside the
# one container (loopback DDS), auto-starts the match, then reads the score from /soccer/game_controller.
#
# Proven working pieces (see ARCHITECTURE.md): headless sim, game_control score, boosteros control,
# our SoccerTeamRuntime connecting to all robots, GAME_CONTROL_AUTO_START.
set -euo pipefail

IMG="${SIM_IMAGE:-booster-robotics-registry.cn-beijing.cr.aliyuncs.com/virtual-robot/virtual-robot:0.6.5-beta}"
SIM_SRC="/usr/share/booster-studio/resources/app/booster-native/statics/virtual-robot/robocup_sim_src"
STAGE="${STAGE_DIR:-$HOME/.soccer-train}"
HERE="$(cd "$(dirname "$0")" && pwd)"
AGENT_REPO="${AGENT_REPO:-/home/seb/test3v3}"
MATCH_ID="${MATCH_ID:-match01}"
REC_ROOT="${REC_ROOT:-/home/seb/soccer-sim/recordings}"
REC_DIR="$REC_ROOT/$MATCH_ID"; mkdir -p "$REC_DIR"

DURATION="${SOCCER_MATCH_DURATION:-120}"     # wall-clock seconds to let the match run before scoring
SPEED="${SOCCER_MATCH_SPEED:-1}"             # speed_factor (>1 tries faster than real time)
RESULT="${SOCCER_MATCH_RESULT:-result.json}"
TUNING_FILE="${SOCCER_TUNING_FILE:-}"        # team1 candidate knobs
FROZEN_FILE="${SOCCER_FROZEN_TUNING:-}"      # team2 frozen baseline knobs
NAME="rec-$MATCH_ID-$$"

# Shared, read-only staging (sim source + robot_info) under $HOME (snap-docker only mounts from there).
# flock serializes staging across parallel matches so they don't race on the copy/patch.
mkdir -p "$STAGE/runs"
(
  flock 9
  [ -f "$STAGE/sim/app_in_container.py" ] || cp -a "$SIM_SRC" "$STAGE/sim"
  # Make speed_factor settable at init via SIM_SPEED_FACTOR (idempotent, atomic patch).
  python3 - "$STAGE/sim/physics/process.py" <<'PY'
import os, sys
p=sys.argv[1]; s=open(p).read()
if "SIM_SPEED_FACTOR" not in s:
    s=s.replace("    timer = SimulationTimer()",
      "    timer = SimulationTimer()\n"
      "    import os as _os\n"
      "    _sf=float(_os.environ.get('SIM_SPEED_FACTOR','1') or '1')\n"
      "    if _sf!=1.0: timer.set_speed_factor(_sf)",1)
    tmp=p+".tmp"; open(tmp,"w").write(s); os.replace(tmp,p)
PY
  [ -d "$STAGE/robot_info.txt" ] && rm -rf "$STAGE/robot_info.txt"
  [ -f "$STAGE/robot_info.txt" ] || cp "$HERE/robot_info.txt" "$STAGE/robot_info.txt"
) 9>"$STAGE/.stage.lock"

# Per-match work dir (parallel-safe): unique tuning + result temp files.
WORK="$HERE/.work/$NAME"; mkdir -p "$WORK"
if [ -n "$TUNING_FILE" ]; then cp "$TUNING_FILE" "$WORK/t1.json"; else echo '{}' > "$WORK/t1.json"; fi
if [ -n "$FROZEN_FILE" ]; then cp "$FROZEN_FILE" "$WORK/t2.json"; else echo '{}' > "$WORK/t2.json"; fi

cleanup() { docker exec "$NAME" bash -c "cp /root/*.log /replay/ 2>/dev/null; pkill -INT -f app_in_container.py; sleep 8; chmod -R a+rwX /replay" >/dev/null 2>&1 || true; docker rm -f "$NAME" >/dev/null 2>&1 || true; rm -rf "$WORK" 2>/dev/null || true; }
trap cleanup EXIT
docker rm -f "$NAME" >/dev/null 2>&1 || true

# NOTE: no GAME_CONTROL_AUTO_START. We connect both agents first (~30s each), THEN POST /match/start,
# so the READY->SET->kickoff sequence happens with the agents already connected and positioned.
# Auto-starting before the agents exist leaves the kickoff untaken and the match stuck at center.
docker run -d --name "$NAME" --privileged --shm-size=512m --ipc=private \
  -e ROS_DOMAIN_ID=0 -e HOME=/root -e SIM_SPEED_FACTOR="$SPEED" \
  -e MATCH_REPLAY_ENABLED=1 -e MATCH_REPLAY_HZ="${MATCH_REPLAY_HZ:-20}" -e MATCH_ID="$MATCH_ID" -e MATCH_REPLAY_OUTPUT_DIR=/replay/data \
  -v "$REC_DIR":/replay \
  -v "$STAGE/sim":/sim-ro:ro -v "$STAGE/robot_info.txt":/robot_info.txt:ro \
  -v "$AGENT_REPO":/agent:ro \
  -v "${EXT_XML:-$HERE/football_match_pitch_6_K1.extensions.xml}":/ext.xml:ro -v "$HERE/log_detections.py":/log_detections.py:ro \
  -v "$WORK/t1.json":/t1.json:ro -v "$WORK/t2.json":/t2.json:ro \
  --entrypoint bash "$IMG" -c 'cp /robot_info.txt /opt/booster/robot_info.txt
    cp -a /sim-ro /sim   # container-local sim so parallel matches do not share writable state
    cp /ext.xml /sim/mjcf/football_match_pitch_6_K1.extensions.xml   # container-local override (originals untouched)
    source /opt/ros/humble/setup.bash; source /opt/booster/BoosterRos2Interface/install/setup.bash 2>/dev/null; source /opt/booster/BoosterRos2/install/setup.bash 2>/dev/null
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
    export LD_LIBRARY_PATH=/opt/booster/Gait/lib:/opt/booster/Gait/lib/motion:${LD_LIBRARY_PATH:-}
    printf "MODEL_PATH=mjcf/football_match_pitch_6_K1.xml\nSIM_TRANSPORT=shm\n" > /root/.env
    nohup /opt/booster/Gait/bin/booster-server >/root/bs.log 2>&1 &
    ( cd /sim && python3 app_in_container.py --model mjcf/football_match_pitch_6_K1.xml --sim-transport shm --no-camera-relay >/root/sim.log 2>&1 ) &
    nohup ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /opt/booster/Gait/configs/K1/robot.urdf)" >/root/rsp.log 2>&1 &
    exec sleep infinity'

echo "[run_match] sim booting ($NAME)..." >&2
sleep 35
docker exec "$NAME" bash -lc 'pip install --quiet --root-user-action=ignore py_trees==2.4.0 >/dev/null 2>&1 || true'

# Launch both agents (they take ~30s each to connect all 3 robots; they connect in parallel).
docker exec -d "$NAME" bash -lc 'source /opt/ros/humble/setup.bash; source /opt/booster/BoosterRos2/install/setup.bash 2>/dev/null
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
  export LD_LIBRARY_PATH=/opt/booster/Gait/lib:/opt/booster/Gait/lib/motion:${LD_LIBRARY_PATH:-}
  export AGENT_DIR=/agent SOCCER_TEAM_ID=1 SOCCER_ROBOT_NAMES=robot1,robot2,robot3 SOCCER_TUNING_FILE=/t1.json
  python3 /agent/tools/training/agent_launch.py >/root/agent1.log 2>&1'
docker exec -d "$NAME" bash -lc 'source /opt/ros/humble/setup.bash; source /opt/booster/BoosterRos2/install/setup.bash 2>/dev/null
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
  export LD_LIBRARY_PATH=/opt/booster/Gait/lib:/opt/booster/Gait/lib/motion:${LD_LIBRARY_PATH:-}
  export AGENT_DIR=/agent SOCCER_TEAM_ID=2 SOCCER_ROBOT_NAMES=robot4,robot5,robot6 SOCCER_TUNING_FILE=/t2.json
  python3 /agent/tools/training/agent_launch.py >/root/agent2.log 2>&1'

# Wait for BOTH agents to connect all 3 of their robots before starting the match.
echo "[run_match] waiting for both agents to connect..." >&2
for _ in $(seq 1 40); do
  c1=$(docker exec "$NAME" bash -lc 'grep -c "Initialized robot" /root/agent1.log 2>/dev/null' || echo 0)
  c2=$(docker exec "$NAME" bash -lc 'grep -c "Initialized robot" /root/agent2.log 2>/dev/null' || echo 0)
  [ "${c1:-0}" -ge 3 ] && [ "${c2:-0}" -ge 3 ] && break
  sleep 3
done

# Start the match: this assigns the kickoff side and drives READY -> SET -> PLAYING (kickoff).
docker exec "$NAME" bash -lc 'python3 -c "import urllib.request; urllib.request.urlopen(urllib.request.Request(\"http://127.0.0.1:38383/match/start\", method=\"POST\"), timeout=10)"' || true
echo "[run_match] match started; sampling ${DURATION}s (speed x${SPEED})..." >&2

# Detection logger (runs for the whole sampling window)
if [ -n "${LOG_DET:-}" ]; then
docker exec -d "$NAME" bash -lc 'source /opt/ros/humble/setup.bash; source /opt/booster/BoosterRos2/install/setup.bash 2>/dev/null
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
  python3 /log_detections.py '"$((DURATION+5))"' /replay/det >/root/detlog.log 2>&1'
fi
# Sample the ball for the whole match window (this IS the match wait + dense territory signal).
docker exec "$NAME" bash -lc 'source /opt/ros/humble/setup.bash; source /opt/booster/BoosterRos2/install/setup.bash 2>/dev/null
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
  python3 /agent/tools/training/ball_sampler.py '"$DURATION"' /team1/soccer/sim/ground_truth/ball' > "$WORK/territory.json" 2>/dev/null || echo '{}' > "$WORK/territory.json"

# Read final score from the referee packet. --full-length is REQUIRED: ros2 echo truncates the long
# JSON string otherwise, so teams/score are never seen and every match falsely reads 0-0.
docker exec "$NAME" bash -lc 'source /opt/ros/humble/setup.bash; source /opt/booster/BoosterRos2/install/setup.bash 2>/dev/null
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
  timeout 10 ros2 topic echo --once --full-length /soccer/game_controller 2>/dev/null' > "$WORK/gc.txt" || true

python3 - "$WORK/gc.txt" "$WORK/territory.json" "$RESULT" <<'PY'
import json, re, sys
raw = open(sys.argv[1]).read()
m = re.search(r"\{.*\}", raw, re.S)
home = away = 0
if m:
    try:
        gc = json.loads(m.group(0))
        by_num = {int(t.get("teamNumber", i + 1)): t for i, t in enumerate(gc.get("teams", []))}
        home = int(by_num.get(1, {}).get("score", 0))
        away = int(by_num.get(2, {}).get("score", 0))
    except Exception:
        pass
terr = {}
try:
    terr = json.loads(open(sys.argv[2]).read() or "{}")
except Exception:
    pass
out = {
    "team1_score": home, "team2_score": away, "goal_diff_for": home - away, "goals_against": away,
    "ball_mean_x": float(terr.get("ball_mean_x", 0.0)),
    "frac_opp_half": float(terr.get("frac_opp_half", 0.5)),
    "samples": int(terr.get("samples", 0)),
}
json.dump(out, open(sys.argv[3], "w")); print(json.dumps(out))
PY
