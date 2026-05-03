#!/usr/bin/env bash
# Run ORB-SLAM3 (monocular) on the right Grasshopper images recorded inside
# a rosbag2 directory.  Spins up:
#   1. scripts/bayer_to_mono.py                (bayer_gbrg8 -> mono8)
#   2. orbslam3 mono node                      (subscribes to mono topic)
#   3. ros2 bag play <bag> --clock             (publishes /stereo/right/image_raw)
#
# All processes are children of this script; Ctrl+C tears them down.  When
# the bag finishes the script signals SLAM to shut down so it can write
# CameraTrajectory.txt / KeyFrameTrajectory.txt into --output-dir.
#
# Usage:
#   scripts/run_orbslam3_right_rosbag.sh \
#       --bag /path/to/earth_rover_20260502_083404 \
#       [--output-dir reports/orbslam3_right] \
#       [--settings ~/ros2_ws/src/orbslam3_ros2/config/monocular/gh.yaml] \
#       [--rate 1.0] [--downscale 1] [--start 0]

set -euo pipefail

# colcon's install/setup.bash tests `[ -n "$COLCON_TRACE" ]`; with `set -u`
# (nounset) that fails if the variable is unset — common when .bashrc enables -u.
export COLCON_TRACE="${COLCON_TRACE:-}"

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BAG=""
OUT="reports/orbslam3_right"
SETTINGS="${HOME}/ros2_ws/src/orbslam3_ros2/config/monocular/gh.yaml"
VOCAB="${HOME}/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt"
BAYER_TOPIC="/stereo/right/image_raw"
MONO_TOPIC="/stereo/right/image_mono"
RATE="1.0"
DOWNSCALE="1"
START="0"
NAME=""

while [ $# -gt 0 ]; do
    case "$1" in
        --bag)         shift; BAG="$1" ;;
        --output-dir)  shift; OUT="$1" ;;
        --settings)    shift; SETTINGS="$1" ;;
        --vocab)       shift; VOCAB="$1" ;;
        --bayer-topic) shift; BAYER_TOPIC="$1" ;;
        --mono-topic)  shift; MONO_TOPIC="$1" ;;
        --rate)        shift; RATE="$1" ;;
        --downscale)   shift; DOWNSCALE="$1" ;;
        --start)       shift; START="$1" ;;
        --name)        shift; NAME="$1" ;;
        --help|-h)
            sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

if [ -z "$BAG" ]; then
    echo "missing --bag <path>" >&2
    exit 2
fi
if [ ! -d "$BAG" ]; then
    echo "bag dir not found: $BAG" >&2
    exit 2
fi
if [ ! -f "$SETTINGS" ]; then
    echo "ORB-SLAM3 settings not found: $SETTINGS" >&2
    exit 2
fi
if [ ! -f "$VOCAB" ]; then
    echo "ORBvoc.txt not found: $VOCAB" >&2
    exit 2
fi

if [ -z "$NAME" ]; then
    NAME="$(basename "$BAG")"
fi

if [[ "$OUT" != /* ]]; then
    OUT="$REPO_DIR/$OUT"
fi
RUN_DIR="$OUT/$NAME"
mkdir -p "$RUN_DIR"

# ---- environment --------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        # shellcheck disable=SC1091
        . /opt/ros/jazzy/setup.bash
    else
        echo "[run-orbslam3] ROS not on PATH; install ros-jazzy-* first" >&2
        exit 1
    fi
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    . "$HOME/ros2_ws/install/setup.bash"
fi

export ORB_SLAM3_ROOT_DIR="${ORB_SLAM3_ROOT_DIR:-$HOME/ORB-SLAM3-STEREO-FIXED}"
export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT_DIR/lib:$ORB_SLAM3_ROOT_DIR/Thirdparty/DBoW2/lib:$ORB_SLAM3_ROOT_DIR/Thirdparty/g2o/lib:$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

cd "$RUN_DIR"

cleanup() {
    local rc=${1:-0}
    echo "[run-orbslam3] cleaning up children ..."
    for pid in "${PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    sleep 1
    for pid in "${PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    exit "$rc"
}

trap 'cleanup 130' INT TERM

PIDS=()

echo "[run-orbslam3] bag      : $BAG"
echo "[run-orbslam3] settings : $SETTINGS"
echo "[run-orbslam3] vocab    : $VOCAB"
echo "[run-orbslam3] outputs  : $RUN_DIR"
echo "[run-orbslam3] bayer    : $BAYER_TOPIC -> $MONO_TOPIC (downscale=$DOWNSCALE)"
echo

# ---- 1. bayer relay -----------------------------------------------------
python3 "$REPO_DIR/scripts/bayer_to_mono.py" \
    --ros-args \
        -p "input_topic:=$BAYER_TOPIC" \
        -p "output_topic:=$MONO_TOPIC" \
        -p "downscale:=$DOWNSCALE" \
    >"$RUN_DIR/bayer_to_mono.log" 2>&1 &
PIDS+=("$!")
echo "[run-orbslam3] started bayer_to_mono.py (pid ${PIDS[-1]})"

# ---- 2. orbslam3 mono ---------------------------------------------------
ros2 run orbslam3 mono "$VOCAB" "$SETTINGS" \
    --ros-args \
        -p "image_topic:=$MONO_TOPIC" \
        -p "trajectory_prefix:=$NAME" \
    >"$RUN_DIR/orbslam3_mono.log" 2>&1 &
PIDS+=("$!")
echo "[run-orbslam3] started orbslam3 mono (pid ${PIDS[-1]})"

# Pause so the SLAM viewer is up before frames start streaming.
sleep 3

# ---- 3. play the bag ----------------------------------------------------
echo "[run-orbslam3] starting ros2 bag play (rate=$RATE start=${START}s) ..."
ros2 bag play "$BAG" \
    --rate "$RATE" \
    --start-offset "$START" \
    --topics "$BAYER_TOPIC" \
    >"$RUN_DIR/ros2_bag_play.log" 2>&1 &
PIDS+=("$!")
BAG_PID="${PIDS[-1]}"
echo "[run-orbslam3] started ros2 bag play (pid $BAG_PID)"

wait "$BAG_PID"
BAG_RC=$?
echo "[run-orbslam3] bag play exited rc=$BAG_RC; stopping SLAM ..."

# Trigger SLAM destructor (writes trajectories) by SIGINT-ing it.
kill -INT "${PIDS[1]}" 2>/dev/null || true
sleep 5
kill -INT "${PIDS[0]}" 2>/dev/null || true
sleep 1
for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null || true
    fi
done

ls -la "$RUN_DIR" || true
echo "[run-orbslam3] done -> $RUN_DIR"
