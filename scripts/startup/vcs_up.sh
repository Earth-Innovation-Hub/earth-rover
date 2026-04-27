#!/bin/bash
# Earth Rover  --  VCS Triplet Launcher (Vehicle Control Station)
#
# Bring up the three services that always launch together for the web UI:
#   1. ROSBridge WebSocket  (ros2 launch rosbridge_server rosbridge_websocket_launch.xml)
#   2. Web Video Server     (ros2 run web_video_server web_video_server)
#   3. Django runserver     (python3 manage.py runserver 0.0.0.0:8000)
#
# Each runs as a detached background process with a PID file and log under
# scripts/logs/.  Use vcs_down.sh to stop, vcs_status.sh to check.
#
# Env:
#   ROS_DISTRO  (default: humble)
#   ROS2_WS     (default: /home/jdas/ros2_ws)
#   VCS_PORT    (default: 8000)
#   VCS_BIND    (default: 0.0.0.0)

set -e

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"
PID_DIR="${PID_DIR:-${LOG_DIR}/pids}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-/home/jdas/ros2_ws}"
VCS_DIR="${EARTH_ROVER_HOME}/vehicle_control_station"
VCS_PORT="${VCS_PORT:-8000}"
VCS_BIND="${VCS_BIND:-0.0.0.0}"

mkdir -p "$LOG_DIR" "$PID_DIR"

# Source ROS env so child processes inherit it
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "$ROS2_WS/install/setup.bash"
fi

is_running() {
    local pidfile="$1"
    [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null
}

start_service() {
    local name="$1"; shift
    local cmd_dir="$1"; shift
    local pidfile="$PID_DIR/$name.pid"
    local logfile="$LOG_DIR/$name.log"

    if is_running "$pidfile"; then
        echo "[vcs_up] $name already running (PID $(cat "$pidfile"))"
        return 0
    fi

    echo "[vcs_up] Starting $name ..."
    ( cd "$cmd_dir" && nohup "$@" >"$logfile" 2>&1 & echo $! >"$pidfile" )
    sleep 1
    if is_running "$pidfile"; then
        echo "  └─ PID $(cat "$pidfile")  log: $logfile"
    else
        echo "  └─ FAILED to start.  See $logfile"
        return 1
    fi
}

echo "================================================"
echo "Earth Rover VCS Triplet"
echo "================================================"
echo "Logs:  $LOG_DIR"
echo "PIDs:  $PID_DIR"
echo ""

start_service "rosbridge"   "$EARTH_ROVER_HOME" \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

start_service "web_video"   "$EARTH_ROVER_HOME" \
    ros2 run web_video_server web_video_server

start_service "vcs_django"  "$VCS_DIR" \
    python3 manage.py runserver "${VCS_BIND}:${VCS_PORT}"

echo ""
echo "VCS triplet up.  Tail logs with: tail -f $LOG_DIR/{rosbridge,web_video,vcs_django}.log"
echo "Open: http://${VCS_BIND}:${VCS_PORT}/"
