#!/bin/bash
# Earth Rover  --  Stop VCS Triplet
#
# Stops services started by vcs_up.sh, using PID files.  Falls back to
# pgrep/pkill against the canonical command lines if PID files are missing.

set -e

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"
PID_DIR="${PID_DIR:-${LOG_DIR}/pids}"

stop_pidfile() {
    local name="$1"
    local pidfile="$PID_DIR/$name.pid"
    if [ -f "$pidfile" ]; then
        local pid
        pid=$(cat "$pidfile")
        if kill -0 "$pid" 2>/dev/null; then
            echo "[vcs_down] Stopping $name (PID $pid) ..."
            # Also kill the process group, in case ros2 launch spawned children
            kill -SIGTERM -- "-$pid" 2>/dev/null || kill -SIGTERM "$pid" 2>/dev/null || true
            sleep 1
            if kill -0 "$pid" 2>/dev/null; then
                kill -SIGKILL -- "-$pid" 2>/dev/null || kill -SIGKILL "$pid" 2>/dev/null || true
            fi
        else
            echo "[vcs_down] $name not running (stale PID file)"
        fi
        rm -f "$pidfile"
    else
        echo "[vcs_down] $name (no PID file)"
    fi
}

stop_pattern() {
    local name="$1"; shift
    local pattern="$*"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        echo "[vcs_down] (fallback) killing $name ($pattern) -> $pids"
        echo "$pids" | xargs -r kill -SIGTERM 2>/dev/null || true
        sleep 1
        pids=$(pgrep -f "$pattern" 2>/dev/null || true)
        [ -n "$pids" ] && echo "$pids" | xargs -r kill -SIGKILL 2>/dev/null || true
    fi
}

echo "================================================"
echo "Stopping Earth Rover VCS Triplet"
echo "================================================"

stop_pidfile vcs_django
stop_pidfile web_video
stop_pidfile rosbridge

# Belt-and-suspenders: clean up anything still matching the launch lines
stop_pattern "Django runserver"   "manage.py runserver"
stop_pattern "web_video_server"   "web_video_server"
stop_pattern "rosbridge_server"   "rosbridge_websocket"

echo "Done."
