#!/bin/bash
# Earth Rover  --  VCS Triplet Status

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"
PID_DIR="${PID_DIR:-${LOG_DIR}/pids}"
VCS_PORT="${VCS_PORT:-8000}"

show() {
    local name="$1"
    local pidfile="$PID_DIR/$name.pid"
    if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
        printf "  %-12s RUNNING (PID %s)\n" "$name" "$(cat "$pidfile")"
    else
        printf "  %-12s STOPPED\n" "$name"
    fi
}

echo "VCS triplet status:"
show rosbridge
show web_video
show vcs_django

echo
echo "Ports of interest:"
ss -tln 2>/dev/null | awk -v p="$VCS_PORT" 'NR==1 || /:8080|:9090/ || $4 ~ ":"p"$"' || true
