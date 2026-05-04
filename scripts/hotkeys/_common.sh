#!/bin/bash
# Shared helpers for earth-rover hotkey scripts.
# Sourced by trike-{start,stop}.sh, rosbag-{start,stop}.sh, status.sh.
# Not meant to be executed directly.
#
# Design:
#   - Each managed action runs as a transient systemd --user unit (er-hotkey-*),
#     so process tree management, signal handling, and logging are delegated
#     to systemd / journald.
#   - "start" is idempotent: re-pressing while the unit is active is a no-op
#     plus a notification.
#   - "stop trike" requires a double-tap within ER_DOUBLE_TAP_WINDOW seconds
#     to confirm; first tap arms, second tap fires. The arm state is a tiny
#     mtime file under $ER_HOTKEY_STATE so it survives bash invocations.
#   - notify-send + an audit log record every action.
#   - Hotkeys run from the GNOME shell with no user shell rc; we explicitly
#     source ROS env and locate the repo via $EARTH_ROVER_HOME (default
#     ~/earth-rover).

set -u

# Resolve the earth-rover repo even when no env is set (GNOME shortcut spawn).
if [ -z "${EARTH_ROVER_HOME:-}" ]; then
    # Walk up from this file: scripts/hotkeys -> scripts -> earth-rover.
    _here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    EARTH_ROVER_HOME="$(cd "$_here/../.." && pwd)"
fi
export EARTH_ROVER_HOME

ER_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ER_ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
export ROS_DISTRO="$ER_ROS_DISTRO" ROS2_WS="$ER_ROS2_WS"

ER_HOTKEY_STATE="${ER_HOTKEY_STATE:-/tmp/earth-rover-hotkeys-$(id -u)}"
ER_HOTKEY_LOG="${ER_HOTKEY_LOG:-$EARTH_ROVER_HOME/scripts/logs/hotkeys}"
ER_DOUBLE_TAP_WINDOW="${ER_DOUBLE_TAP_WINDOW:-3}"
mkdir -p "$ER_HOTKEY_STATE" "$ER_HOTKEY_LOG"

ER_AUDIT_LOG="$ER_HOTKEY_LOG/audit.log"

# --- Notification + audit ----------------------------------------------------

er_audit() {
    # er_audit <action> <result> [<detail>]
    local action="$1" result="$2" detail="${3:-}"
    printf '%s %-12s %-10s %s\n' \
        "$(date -Iseconds)" "$action" "$result" "$detail" \
        >> "$ER_AUDIT_LOG"
}

er_notify() {
    # er_notify <urgency> <title> [<body>]
    local urgency="$1" title="$2" body="${3:-}"
    if command -v notify-send >/dev/null 2>&1 && [ -n "${DISPLAY:-${WAYLAND_DISPLAY:-}}" ]; then
        notify-send -u "$urgency" -t 4500 -i system-run \
            -a "earth-rover" "$title" "$body" 2>/dev/null || true
    fi
    er_audit "$title" "$urgency" "$body"
}

# --- systemd unit lifecycle --------------------------------------------------

er_unit_active() {
    # er_unit_active <unit-name>; returns 0 if unit is active.
    systemctl --user is-active --quiet "$1"
}

er_unit_running_msg() {
    # er_unit_running_msg <unit-name>: print a one-liner with PID + start time
    local unit="$1"
    local pid started
    pid="$(systemctl --user show -p MainPID --value "$unit" 2>/dev/null || true)"
    started="$(systemctl --user show -p ActiveEnterTimestamp --value "$unit" 2>/dev/null || true)"
    printf 'unit=%s pid=%s since="%s"' "$unit" "${pid:-?}" "${started:-?}"
}

er_systemd_run_ros() {
    # er_systemd_run_ros <unit> <description> <KillSignal> <TimeoutStopSec> <make-target>
    #
    # Spawns a transient user unit that sources ROS env then runs `make <target>`
    # from $EARTH_ROVER_HOME. KillSignal lets us send SIGINT (ROS-friendly) on
    # `systemctl --user stop`.
    local unit="$1" desc="$2" sig="$3" timeout="$4" target="$5"

    if er_unit_active "$unit"; then
        er_notify normal "$unit already running" "$(er_unit_running_msg "$unit")"
        return 0
    fi

    # Clear any failed state from a prior run so unit name is reusable.
    systemctl --user reset-failed "$unit" 2>/dev/null || true

    systemd-run --user \
        --unit="$unit" \
        --description="$desc" \
        --collect \
        -p "KillSignal=$sig" \
        -p "TimeoutStopSec=$timeout" \
        -p "Restart=no" \
        -p "WorkingDirectory=$EARTH_ROVER_HOME" \
        -E "EARTH_ROVER_HOME=$EARTH_ROVER_HOME" \
        -E "ROS_DISTRO=$ER_ROS_DISTRO" \
        -E "ROS2_WS=$ER_ROS2_WS" \
        -E "HOME=$HOME" \
        -E "USER=$USER" \
        /bin/bash -lc "exec make -C '$EARTH_ROVER_HOME' $target" \
        >/tmp/_er_hotkey_run.$$ 2>&1
    local rc=$?
    local out; out="$(cat /tmp/_er_hotkey_run.$$ 2>/dev/null || true)"
    rm -f /tmp/_er_hotkey_run.$$

    if [ $rc -ne 0 ]; then
        er_notify critical "$unit start FAILED" "$out"
        return $rc
    fi

    sleep 0.5
    if er_unit_active "$unit"; then
        er_notify normal "$unit started" "$(er_unit_running_msg "$unit")"
    else
        er_notify critical "$unit failed to start" "$(systemctl --user status --no-pager -n 5 "$unit" 2>&1 | tail -5)"
        return 1
    fi
}

er_systemd_stop() {
    # er_systemd_stop <unit-name> <human-label>
    local unit="$1" label="$2"
    if ! er_unit_active "$unit"; then
        er_notify low "$label not running" "$unit is inactive"
        return 0
    fi
    systemctl --user stop "$unit" 2>>"$ER_AUDIT_LOG" || true
    # Wait briefly for inactivation
    for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
        er_unit_active "$unit" || break
        sleep 1
    done
    if er_unit_active "$unit"; then
        # Escalate
        systemctl --user kill --signal=SIGKILL --kill-who=all "$unit" 2>/dev/null || true
        sleep 1
    fi
    if er_unit_active "$unit"; then
        er_notify critical "$label stop FAILED" "$unit still active after SIGKILL"
        return 1
    fi
    er_notify normal "$label stopped" "$unit now inactive"
    systemctl --user reset-failed "$unit" 2>/dev/null || true
}

# --- Double-tap confirmation -------------------------------------------------

er_double_tap_check() {
    # er_double_tap_check <action-name>
    # Returns 0 if this is the second press inside the window (CONFIRMED).
    # Returns 1 if this is the first press; arms the timer + notifies.
    local action="$1"
    local ts_file="$ER_HOTKEY_STATE/$action.confirm"
    local now prev
    now="$(date +%s)"
    if [ -f "$ts_file" ]; then
        prev="$(cat "$ts_file" 2>/dev/null || echo 0)"
        if [ -n "$prev" ] && [ "$((now - prev))" -le "$ER_DOUBLE_TAP_WINDOW" ]; then
            rm -f "$ts_file"
            return 0
        fi
    fi
    echo "$now" > "$ts_file"
    er_notify normal "$action: confirm?" "Press the same key again within ${ER_DOUBLE_TAP_WINDOW}s to actually do it."
    return 1
}
