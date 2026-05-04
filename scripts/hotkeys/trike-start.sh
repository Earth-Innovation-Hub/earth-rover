#!/bin/bash
# Hotkey: start the trike system launch.
#
# Idempotent. Spawns `make system-launch` inside a transient
# `er-hotkey-trike.service` user unit so the launch survives this script's
# exit, gets unified journald logs, and can be cleanly stopped via SIGINT
# (which `make` propagates to the underlying `ros2 launch` process tree).
#
# Bound by default to Ctrl+Alt+Super+T (see install-gnome.sh).
#
# Override the make target (e.g. for a non-trike profile):
#   ER_TRIKE_TARGET=mavros ~/earth-rover/scripts/hotkeys/trike-start.sh

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$HERE/_common.sh"

ER_TRIKE_UNIT="${ER_TRIKE_UNIT:-er-hotkey-trike.service}"
ER_TRIKE_TARGET="${ER_TRIKE_TARGET:-system-launch}"

er_systemd_run_ros \
    "$ER_TRIKE_UNIT" \
    "Earth Rover trike system launch (hotkey)" \
    "SIGINT" \
    "20" \
    "$ER_TRIKE_TARGET"
