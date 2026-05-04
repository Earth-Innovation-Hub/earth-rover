#!/bin/bash
# Hotkey: start a rosbag recording.
#
# Idempotent. Spawns `make $ER_BAG_TARGET` inside `er-hotkey-bag.service`.
# Default target is `record-bag-mavros` (light: MAVROS + tf + diagnostics +
# adsb, no cameras). Override at press time via env var:
#
#   ER_BAG_TARGET=record-bag        ~/.../rosbag-start.sh   # full
#   ER_BAG_TARGET=record-bag-stereo ~/.../rosbag-start.sh   # cameras only
#
# Bound by default to Ctrl+Alt+Super+B.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$HERE/_common.sh"

ER_BAG_UNIT="${ER_BAG_UNIT:-er-hotkey-bag.service}"
ER_BAG_TARGET="${ER_BAG_TARGET:-record-bag-mavros}"

er_systemd_run_ros \
    "$ER_BAG_UNIT" \
    "Earth Rover rosbag recording (hotkey, target=$ER_BAG_TARGET)" \
    "SIGINT" \
    "30" \
    "$ER_BAG_TARGET"
