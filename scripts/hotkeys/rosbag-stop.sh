#!/bin/bash
# Hotkey: stop the rosbag recording cleanly.
#
# Single-tap (no double-tap): losing a partial bag is recoverable, and the
# longer the user has to wait, the more bytes are dropped. KillSignal=SIGINT
# is configured on the unit so `systemctl --user stop` lets ros2 bag finalize
# the metadata.yaml before the process exits.
#
# Bound by default to Ctrl+Alt+Super+E.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$HERE/_common.sh"

ER_BAG_UNIT="${ER_BAG_UNIT:-er-hotkey-bag.service}"

er_systemd_stop "$ER_BAG_UNIT" "rosbag"
