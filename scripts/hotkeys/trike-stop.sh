#!/bin/bash
# Hotkey: stop the trike system launch (double-tap confirm).
#
# First press arms a confirm timer (notify-send shows "press again within 3s")
# and exits. Second press inside the window actually stops the
# er-hotkey-trike.service unit by sending SIGINT to the whole cgroup, then
# escalating to SIGKILL if the launch tree refuses to drop within 20s.
#
# Bound by default to Ctrl+Alt+Super+Q.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$HERE/_common.sh"

ER_TRIKE_UNIT="${ER_TRIKE_UNIT:-er-hotkey-trike.service}"

# If unit is not running, no need to confirm -- inform and exit.
if ! er_unit_active "$ER_TRIKE_UNIT"; then
    er_notify low "trike not running" "$ER_TRIKE_UNIT is inactive"
    exit 0
fi

if ! er_double_tap_check "trike-stop"; then
    exit 0
fi

er_systemd_stop "$ER_TRIKE_UNIT" "trike"
