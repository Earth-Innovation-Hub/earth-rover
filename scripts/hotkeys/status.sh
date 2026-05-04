#!/bin/bash
# Quick status report for all hotkey-managed units. Intended for terminal
# use ("make hotkeys-status") or as a one-shot debug aid.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "$HERE/_common.sh"

declare -a UNITS=(
    "er-hotkey-trike.service"
    "er-hotkey-bag.service"
)

printf '%-32s %-10s %-12s %s\n' "UNIT" "STATE" "MAIN_PID" "SINCE"
printf '%-32s %-10s %-12s %s\n' "----" "-----" "--------" "-----"
for u in "${UNITS[@]}"; do
    state="$(systemctl --user is-active "$u" 2>/dev/null || true)"
    pid="$(systemctl --user show -p MainPID --value "$u" 2>/dev/null || echo -)"
    since="$(systemctl --user show -p ActiveEnterTimestamp --value "$u" 2>/dev/null || echo -)"
    printf '%-32s %-10s %-12s %s\n' "$u" "${state:-unknown}" "${pid:-0}" "${since:--}"
done
echo
echo "Audit log tail (last 10):"
if [ -f "$ER_AUDIT_LOG" ]; then
    tail -n 10 "$ER_AUDIT_LOG"
else
    echo "  (no audit log yet at $ER_AUDIT_LOG)"
fi
echo
echo "Tail unit logs with:"
echo "  journalctl --user -fu er-hotkey-trike"
echo "  journalctl --user -fu er-hotkey-bag"
