#!/bin/bash
# Remove the four Earth Rover entries from GNOME custom-keybindings.
# Identifies entries by the "Earth Rover :: ..." name prefix so it never
# touches anything else the user has bound.

set -euo pipefail

if ! command -v gsettings >/dev/null 2>&1; then
    echo "gsettings not found -- this uninstaller is GNOME/Cinnamon-only." >&2
    exit 1
fi

KEY_SCHEMA="org.gnome.settings-daemon.plugins.media-keys"
ENTRY_SCHEMA="org.gnome.settings-daemon.plugins.media-keys.custom-keybinding"

raw="$(gsettings get "$KEY_SCHEMA" custom-keybindings 2>/dev/null || echo '@as []')"

python3 - "$raw" <<'PY' > /tmp/_er_uninst_paths.$$
import ast, sys
raw = sys.argv[1].replace('@as ', '', 1).strip()
try:
    paths = ast.literal_eval(raw) if raw else []
except Exception:
    paths = []
for p in paths:
    print(p)
PY
mapfile -t paths < /tmp/_er_uninst_paths.$$
rm -f /tmp/_er_uninst_paths.$$

removed=0
remaining=()
for p in "${paths[@]}"; do
    n="$(gsettings get "$ENTRY_SCHEMA:$p" name 2>/dev/null || echo "''")"
    n="${n%\'}"; n="${n#\'}"
    if [[ "$n" == Earth\ Rover\ ::* ]]; then
        # Reset the entry's keys, then drop from the list.
        gsettings reset-recursively "$ENTRY_SCHEMA:$p" 2>/dev/null || true
        echo "removed: $p ($n)"
        removed=$((removed + 1))
    else
        remaining+=("$p")
    fi
done

list="["
for p in "${remaining[@]}"; do list+="'$p', "; done
list="${list%, }]"
[ "$list" = "[" ] && list="@as []"
gsettings set "$KEY_SCHEMA" custom-keybindings "$list"

echo "Uninstalled $removed Earth Rover hotkey(s)."
