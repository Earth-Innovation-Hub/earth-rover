#!/bin/bash
# Idempotently install GNOME custom keybindings for the four trike hotkeys.
#
# Default bindings (Ctrl+Alt+Super-heavy so you cannot fire them by accident):
#   Ctrl+Alt+Super+T -> trike-start.sh
#   Ctrl+Alt+Super+Q -> trike-stop.sh   (double-tap inside 3s)
#   Ctrl+Alt+Super+B -> rosbag-start.sh
#   Ctrl+Alt+Super+E -> rosbag-stop.sh
#
# Override the binding for any action via env vars before running:
#   ER_BIND_TRIKE_START='<Ctrl><Alt><Super>F9' ./install-gnome.sh
#   ER_BIND_TRIKE_STOP='<Ctrl><Alt><Super>F10' ./install-gnome.sh
#   ER_BIND_BAG_START='<Ctrl><Alt><Super>F11'  ./install-gnome.sh
#   ER_BIND_BAG_STOP='<Ctrl><Alt><Super>F12'   ./install-gnome.sh
#
# This script reads the existing `custom-keybindings` list, ADDS our four
# entries (or updates them in place if names match) without clobbering
# anything else, and writes the list back. Safe to re-run.
#
# Uninstall with ./uninstall-gnome.sh (or `make hotkeys-uninstall`).

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v gsettings >/dev/null 2>&1; then
    echo "gsettings not found -- this installer is GNOME/Cinnamon-only." >&2
    echo "On other desktops use the alternatives in scripts/hotkeys/README.md." >&2
    exit 1
fi

# These names are how we identify our entries on later re-installs / uninstall.
TRIKE_START_NAME="Earth Rover :: trike start"
TRIKE_STOP_NAME="Earth Rover :: trike stop (double-tap)"
BAG_START_NAME="Earth Rover :: rosbag start"
BAG_STOP_NAME="Earth Rover :: rosbag stop"

TRIKE_START_BIND="${ER_BIND_TRIKE_START:-<Ctrl><Alt><Super>t}"
TRIKE_STOP_BIND="${ER_BIND_TRIKE_STOP:-<Ctrl><Alt><Super>q}"
BAG_START_BIND="${ER_BIND_BAG_START:-<Ctrl><Alt><Super>b}"
BAG_STOP_BIND="${ER_BIND_BAG_STOP:-<Ctrl><Alt><Super>e}"

TRIKE_START_CMD="$HERE/trike-start.sh"
TRIKE_STOP_CMD="$HERE/trike-stop.sh"
BAG_START_CMD="$HERE/rosbag-start.sh"
BAG_STOP_CMD="$HERE/rosbag-stop.sh"

chmod +x "$TRIKE_START_CMD" "$TRIKE_STOP_CMD" "$BAG_START_CMD" "$BAG_STOP_CMD" \
         "$HERE/_common.sh" "$HERE/status.sh" 2>/dev/null || true

KEY_SCHEMA="org.gnome.settings-daemon.plugins.media-keys"
KEY_PATH_PREFIX="/org/gnome/settings-daemon/plugins/media-keys/custom-keybindings"
ENTRY_SCHEMA="org.gnome.settings-daemon.plugins.media-keys.custom-keybinding"

# Read existing list. gsettings prints either '@as []' or a Python-ish list.
existing_raw="$(gsettings get "$KEY_SCHEMA" custom-keybindings 2>/dev/null || echo '@as []')"

# Use Python to parse / dedupe / normalize.
python3 - "$existing_raw" <<'PY' > /tmp/_er_existing_paths.$$
import ast, sys
raw = sys.argv[1]
# Strip "@as " prefix that gsettings sometimes prints for empty arrays.
raw = raw.replace('@as ', '', 1).strip()
try:
    paths = ast.literal_eval(raw) if raw else []
except Exception:
    paths = []
for p in paths:
    print(p)
PY
mapfile -t existing_paths < /tmp/_er_existing_paths.$$
rm -f /tmp/_er_existing_paths.$$

# Find the highest custom<N> index already in use, so we never collide.
max_idx=-1
for p in "${existing_paths[@]}"; do
    if [[ "$p" =~ /custom([0-9]+)/?$ ]]; then
        n="${BASH_REMATCH[1]}"
        (( n > max_idx )) && max_idx=$n
    fi
done

# For each of OUR four entries, find an existing path whose `name` matches us
# (so reruns update in place); otherwise allocate a new index.
allocate_path() {
    local target_name="$1"
    for p in "${existing_paths[@]}"; do
        local n
        n="$(gsettings get "$ENTRY_SCHEMA:$p" name 2>/dev/null || echo "''")"
        n="${n%\'}"; n="${n#\'}"
        if [ "$n" = "$target_name" ]; then
            echo "$p"
            return
        fi
    done
    max_idx=$(( max_idx + 1 ))
    echo "$KEY_PATH_PREFIX/custom$max_idx/"
}

TRIKE_START_PATH="$(allocate_path "$TRIKE_START_NAME")"
TRIKE_STOP_PATH="$(allocate_path "$TRIKE_STOP_NAME")"
BAG_START_PATH="$(allocate_path "$BAG_START_NAME")"
BAG_STOP_PATH="$(allocate_path "$BAG_STOP_NAME")"

set_entry() {
    local path="$1" name="$2" cmd="$3" bind="$4"
    gsettings set "$ENTRY_SCHEMA:$path" name    "$name"
    gsettings set "$ENTRY_SCHEMA:$path" command "$cmd"
    gsettings set "$ENTRY_SCHEMA:$path" binding "$bind"
}

set_entry "$TRIKE_START_PATH" "$TRIKE_START_NAME" "$TRIKE_START_CMD" "$TRIKE_START_BIND"
set_entry "$TRIKE_STOP_PATH"  "$TRIKE_STOP_NAME"  "$TRIKE_STOP_CMD"  "$TRIKE_STOP_BIND"
set_entry "$BAG_START_PATH"   "$BAG_START_NAME"   "$BAG_START_CMD"   "$BAG_START_BIND"
set_entry "$BAG_STOP_PATH"    "$BAG_STOP_NAME"    "$BAG_STOP_CMD"    "$BAG_STOP_BIND"

# Build the merged path list (existing + ours, deduped, ordered).
declare -A seen=()
out=()
add() { local p="$1"; [ -z "${seen[$p]:-}" ] && { out+=("$p"); seen[$p]=1; }; }
for p in "${existing_paths[@]}"; do add "$p"; done
add "$TRIKE_START_PATH"
add "$TRIKE_STOP_PATH"
add "$BAG_START_PATH"
add "$BAG_STOP_PATH"

# Write list back as a gsettings array literal.
list="["
for p in "${out[@]}"; do list+="'$p', "; done
list="${list%, }]"
gsettings set "$KEY_SCHEMA" custom-keybindings "$list"

cat <<EOF
Installed Earth Rover hotkeys (GNOME custom-keybindings):
  $TRIKE_START_BIND  ->  trike-start
  $TRIKE_STOP_BIND  ->  trike-stop  (double-tap inside ${ER_DOUBLE_TAP_WINDOW:-3}s)
  $BAG_START_BIND  ->  rosbag-start  (target=${ER_BAG_TARGET:-record-bag-mavros})
  $BAG_STOP_BIND  ->  rosbag-stop

Verify:    gsettings list-recursively $ENTRY_SCHEMA | grep 'Earth Rover'
Uninstall: $HERE/uninstall-gnome.sh   (or: make hotkeys-uninstall)

If a key combo doesn't fire, open Settings -> Keyboard -> View and Customize
Shortcuts -> Custom Shortcuts to confirm GNOME accepted the binding (some
distros remap <Super> to the gnome-shell overview; add an extra modifier in
that case).
EOF
