#!/bin/bash
# Earth Rover  --  Install systemd --user units from mission.yaml
#
# Reads scripts/startup/mission.yaml and emits one .service per service entry
# plus an aggregating er-mission.target that pulls them all up in dependency
# order via After= / Wants= / Requires=.
#
# After install:
#   systemctl --user daemon-reload
#   systemctl --user enable --now er-mission.target
#   systemctl --user list-units 'er-*'
#
# To survive logout:
#   loginctl enable-linger $USER

set -euo pipefail

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
USER_UNIT_DIR="${HOME}/.config/systemd/user"
AUTOSTART_DIR="${HOME}/.config/autostart"
MISSION_YAML="${MISSION_YAML:-${EARTH_ROVER_HOME}/scripts/startup/mission.yaml}"
LEGACY_KIOSK_AUTOSTART="${AUTOSTART_DIR}/earth-rover-kiosk.desktop"

ENABLE_MISSION=0
PASSTHROUGH=()

usage() {
    cat <<EOF
Usage: install_user_units.sh [options]

Generates systemd --user units from mission.yaml.

The kiosk service (er-kiosk.service, marked needs_display in mission.yaml) is
bound to graphical-session.target, so it fires automatically once GNOME (or
any logind graphical session) comes up.  We deliberately do NOT install an
XDG autostart .desktop entry to avoid double-firing Firefox.

Options:
  --enable-mission    systemctl --user enable --now er-mission.target after install
  --dry-run           show generated units without writing
  -h, --help          this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --enable-mission) ENABLE_MISSION=1 ;;
        --dry-run)        PASSTHROUGH+=("--dry-run") ;;
        -h|--help)        usage; exit 0 ;;
        *)                PASSTHROUGH+=("$1") ;;
    esac
    shift
done

mkdir -p "$USER_UNIT_DIR"

if [ ! -f "$MISSION_YAML" ]; then
    echo "[install_user_units] mission YAML not found: $MISSION_YAML" >&2
    exit 1
fi

python3 "${EARTH_ROVER_HOME}/scripts/startup/install_user_units.py" \
    --mission "$MISSION_YAML" \
    --output-dir "$USER_UNIT_DIR" \
    "${PASSTHROUGH[@]}"

# ---- Tear down legacy XDG autostart (kiosk is now systemd-managed) ----------
if [ -L "$LEGACY_KIOSK_AUTOSTART" ] || [ -f "$LEGACY_KIOSK_AUTOSTART" ]; then
    rm -f "$LEGACY_KIOSK_AUTOSTART"
    echo "[install_user_units] removed legacy XDG autostart entry: $LEGACY_KIOSK_AUTOSTART"
    echo "    (kiosk now fires via er-kiosk.service bound to graphical-session.target)"
fi

# ---- enable the mission target -----------------------------------------------
if [ "$ENABLE_MISSION" -eq 1 ]; then
    systemctl --user enable --now er-mission.target
    systemctl --user enable er-kiosk.service || true
    echo "[install_user_units] er-mission.target enabled and started."
    echo "[install_user_units] er-kiosk.service enabled (will fire on graphical login)."
    systemctl --user list-units 'er-*' --all | head -25
else
    echo
    echo "[install_user_units] To enable the full mission stack at login:"
    echo "    systemctl --user enable --now er-mission.target"
    echo "    systemctl --user enable er-kiosk.service       # fires on graphical login"
fi
