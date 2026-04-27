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
KIOSK_DESKTOP="${EARTH_ROVER_HOME}/scripts/startup/earth-rover-kiosk.desktop"

ENABLE_MISSION=0
ENABLE_KIOSK=0
NO_AUTOSTART=0
PASSTHROUGH=()

usage() {
    cat <<EOF
Usage: install_user_units.sh [options]

Generates systemd --user units from mission.yaml and (optionally) wires up
the Firefox kiosk autostart for graphical sessions.

Options:
  --enable-mission    systemctl --user enable --now er-mission.target after install
  --enable-kiosk      symlink earth-rover-kiosk.desktop into ~/.config/autostart/
  --no-autostart      do not touch ~/.config/autostart/ at all
  --dry-run           show generated units without writing
  -h, --help          this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --enable-mission) ENABLE_MISSION=1 ;;
        --enable-kiosk)   ENABLE_KIOSK=1 ;;
        --no-autostart)   NO_AUTOSTART=1 ;;
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

# ---- XDG autostart for the kiosk ---------------------------------------------
if [ "$NO_AUTOSTART" -eq 0 ] && [ "$ENABLE_KIOSK" -eq 1 ]; then
    mkdir -p "$AUTOSTART_DIR"
    ln -sf "$KIOSK_DESKTOP" "$AUTOSTART_DIR/earth-rover-kiosk.desktop"
    echo "[install_user_units] enabled kiosk autostart via $AUTOSTART_DIR/earth-rover-kiosk.desktop"
elif [ "$NO_AUTOSTART" -eq 0 ]; then
    echo "[install_user_units] kiosk autostart NOT enabled.  Enable with:"
    echo "    ln -sf $KIOSK_DESKTOP $AUTOSTART_DIR/earth-rover-kiosk.desktop"
fi

# ---- enable the mission target -----------------------------------------------
if [ "$ENABLE_MISSION" -eq 1 ]; then
    systemctl --user enable --now er-mission.target
    echo "[install_user_units] er-mission.target enabled and started."
    systemctl --user list-units 'er-*' --all | head -25
else
    echo
    echo "[install_user_units] To enable the full mission stack at login:"
    echo "    systemctl --user enable --now er-mission.target"
fi
