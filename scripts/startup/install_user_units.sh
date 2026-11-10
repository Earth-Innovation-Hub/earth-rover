#!/bin/bash
# Earth Rover  --  Install systemd --user units from mission.yaml
#
# Generates er-<id>.service for every service in scripts/startup/mission.yaml
# plus one target per `targets:` entry, removes generated units the manifest
# no longer lists, installs the hand-written sidecars (er-deepgis*, archive),
# and enables the targets marked `boot: true` (plus er-deepgis.target).
#
# Needs a lingering user manager so the targets start at boot without a login:
#   loginctl enable-linger $USER

set -euo pipefail

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
USER_UNIT_DIR="${HOME}/.config/systemd/user"
AUTOSTART_DIR="${HOME}/.config/autostart"
MISSION_YAML="${MISSION_YAML:-${EARTH_ROVER_HOME}/scripts/startup/mission.yaml}"
GENERATOR="${EARTH_ROVER_HOME}/scripts/startup/install_user_units.py"
LEGACY_KIOSK_AUTOSTART="${AUTOSTART_DIR}/earth-rover-kiosk.desktop"

DRY_RUN=0
ENABLE=1

usage() {
    cat <<EOF
Usage: install_user_units.sh [options]

Generates ~/.config/systemd/user/er-*.service and the er-*.target units from
mission.yaml, installs the hand-written sidecars, reloads the user manager and
enables the boot targets. Nothing is started; use 'make mission-up' (or reboot).

Options:
  --dry-run      show generated units without writing
  --no-enable    install units but do not enable any target
  -h, --help     this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)    DRY_RUN=1 ;;
        --no-enable)  ENABLE=0 ;;
        -h|--help)    usage; exit 0 ;;
        *)            echo "unknown option: $1" >&2; usage; exit 2 ;;
    esac
    shift
done

if [ ! -f "$MISSION_YAML" ]; then
    echo "[install_user_units] mission YAML not found: $MISSION_YAML" >&2
    exit 1
fi

if [ "$DRY_RUN" = 1 ]; then
    exec python3 "$GENERATOR" --mission "$MISSION_YAML" --output-dir "$USER_UNIT_DIR" --dry-run
fi

mkdir -p "$USER_UNIT_DIR"
python3 "$GENERATOR" --mission "$MISSION_YAML" --output-dir "$USER_UNIT_DIR"

# ---- Hand-written sidecar units (not generated; see their headers) ----------
for sidecar in er-rsync-archive.service er-rsync-archive.timer \
        er-deepgis.target er-deepgis-mavros.service er-deepgis-telemetry.service er-deepgis-pose.service er-deepgis-keyframe.service; do
    src="${EARTH_ROVER_HOME}/scripts/startup/${sidecar}"
    if [ -f "$src" ]; then
        install -m 0644 "$src" "${USER_UNIT_DIR}/${sidecar}"
        echo "[install_user_units] installed sidecar unit ${sidecar}"
    fi
done
systemctl --user daemon-reload

# ---- Tear down legacy XDG autostart (kiosk is systemd-managed) --------------
if [ -L "$LEGACY_KIOSK_AUTOSTART" ] || [ -f "$LEGACY_KIOSK_AUTOSTART" ]; then
    rm -f "$LEGACY_KIOSK_AUTOSTART"
    echo "[install_user_units] removed legacy XDG autostart entry: $LEGACY_KIOSK_AUTOSTART"
fi

BOOT_TARGETS="er-deepgis.target $(python3 "$GENERATOR" --mission "$MISSION_YAML" --output-dir "$USER_UNIT_DIR" --print-boot-targets)"
if [ "$ENABLE" = 1 ]; then
    for t in $BOOT_TARGETS; do
        systemctl --user enable "$t" >/dev/null 2>&1 && echo "[install_user_units] enabled $t (starts at boot)"
    done
fi

cat <<EOF

[install_user_units] Done. Nothing was started or restarted.

Boot targets: ${BOOT_TARGETS}
  make -C "${EARTH_ROVER_HOME}" mission-up       # start them now
  make -C "${EARTH_ROVER_HOME}" mission-status   # one line per er-* unit
  journalctl --user -u er-grasshopper -f          # one unit's log

A sensor that is not plugged in leaves its unit "activating" (waiting for the
hardware); it starts by itself when the sensor appears.

On demand:
  systemctl --user start er-record.service        # rosbag2 recording
  systemctl --user start er-rqt.service           # rqt on the rover display
  systemctl --user start er-console.target        # rosbridge + web video + VCS
EOF
