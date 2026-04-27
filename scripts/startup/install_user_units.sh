#!/bin/bash
# Earth Rover  --  Install systemd --user units from mission.yaml
#
# Reads scripts/startup/mission.yaml and emits one .service per service entry
# plus an aggregating er-mission.target that pulls them all up in dependency
# order via After= / Wants= / Requires=.
#
# This script ONLY generates and installs the unit files; it does NOT enable
# anything to auto-start at boot/login.  Every launch is operator-driven.
# Use the helpers in the top-level Makefile (or `systemctl --user start ...`
# directly) to bring the stack up:
#
#   make mission-up      # the whole graph
#   make ui-up           # just the web-frontend layer (vcs + rosbridge + ...)
#   make archive-now     # one-shot SSD->NAS rsync
#
# To survive logout (headless rover):
#   loginctl enable-linger $USER

set -euo pipefail

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
USER_UNIT_DIR="${HOME}/.config/systemd/user"
AUTOSTART_DIR="${HOME}/.config/autostart"
MISSION_YAML="${MISSION_YAML:-${EARTH_ROVER_HOME}/scripts/startup/mission.yaml}"
LEGACY_KIOSK_AUTOSTART="${AUTOSTART_DIR}/earth-rover-kiosk.desktop"

PASSTHROUGH=()

usage() {
    cat <<EOF
Usage: install_user_units.sh [options]

Generates ~/.config/systemd/user/er-*.service from mission.yaml and reloads
the user systemd daemon.  Nothing auto-starts -- every service is launched
manually by the operator (CLI, Makefile, or the Django mission console).

The kiosk service is bound to graphical-session.target so that IF the
operator chooses to enable it (\`systemctl --user enable er-kiosk.service\`)
it fires on graphical login without double-firing through XDG autostart.
By default it is NOT enabled.

Options:
  --dry-run           show generated units without writing
  -h, --help          this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --enable-mission)
            echo "[install_user_units] --enable-mission has been removed." >&2
            echo "    Auto-enable was deliberately dropped: every launch is manual." >&2
            echo "    Use:  make mission-up   (or: systemctl --user start er-mission.target)" >&2
            exit 2
            ;;
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

# ---- Hand-written sidecar units (kept alongside the auto-generated ones) ----
# These are NOT part of the mission graph -- they cover periodic maintenance
# like the SSD->NAS archive rsync.
for sidecar in er-rsync-archive.service er-rsync-archive.timer; do
    src="${EARTH_ROVER_HOME}/scripts/startup/${sidecar}"
    if [ -f "$src" ]; then
        install -m 0644 "$src" "${USER_UNIT_DIR}/${sidecar}"
        echo "[install_user_units] installed sidecar unit ${sidecar}"
    fi
done

# ---- Tear down legacy XDG autostart (kiosk is now systemd-managed) ----------
if [ -L "$LEGACY_KIOSK_AUTOSTART" ] || [ -f "$LEGACY_KIOSK_AUTOSTART" ]; then
    rm -f "$LEGACY_KIOSK_AUTOSTART"
    echo "[install_user_units] removed legacy XDG autostart entry: $LEGACY_KIOSK_AUTOSTART"
    echo "    (kiosk now fires via er-kiosk.service bound to graphical-session.target)"
fi

# ---- done -- everything is manual --------------------------------------------
cat <<EOF

[install_user_units] Done.  No services were enabled or started.

To bring the mission stack up manually (preferred):
    make -C "${EARTH_ROVER_HOME}" mission-up
    make -C "${EARTH_ROVER_HOME}" mission-status
    make -C "${EARTH_ROVER_HOME}" mission-down

To bring up just the web-frontend layer (so the Django console can act as
a launch panel for individual services):
    make -C "${EARTH_ROVER_HOME}" ui-up

One-shot SSD->NAS archive rsync (no auto-timer):
    make -C "${EARTH_ROVER_HOME}" archive-now

Or talk to systemd directly:
    systemctl --user list-units 'er-*' --all
    systemctl --user start  er-mavros.service
    systemctl --user stop   er-mavros.service
    journalctl --user -u    er-mavros -f

If you ever want the old auto-on-boot behavior back (NOT recommended --
re-enable per service so you keep due-diligence per launch):
    systemctl --user enable --now er-mission.target            # whole graph
    systemctl --user enable      er-kiosk.service              # graphical login
    systemctl --user enable --now er-rsync-archive.timer       # daily archive
EOF
