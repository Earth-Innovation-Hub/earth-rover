#!/bin/bash
# Earth Rover  --  Firefox Kiosk Launcher
#
# Waits for the Vehicle Control Station to be reachable, then launches Firefox
# in fullscreen kiosk mode pointed at the given URL.
#
# Usage:    firefox_kiosk.sh [URL]    (default: http://localhost:8000/mission/)
# Env:
#   VCS_PORT        (default 8000) — used for the readiness probe
#   KIOSK_TIMEOUT   (default 120)  — seconds to wait for VCS before giving up
#   KIOSK_PROFILE   (default $HOME/.mozilla/firefox/kiosk) — dedicated profile

set -e

URL="${1:-http://localhost:8000/mission/}"
VCS_PORT="${VCS_PORT:-8000}"
KIOSK_TIMEOUT="${KIOSK_TIMEOUT:-120}"
KIOSK_PROFILE="${KIOSK_PROFILE:-$HOME/.mozilla/firefox/kiosk}"

# Ensure we have a graphical session — kiosk Firefox needs DISPLAY.
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    export DISPLAY=":0"
fi

# Wait for VCS to be reachable.
echo "[firefox_kiosk] Waiting up to ${KIOSK_TIMEOUT}s for $URL ..."
deadline=$(( $(date +%s) + KIOSK_TIMEOUT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    if curl -fsS -o /dev/null --max-time 2 "http://localhost:${VCS_PORT}/"; then
        echo "[firefox_kiosk] VCS is up."
        break
    fi
    sleep 2
done

if ! curl -fsS -o /dev/null --max-time 2 "http://localhost:${VCS_PORT}/"; then
    echo "[firefox_kiosk] WARNING: VCS not reachable at :$VCS_PORT after $KIOSK_TIMEOUT s; opening anyway."
fi

# Provision a dedicated kiosk profile so we don't pollute the user's main
# Firefox profile with kiosk overrides.
mkdir -p "$KIOSK_PROFILE"
if [ ! -f "$KIOSK_PROFILE/user.js" ]; then
    cat > "$KIOSK_PROFILE/user.js" <<'PREFS'
// Earth Rover kiosk-mode prefs.
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.tabs.warnOnClose", false);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.aboutwelcome.enabled", false);
user_pref("datareporting.policy.firstRunURL", "");
user_pref("toolkit.telemetry.reportingpolicy.firstRun", false);
user_pref("dom.disable_open_during_load", false);
PREFS
fi

# Pick the firefox binary (snap or apt), preferring the one that comes
# first in PATH.
FIREFOX_BIN="${FIREFOX_BIN:-$(command -v firefox || true)}"
if [ -z "$FIREFOX_BIN" ]; then
    echo "[firefox_kiosk] ERROR: firefox not found in PATH" >&2
    exit 1
fi

echo "[firefox_kiosk] Launching $FIREFOX_BIN --kiosk $URL"
exec "$FIREFOX_BIN" \
    --profile "$KIOSK_PROFILE" \
    --kiosk \
    --new-instance \
    "$URL"
