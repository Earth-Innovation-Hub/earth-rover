#!/bin/bash
# Block until every listed piece of hardware is present, then exit 0.
# Used as ExecStartPre= by the always-on sensor units, so a sensor that is not on
# the rig leaves its unit waiting ("activating") instead of crash-looping, and the
# unit starts by itself when the sensor is plugged in.
#
#   wait_for_hw.sh usb:1e10:3300                 # USB vendor:product (lsusb -d)
#   wait_for_hw.sh path:/dev/serial/by-id/...    # device node
#   wait_for_hw.sh udp:2368                      # a datagram arrives on this port
#
# HW_WAIT_TIMEOUT=<sec> gives up after that long (exit 1); default waits forever.

set -u
INTERVAL="${HW_WAIT_INTERVAL:-5}"
TIMEOUT="${HW_WAIT_TIMEOUT:-0}"

present() {
    local kind="${1%%:*}" arg="${1#*:}"
    case "$kind" in
        usb)  lsusb -d "$arg" >/dev/null 2>&1 ;;
        path) [ -e "$arg" ] ;;
        udp)  python3 - "$arg" <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.settimeout(2.0)
try:
    s.bind(("", int(sys.argv[1])))
    s.recvfrom(64)
except OSError:
    sys.exit(1)
PY
              ;;
        *)    echo "wait_for_hw: unknown spec '$1'" >&2; exit 2 ;;
    esac
}

[ $# -gt 0 ] || { echo "usage: wait_for_hw.sh <usb:VID:PID|path:/dev/x|udp:PORT>..." >&2; exit 2; }

start=$SECONDS
for spec in "$@"; do
    announced=0
    until present "$spec"; do
        if [ "$announced" = 0 ]; then
            echo "waiting for $spec"
            announced=1
        fi
        if [ "$TIMEOUT" != 0 ] && [ $((SECONDS - start)) -ge "$TIMEOUT" ]; then
            echo "gave up waiting for $spec after ${TIMEOUT}s" >&2
            exit 1
        fi
        sleep "$INTERVAL"
    done
    [ "$announced" = 1 ] && echo "found $spec"
done
exit 0
