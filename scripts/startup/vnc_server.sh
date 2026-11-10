#!/bin/bash
# TigerVNC XFCE desktop (er-vnc.service). Listens only on VNC_INTERFACE's IPv4 address, so
# the desktop is offered on that network and nowhere else; with no address yet (cable
# unplugged) it exits and systemd retries. Password: ~/.vnc/passwd (set with vncpasswd).
set -u
iface=${VNC_INTERFACE:-lo}
display=${VNC_DISPLAY:-:2}
addr=$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{split($4, a, "/"); print a[1]; exit}')
if [ -z "$addr" ]; then
    echo "no IPv4 address on $iface yet"
    exit 1
fi
if [ ! -s "$HOME/.vnc/passwd" ]; then
    echo "no VNC password; run vncpasswd"
    exit 1
fi
tigervncserver -kill "$display" -clean >/dev/null 2>&1 || true
echo "VNC desktop $display on $iface ($addr), port $((5900 + ${display#:}))"
# TLSVnc (encrypted) first; plain VncAuth for viewers without TLS, which tigervncserver
# only serves beyond localhost with --I-KNOW-THIS-IS-INSECURE.
exec tigervncserver "$display" -fg -autokill -localhost no --I-KNOW-THIS-IS-INSECURE -interface "$addr" \
    -geometry "${VNC_GEOMETRY:-1920x1080}" -depth 24 -SecurityTypes TLSVnc,VncAuth \
    -xstartup "$(dirname "$(readlink -f "$0")")/vnc_xstartup.sh"

