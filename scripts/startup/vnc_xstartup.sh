#!/bin/sh
# Session inside the VNC desktop. XFCE, not GNOME: a second GNOME session for the user who
# is already logged in at the console fails ("Oh no! Something has gone wrong").
# GDK_BACKEND: without WAYLAND_DISPLAY, GTK still tries the default wayland-0 socket, finds
# the console's GNOME session there, and XFCE would hand DISPLAY=wayland-0 to its components.
unset SESSION_MANAGER DBUS_SESSION_BUS_ADDRESS WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11 XDG_CURRENT_DESKTOP=XFCE XDG_SESSION_DESKTOP=xfce GDK_BACKEND=x11
exec dbus-run-session -- startxfce4

