#!/bin/bash
# Build the Python environment er-vcs.service runs in (no sudo needed).
# --system-site-packages keeps the ROS / distro packages importable.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${VCS_VENV:-$HOME/.local/share/earth-rover/vcs-venv}"

if [ ! -x "$VENV/bin/python" ]; then
    mkdir -p "$(dirname "$VENV")"
    python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/python" -m pip install --quiet --upgrade pip
"$VENV/bin/python" -m pip install --quiet -r "$HERE/requirements.txt"
"$VENV/bin/python" -c "import django, whitenoise, gunicorn; print('vcs venv:', '$VENV', 'django', django.get_version())"
