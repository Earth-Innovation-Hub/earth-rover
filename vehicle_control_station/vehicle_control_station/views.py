"""Dashboard page and JSON APIs for the Vehicle Control Station."""

import os
import shutil
import subprocess
import time
from pathlib import Path

from django.conf import settings
from django.http import JsonResponse
from django.http.request import split_domain_port
from django.shortcuts import render
from django.views.decorators.http import require_GET, require_POST

RECORD_UNIT = 'er-record.service'
BAG_DIR = Path(os.path.expanduser(os.getenv('RECORD_BAG_DIR', '~/earth-rover-bags')))


def home(request):
    """Dashboard. rosbridge and web_video_server run on the host that served the page."""
    host, _ = split_domain_port(request.get_host())
    return render(request, 'home.html', {
        'ros_bridge_url': settings.ROS_BRIDGE_URL or f'ws://{host}:{settings.ROS_BRIDGE_PORT}',
        'video_server_url': settings.VIDEO_SERVER_URL or f'http://{host}:{settings.VIDEO_SERVER_PORT}',
    })


def _read_cpu_usage():
    """Return aggregate CPU usage percent (0-100) using /proc/stat. Linux only."""
    def sample():
        with open('/proc/stat') as f:
            parts = f.readline().split()
        if parts[0] != 'cpu' or len(parts) < 5:
            raise ValueError
        user, nice, system, idle = (int(x) for x in parts[1:5])
        return user + nice + system + idle, idle

    try:
        total, idle = sample()
        time.sleep(0.15)
        total2, idle2 = sample()
    except (OSError, ValueError, IndexError):
        return None
    if total2 - total <= 0:
        return 0.0
    return round(100.0 * (1.0 - (idle2 - idle) / (total2 - total)), 1)


@require_GET
def system_info(request):
    """GET /api/system-info/ — disk usage and CPU percent for the System Info panel."""
    try:
        usage = shutil.disk_usage(BAG_DIR if BAG_DIR.exists() else '/')
    except OSError as e:
        return JsonResponse({'status': 'error', 'message': str(e)}, status=500)
    gb = 1024 ** 3
    return JsonResponse({
        'status': 'ok',
        'disk': {
            'used_gb': round(usage.used / gb, 2),
            'free_gb': round(usage.free / gb, 2),
            'total_gb': round(usage.total / gb, 2),
            'used_percent': round(100.0 * usage.used / usage.total, 1) if usage.total else 0,
        },
        'cpu_percent': _read_cpu_usage(),
    })


# ---------- recording: drives er-record.service (record_bag.launch.py) --------

def _systemctl(*args, timeout=10):
    return subprocess.run(['systemctl', '--user', *args], capture_output=True, text=True, timeout=timeout)


def _recording_state():
    out = _systemctl('show', '--timestamp=unix', '--property=ActiveState,MainPID,Result,ExecMainStartTimestamp',
                     RECORD_UNIT, timeout=5).stdout
    props = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    recording = props.get('ActiveState') in ('active', 'activating')
    latest = None
    started = props.get('ExecMainStartTimestamp', '').lstrip('@')
    if recording and started.isdigit() and BAG_DIR.is_dir():
        # Only a bag directory created since the recorder started belongs to this run.
        bags = [p for p in BAG_DIR.iterdir() if p.is_dir() and p.stat().st_mtime >= int(started) - 1]
        latest = str(max(bags, key=lambda p: p.stat().st_mtime)) if bags else None
    pid = props.get('MainPID', '0')
    return {
        'recording': recording,
        'output_file': latest,
        'pid': int(pid) if pid.isdigit() and pid != '0' else None,
        'result': props.get('Result'),
    }


@require_GET
def recording_status(request):
    """GET /api/recording/status/"""
    try:
        return JsonResponse({'status': 'ok', **_recording_state()})
    except (OSError, subprocess.TimeoutExpired) as e:
        return JsonResponse({'status': 'error', 'message': str(e)}, status=500)


@require_POST
def recording_start(request):
    """POST /api/recording/start/ — start er-record.service."""
    try:
        out = _systemctl('start', RECORD_UNIT, timeout=30)
        if out.returncode != 0:
            return JsonResponse({'status': 'error', 'message': out.stderr.strip() or 'start failed',
                                 'recording': False}, status=500)
        time.sleep(2)
        state = _recording_state()
        if not state['recording']:
            return JsonResponse({'status': 'error', 'recording': False,
                                 'message': f"recorder exited ({state['result']}); "
                                            f"journalctl --user -u {RECORD_UNIT}"}, status=500)
        return JsonResponse({'status': 'ok', **state})
    except (OSError, subprocess.TimeoutExpired) as e:
        return JsonResponse({'status': 'error', 'message': str(e), 'recording': False}, status=500)


@require_POST
def recording_stop(request):
    """POST /api/recording/stop/ — stop er-record.service (SIGINT closes the bag cleanly)."""
    try:
        before = _recording_state()
        out = _systemctl('stop', RECORD_UNIT, timeout=40)
        if out.returncode != 0:
            return JsonResponse({'status': 'error', 'message': out.stderr.strip() or 'stop failed'}, status=500)
        return JsonResponse({'status': 'ok', 'recording': False, 'output_file': before['output_file']})
    except (OSError, subprocess.TimeoutExpired) as e:
        return JsonResponse({'status': 'error', 'message': str(e)}, status=500)


# ---------- ROS diagnostics ----------------------------------------------------

def _get_ros_nodes():
    """ROS 2 node list (Django runs on the rover with ROS sourced by its unit)."""
    try:
        out = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True,
                             timeout=5, env=os.environ.copy())
    except (OSError, subprocess.TimeoutExpired):
        return []
    if out.returncode != 0:
        return []
    return [n.strip() for n in out.stdout.splitlines() if n.strip()]


@require_GET
def ros_nodes(request):
    """GET /api/ros-nodes/"""
    nodes = _get_ros_nodes()
    return JsonResponse({'status': 'ok', 'nodes': nodes, 'count': len(nodes), 'ros_version': 'ROS 2'})
