# django view to serve template html file
# Path: vehicle_control_station/views.py
import shutil
import subprocess
import time
from django.shortcuts import render
from django.http import JsonResponse
from django.views.decorators.http import require_GET, require_POST


def ros_image_grid(request):
    return render(request, 'home.html', {})


def _read_cpu_usage():
    """Return aggregate CPU usage percent (0-100) using /proc/stat. Linux only."""
    try:
        with open('/proc/stat') as f:
            line = f.readline()
        # cpu  user nice system idle iowait irq softirq steal guest guest_nice
        parts = line.split()
        if parts[0] != 'cpu' or len(parts) < 5:
            return None
        values = [int(x) for x in parts[1:5]]
        user, nice, system, idle = values
        total = user + nice + system + idle
        time.sleep(0.15)
        with open('/proc/stat') as f:
            line = f.readline()
        parts = line.split()
        if parts[0] != 'cpu' or len(parts) < 5:
            return None
        values2 = [int(x) for x in parts[1:5]]
        user2, nice2, system2, idle2 = values2
        total2 = user2 + nice2 + system2 + idle2
        total_delta = total2 - total
        idle_delta = idle2 - idle
        if total_delta <= 0:
            return 0.0
        return round(100.0 * (1.0 - idle_delta / total_delta), 1)
    except (OSError, ValueError, IndexError):
        return None


def system_info(request):
    """GET /api/system-info/ — disk usage and CPU percent for System Info panel."""
    if request.method != 'GET':
        return JsonResponse({'status': 'error', 'message': 'Method not allowed'}, status=405)
    try:
        # Disk usage (root filesystem)
        usage = shutil.disk_usage('/')
        total_gb = round(usage.total / (1024 ** 3), 2)
        free_gb = round(usage.free / (1024 ** 3), 2)
        used_gb = round(usage.used / (1024 ** 3), 2)
        used_percent = round(100.0 * usage.used / usage.total, 1) if usage.total else 0

        cpu_percent = _read_cpu_usage()

        data = {
            'status': 'ok',
            'disk': {
                'used_gb': used_gb,
                'free_gb': free_gb,
                'total_gb': total_gb,
                'used_percent': used_percent,
            },
            'cpu_percent': cpu_percent,
        }
        return JsonResponse(data)
    except OSError as e:
        return JsonResponse({'status': 'error', 'message': str(e)}, status=500)


@require_GET
def recording_status(request):
    """GET /api/recording/status/ — current recording state. Stub when no recorder backend."""
    return JsonResponse({
        'status': 'ok',
        'recording': False,
        'output_file': None,
        'pid': None,
    })


@require_POST
def recording_start(request):
    """POST /api/recording/start/ — start rosbag recording. Stub when no recorder backend."""
    return JsonResponse({
        'status': 'error',
        'message': 'Recording not configured on server',
        'recording': False,
    }, status=501)


@require_POST
def recording_stop(request):
    """POST /api/recording/stop/ — stop recording. Stub when no recorder backend."""
    return JsonResponse({
        'status': 'ok',
        'recording': False,
        'output_file': None,
        'message': 'No active recording',
    })


def _get_ros_nodes():
    """Try to get ROS2 node list (when Django runs on same machine as ROS)."""
    import os
    try:
        out = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=os.environ.copy(),
        )
        if out.returncode == 0 and out.stdout:
            nodes = [n.strip() for n in out.stdout.splitlines() if n.strip()]
            return nodes
    except (FileNotFoundError, subprocess.TimeoutExpired, Exception):
        pass
    return []


@require_GET
def ros_nodes(request):
    """GET /api/ros-nodes/ — list of ROS nodes (from ros2 node list when available)."""
    nodes = _get_ros_nodes()
    ros_version = 'ROS2'
    return JsonResponse({
        'status': 'ok',
        'nodes': nodes,
        'count': len(nodes),
        'ros_version': ros_version,
    })

