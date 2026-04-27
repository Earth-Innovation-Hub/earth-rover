"""Mission launch / dependency views.

Exposes:
    GET /mission/             -> HTML page rendering the live tier chart.
    GET /api/mission/         -> JSON payload combining mission.yaml and the
                                 current systemctl --user state of every unit.

The mission topology comes from ``earth-rover/scripts/startup/mission.yaml``;
runtime state comes from ``systemctl --user show er-<id>.service`` for each
service.  Falls back to "unknown" when systemd-user is unavailable (e.g. when
the Django app is run from a container that can't see the user bus).
"""

from __future__ import annotations

import os
import subprocess
from functools import lru_cache
from pathlib import Path

import yaml
from django.conf import settings
from django.http import JsonResponse
from django.shortcuts import render
from django.views.decorators.http import require_GET


# ---------- mission.yaml loading ---------------------------------------------

def _mission_yaml_path() -> Path:
    """Locate scripts/startup/mission.yaml relative to the Django project."""
    candidates = [
        os.environ.get("MISSION_YAML"),
        getattr(settings, "MISSION_YAML", None),
        # vehicle_control_station/ is two levels deep inside the repo.
        Path(settings.BASE_DIR).parent / "scripts" / "startup" / "mission.yaml",
        Path(__file__).resolve().parents[3] / "scripts" / "startup" / "mission.yaml",
    ]
    for c in candidates:
        if c and Path(c).exists():
            return Path(c)
    raise FileNotFoundError("mission.yaml not found in any expected location")


@lru_cache(maxsize=1)
def _load_mission_cached(mtime: float) -> dict:
    """Cache by file mtime so live edits to mission.yaml are picked up."""
    del mtime  # used only to bust the cache
    path = _mission_yaml_path()
    with path.open() as fh:
        return yaml.safe_load(fh) or {}


def _load_mission() -> dict:
    path = _mission_yaml_path()
    return _load_mission_cached(path.stat().st_mtime)


# ---------- systemd --user querying ------------------------------------------

# Properties we care about for each unit; ActiveState/SubState describe
# whether it's running, ExecMainStartTimestamp gives "started at".
_PROPS = (
    "Id",
    "ActiveState",
    "SubState",
    "Result",
    "ExecMainStartTimestamp",
    "ExecMainPID",
    "Description",
)


def _query_unit(unit: str) -> dict:
    """Return a {prop: value} dict for one er-<id>.service."""
    try:
        out = subprocess.run(
            ["systemctl", "--user", "show",
             "--property=" + ",".join(_PROPS), unit],
            capture_output=True, text=True, timeout=3,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return {"ActiveState": "unknown", "SubState": "unknown"}

    data = {}
    for line in out.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            data[k] = v
    if not data:
        return {"ActiveState": "unknown", "SubState": "unknown"}
    return data


# ---------- public views ------------------------------------------------------

def mission_page(request):
    """Render the HTML page; the actual data is fetched client-side via JSON."""
    return render(request, "mission.html", {})


@require_GET
def mission_status(request):
    """JSON: mission topology + live unit state, for the dashboard JS."""
    try:
        manifest = _load_mission()
    except FileNotFoundError as e:
        return JsonResponse({"status": "error", "message": str(e)}, status=500)

    services = manifest.get("services", []) or []
    mission = manifest.get("mission", {})

    enriched = []
    for svc in services:
        unit = f"er-{svc['id']}.service"
        state = _query_unit(unit)
        active = state.get("ActiveState", "unknown")
        sub = state.get("SubState", "unknown")

        if active == "active" and sub in ("running", "exited"):
            ui_state = "running" if sub == "running" else "ok"
        elif active == "activating":
            ui_state = "starting"
        elif active == "failed":
            ui_state = "failed"
        elif active == "inactive":
            ui_state = "stopped"
        else:
            ui_state = "unknown"

        enriched.append({
            "id": svc["id"],
            "name": svc.get("name", svc["id"]),
            "tier": svc.get("tier", 0),
            "description": svc.get("description", ""),
            "after": svc.get("after", []) or [],
            "requires": svc.get("requires", []) or [],
            "optional": bool(svc.get("optional", False)),
            "type": svc.get("type", "simple"),
            "exec": svc.get("exec", ""),
            "health_topic": svc.get("health_topic"),
            "health_url": svc.get("health_url"),
            "unit": unit,
            "active_state": active,
            "sub_state": sub,
            "ui_state": ui_state,
            "started_at": state.get("ExecMainStartTimestamp", ""),
            "pid": state.get("ExecMainPID", ""),
            "result": state.get("Result", ""),
        })

    # Group by tier for the renderer.
    tiers: dict[int, list] = {}
    for svc in enriched:
        tiers.setdefault(svc["tier"], []).append(svc)

    tier_labels = {
        0: "Hardware Detection",
        1: "Sensor / I-O Drivers",
        2: "State Estimators",
        3: "Web Bridge",
        4: "Application",
        5: "User Interface",
    }

    target_state = _query_unit(mission.get("target", "er-mission.target"))

    return JsonResponse({
        "status": "ok",
        "mission": {
            "name": mission.get("name", "Earth Rover Mission"),
            "description": mission.get("description", ""),
            "target": mission.get("target", "er-mission.target"),
            "target_state": target_state.get("ActiveState", "unknown"),
        },
        "tiers": [
            {
                "tier": t,
                "label": tier_labels.get(t, f"Tier {t}"),
                "services": tiers[t],
            }
            for t in sorted(tiers.keys())
        ],
    })
