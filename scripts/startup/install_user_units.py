#!/usr/bin/env python3
"""Generate systemd --user units from mission.yaml.

Each non-external service in mission.yaml becomes one ``er-<id>.service``;
each entry under ``targets:`` becomes a target that ``Wants=`` its members.

    target: <key>     -> member of that target, Restart=always, never gives up
    target: graphical -> bound to graphical-session.target, started by hand
    target: none      -> on demand, started by hand or from the VCS
    stage: <name>     -> one stage of earth_rover_system.launch.py, no delay
    wait_for: [...]   -> ExecStartPre=wait_for_hw.sh (blocks until present)
    after/requires    -> After= / Requires= on the matching er-<dep>.service

Generated units carry a header line; units with that header that are no longer
produced by the manifest are stopped and removed, so the unit directory never
drifts from mission.yaml. Hand-written sidecars (no header) are left alone.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml


HEADER = "# Auto-generated from mission.yaml -- DO NOT EDIT BY HAND."

# stage -> (enable flag, delay argument) in launch/earth_rover_system.launch.py
STAGES = {
    "radio_vio": ("enable_radio_vio", "radio_delay_sec"),
    "rtl_sdr": ("enable_rtl_sdr", "rtl_sdr_delay_sec"),
    "laser": ("enable_laser", "laser_delay_sec"),
    "grasshopper": ("enable_grasshopper", "camera_delay_sec"),
    "realsense": ("enable_realsense", "camera_delay_sec"),
    "velodyne": ("enable_velodyne", "velodyne_delay_sec"),
    "spectrometer": ("enable_spectrometer", "spectrometer_delay_sec"),
    "rqt": ("rqt_gui", "rqt_delay_sec"),
}

SERVICE_TEMPLATE = """\
{header}
# Edit scripts/startup/mission.yaml then re-run install_user_units.sh.
[Unit]
Description={description}
{unit_lines}
[Service]
Type=simple
EnvironmentFile=-%h/.config/earth-rover/secrets.env
{env_lines}{pre_lines}ExecStart=/bin/bash -lc {exec_quoted}
{restart_lines}KillSignal=SIGINT
TimeoutStopSec=20
StandardOutput=journal
StandardError=journal
{install}"""

TARGET_TEMPLATE = """\
{header}
[Unit]
Description={description}
Wants={wants}
# Without this a target is ordered After= everything it Wants=, so one sensor
# waiting for absent hardware would hold this target (and default.target) forever.
DefaultDependencies=no
Conflicts=shutdown.target
Before=shutdown.target

[Install]
WantedBy=default.target
"""


def shell_quote(s: str) -> str:
    """Single-quote a string for safe inclusion in an ExecStart= bash -lc arg."""
    return "'" + s.replace("'", "'\\''") + "'"


def build_env_block(defaults: dict, svc: dict) -> str:
    """Defaults become upper-case env vars (ros_distro -> ROS_DISTRO); per-service env is verbatim."""
    lines = [f'Environment="{k.upper()}={v}"' for k, v in defaults.items()]
    lines += [f'Environment="{k}={v}"' for k, v in (svc.get("env") or {}).items()]
    return "\n".join(lines) + ("\n" if lines else "")


def stage_command(svc: dict) -> str:
    stage = svc["stage"]
    if stage not in STAGES:
        raise SystemExit(f"[install_user_units] {svc['id']}: unknown stage {stage!r} (known: {sorted(STAGES)})")
    args = {"defer_to_units": "false", "enable_mavros": "false"}
    for flag, _ in STAGES.values():
        args[flag] = "false"
    flag, delay = STAGES[stage]
    args[flag] = "true"
    args[delay] = "0.0"
    args.update({k: str(v) for k, v in (svc.get("args") or {}).items()})
    argv = " ".join(f"{k}:={v}" for k, v in args.items())
    return f"exec ros2 launch deepgis_vehicles earth_rover_system.launch.py {argv}"


def emit_service(svc: dict, defaults: dict, targets: dict) -> tuple[str, str]:
    sid: str = svc["id"]
    target = svc.get("target", "none")
    after = [f"er-{d}.service" for d in svc.get("after", []) or []]
    requires = [f"er-{d}.service" for d in svc.get("requires", []) or []]

    unit_lines = []
    install = ""
    if target == "graphical":
        after.append("graphical-session.target")
        unit_lines.append("PartOf=graphical-session.target")
        install = "\n[Install]\nWantedBy=graphical-session.target\n"
    elif target != "none":
        if target not in targets:
            raise SystemExit(f"[install_user_units] {sid}: unknown target {target!r}")
        unit = targets[target]["unit"]
        unit_lines.append(f"PartOf={unit}")
        # Keep retrying forever: a sensor that is unplugged and replugged comes back.
        unit_lines.append("StartLimitIntervalSec=0")
        install = f"\n[Install]\nWantedBy={unit}\n"
    if after:
        unit_lines.append(f"After={' '.join(after)}")
    if requires:
        unit_lines.append(f"Requires={' '.join(requires)}")

    pre_lines = ""
    if svc.get("wait_for"):
        specs = " ".join(svc["wait_for"])
        pre_lines = (
            f"ExecStartPre=/bin/bash ${{EARTH_ROVER_HOME}}/scripts/startup/wait_for_hw.sh {specs}\n"
            "TimeoutStartSec=infinity\n"
        )

    if target in ("none", "graphical"):
        restart_lines = "Restart=on-failure\nRestartSec=10\n"
    else:
        restart_lines = "Restart=always\nRestartSec=10\n"

    command = stage_command(svc) if svc.get("stage") else svc["exec"]
    ros_setup = (
        "source /opt/ros/${ROS_DISTRO}/setup.bash; "
        "[ -f ${ROS2_WS}/install/setup.bash ] && source ${ROS2_WS}/install/setup.bash;"
    )
    description = (svc.get("description") or svc.get("name") or sid).strip()

    body = SERVICE_TEMPLATE.format(
        header=HEADER,
        description=description,
        unit_lines="\n".join(unit_lines) + "\n",
        env_lines=build_env_block(defaults, svc),
        pre_lines=pre_lines,
        exec_quoted=shell_quote(f"{ros_setup} {command}"),
        restart_lines=restart_lines,
        install=install,
    )
    return f"er-{sid}.service", body


def emit_targets(targets: dict, services: list[dict]) -> list[tuple[str, str]]:
    out = []
    for key, spec in targets.items():
        members = [f"er-{s['id']}.service" for s in services
                   if not s.get("external") and s.get("target") == key]
        out.append((spec["unit"], TARGET_TEMPLATE.format(
            header=HEADER,
            description=spec.get("description", spec["unit"]),
            wants=" ".join(members),
        )))
    return out


def prune(output_dir: Path, keep: set[str]) -> list[str]:
    """Stop and remove previously generated er-* units the manifest no longer produces."""
    stale = []
    for path in sorted(output_dir.glob("er-*")):
        if path.name in keep or not path.is_file():
            continue
        try:
            first = path.read_text().splitlines()[0]
        except (OSError, IndexError):
            continue
        if first.strip() == HEADER:
            stale.append(path)
    for path in stale:
        subprocess.run(["systemctl", "--user", "disable", "--now", path.name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        path.unlink()
        wants = output_dir / f"{path.name}.wants"
        if wants.is_dir():
            for link in wants.iterdir():
                link.unlink()
            wants.rmdir()
    return [p.name for p in stale]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true", help="Print unit contents without writing.")
    parser.add_argument("--print-boot-targets", action="store_true",
                        help="Print the target units marked boot: true and exit.")
    args = parser.parse_args()

    with args.mission.open() as fh:
        manifest = yaml.safe_load(fh)

    defaults = manifest.get("defaults", {}) or {}
    targets = manifest.get("targets", {}) or {}
    services = manifest.get("services", []) or []

    if args.print_boot_targets:
        print(" ".join(t["unit"] for t in targets.values() if t.get("boot")))
        return 0
    if not services:
        print("[install_user_units] no services in mission.yaml", file=sys.stderr)
        return 1

    units = [emit_service(s, defaults, targets) for s in services if not s.get("external")]
    units += emit_targets(targets, services)

    if args.dry_run:
        for name, body in units:
            print(f"--- {args.output_dir / name} ---")
            print(body)
        return 0

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, body in units:
        (args.output_dir / name).write_text(body)
    removed = prune(args.output_dir, {name for name, _ in units})

    print(f"[install_user_units] wrote {len(units)} units to {args.output_dir}")
    for name, _ in units:
        print(f"   {name}")
    for name in removed:
        print(f"   removed stale {name}")
    rc = os.system("systemctl --user daemon-reload")
    if rc != 0:
        print(f"[install_user_units] WARN: 'systemctl --user daemon-reload' exited {rc}.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
