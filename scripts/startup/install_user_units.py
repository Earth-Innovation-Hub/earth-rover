#!/usr/bin/env python3
"""Generate systemd --user units from mission.yaml.

Each service in mission.yaml becomes one ``er-<id>.service`` file.  An
aggregating ``er-mission.target`` is generated that ``Wants=`` every service
so a single ``systemctl --user start er-mission.target`` brings the whole
rover stack up in tier order.

Edge encoding:
    after:    -> After=
    requires: -> Requires=  (hard dependency: failure of dep stops this unit)

Mission topology is therefore expressed declaratively in mission.yaml; this
script is a pure translator.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import yaml


SERVICE_TEMPLATE = """\
# Auto-generated from mission.yaml -- DO NOT EDIT BY HAND.
# Edit scripts/startup/mission.yaml then re-run install_user_units.sh.
[Unit]
Description={description}
PartOf=er-mission.target
{after_line}{wants_line}{requires_line}

[Service]
Type={unit_type}
{env_lines}\
ExecStart=/bin/bash -lc {exec_quoted}
{restart_line}\
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=er-mission.target
"""


TARGET_TEMPLATE = """\
# Auto-generated from mission.yaml -- DO NOT EDIT BY HAND.
[Unit]
Description={description}
Wants={wants}
After={after}

[Install]
WantedBy=default.target
"""


def shell_quote(s: str) -> str:
    """Single-quote a string for safe inclusion in an ExecStart= bash -lc arg."""
    return "'" + s.replace("'", "'\\''") + "'"


def build_env_block(defaults: dict, svc: dict) -> str:
    """Return Environment= lines covering the global defaults + per-service env.

    Defaults are exposed as upper-case env vars (e.g. ``ros_distro`` ->
    ``ROS_DISTRO``); per-service ``env`` values are added verbatim.
    """
    lines = []
    for key, val in defaults.items():
        env_key = key.upper()
        lines.append(f'Environment="{env_key}={val}"')
    for key, val in (svc.get("env") or {}).items():
        lines.append(f'Environment="{key}={val}"')
    return "\n".join(lines) + ("\n" if lines else "")


def emit_service(svc: dict, defaults: dict) -> tuple[str, str]:
    """Return (filename, contents) for one service entry."""

    sid: str = svc["id"]
    unit_name = f"er-{sid}.service"

    description = svc.get("description") or svc.get("name") or sid

    # Build After= / Requires= referencing the matching er-<dep>.service.
    after_deps = [f"er-{d}.service" for d in svc.get("after", []) or []]
    requires_deps = [f"er-{d}.service" for d in svc.get("requires", []) or []]
    after_line = f"After={' '.join(after_deps)}\n" if after_deps else ""
    wants_line = ""  # use Requires= for hard deps; Wants= adds noise here
    requires_line = (
        f"Requires={' '.join(requires_deps)}\n" if requires_deps else ""
    )

    unit_type = svc.get("type", "simple")

    # ROS env must be sourced inside the ExecStart shell so child processes
    # have $ROS_DISTRO etc. picked up from the systemd Environment=.
    ros_setup = (
        "source /opt/ros/${ROS_DISTRO}/setup.bash; "
        "[ -f ${ROS2_WS}/install/setup.bash ] && "
        "source ${ROS2_WS}/install/setup.bash;"
    )
    full_cmd = f"{ros_setup} {svc['exec']}"

    # oneshot type: no Restart=, fast precondition checks
    if unit_type == "oneshot":
        restart_line = "RemainAfterExit=yes\n"
    else:
        restart_line = "Restart=on-failure\nRestartSec=5\n"

    contents = SERVICE_TEMPLATE.format(
        description=description.strip(),
        after_line=after_line,
        wants_line=wants_line,
        requires_line=requires_line,
        unit_type=unit_type,
        env_lines=build_env_block(defaults, svc),
        exec_quoted=shell_quote(full_cmd),
        restart_line=restart_line,
    )
    return unit_name, contents


def emit_target(mission: dict, services: list[dict]) -> tuple[str, str]:
    target_name = mission.get("target", "er-mission.target")

    unit_names = [f"er-{s['id']}.service" for s in services]
    contents = TARGET_TEMPLATE.format(
        description=mission.get("name", "Earth Rover Mission"),
        wants=" ".join(unit_names),
        after=" ".join(unit_names),
    )
    return target_name, contents


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print unit contents without writing.",
    )
    args = parser.parse_args()

    with args.mission.open() as fh:
        manifest = yaml.safe_load(fh)

    mission = manifest.get("mission", {})
    defaults = manifest.get("defaults", {}) or {}
    services = manifest.get("services", []) or []

    if not services:
        print("[install_user_units] no services in mission.yaml", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    written = []
    for svc in services:
        name, body = emit_service(svc, defaults)
        path = args.output_dir / name
        if args.dry_run:
            print(f"--- {path} ---")
            print(body)
        else:
            path.write_text(body)
        written.append(name)

    target_name, target_body = emit_target(mission, services)
    target_path = args.output_dir / target_name
    if args.dry_run:
        print(f"--- {target_path} ---")
        print(target_body)
    else:
        target_path.write_text(target_body)
    written.append(target_name)

    if not args.dry_run:
        print(f"[install_user_units] wrote {len(written)} units to {args.output_dir}")
        for n in written:
            print(f"   {n}")
        # Best-effort daemon-reload so systemd picks up new files immediately.
        rc = os.system("systemctl --user daemon-reload")
        if rc != 0:
            print(
                "[install_user_units] WARN: 'systemctl --user daemon-reload' "
                f"exited {rc}. You may need to run it manually.",
                file=sys.stderr,
            )

    print(
        "\nNext:\n"
        "  systemctl --user enable --now er-mission.target\n"
        "  systemctl --user list-units 'er-*'\n"
        "  journalctl --user -u er-vcs -f\n"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
