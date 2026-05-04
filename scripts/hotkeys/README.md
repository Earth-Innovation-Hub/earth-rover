# Earth Rover hotkeys

Safe system-wide keyboard shortcuts for the four common operator actions:

| Action | Default key | Script | Backing unit |
|---|---|---|---|
| Start trike system launch | `Ctrl+Alt+Super+T` | `trike-start.sh` | `er-hotkey-trike.service` |
| Stop trike (double-tap) | `Ctrl+Alt+Super+Q` | `trike-stop.sh` | `er-hotkey-trike.service` |
| Start rosbag recording | `Ctrl+Alt+Super+B` | `rosbag-start.sh` | `er-hotkey-bag.service` |
| Stop rosbag recording | `Ctrl+Alt+Super+E` | `rosbag-stop.sh` | `er-hotkey-bag.service` |

## Why this design

Each hotkey just spawns / stops a **transient `systemd --user` unit** via
`systemd-run`. systemd handles process-tree lifecycle, signal escalation, and
log capture — there is no PID file to corrupt and no orphan ROS subprocesses
when something crashes.

Concretely, each "start" hotkey is equivalent to:

```bash
systemd-run --user --unit=er-hotkey-trike.service \
    --collect -p KillSignal=SIGINT -p TimeoutStopSec=20 \
    /bin/bash -lc 'exec make -C ~/earth-rover system-launch'
```

and each "stop" hotkey is equivalent to:

```bash
systemctl --user stop er-hotkey-trike.service
```

`KillSignal=SIGINT` is the key piece — `systemctl --user stop` then sends
`SIGINT` to the whole cgroup, which is what `ros2 launch` and `ros2 bag`
actually want for graceful shutdown (rosbag2 needs `SIGINT` to flush
`metadata.yaml`; `ros2 launch` propagates `SIGINT` to every node it spawned).

## Safety properties

1. **Modifier-heavy keybindings** — three modifiers (`Ctrl+Alt+Super`) plus a
   key. Practically impossible to mash by accident, no collision with GNOME
   defaults.
2. **Idempotent start** — pressing start while the unit is already active is
   a no-op + a "already running" notification. No double-spawn.
3. **Idempotent stop** — pressing stop while the unit is inactive is a no-op
   + a "not running" notification.
4. **Double-tap confirm on the destructive action** — `trike-stop` requires
   two presses inside a 3-second window. First tap arms a confirm timer and
   shows a notification ("press again within 3s"); second tap fires.
   Configurable via `ER_DOUBLE_TAP_WINDOW`.
5. **Visible feedback** — every action emits `notify-send` (urgency
   `normal`/`critical`) and appends to a master audit log at
   `scripts/logs/hotkeys/audit.log`.
6. **Logs survive the press** — unit stdout+stderr go to journald, tailable
   with `journalctl --user -fu er-hotkey-trike` (or `make hotkey-tail-trike`).
7. **No environment surprises** — scripts source `/opt/ros/$ROS_DISTRO` and
   the workspace explicitly, do not depend on the user's shell rc, and pass
   `EARTH_ROVER_HOME` / `ROS_DISTRO` / `ROS2_WS` / `HOME` / `USER` into the
   transient unit.

## Install / uninstall

GNOME (Ubuntu / Pop!\_OS / Fedora Workstation default):

```bash
make hotkeys-install         # writes the four custom-keybindings idempotently
make hotkeys-status          # quick state report
make hotkeys-uninstall       # removes only the "Earth Rover :: ..." entries
```

The installer reads the existing `org.gnome.settings-daemon.plugins.media-keys
custom-keybindings` array, **adds** our four entries (or updates them in place
if the names match), and writes the array back. It never clobbers
unrelated entries.

Override any binding at install time:

```bash
ER_BIND_TRIKE_START='<Ctrl><Alt><Super>F9'  \
ER_BIND_TRIKE_STOP='<Ctrl><Alt><Super>F10'  \
ER_BIND_BAG_START='<Ctrl><Alt><Super>F11'   \
ER_BIND_BAG_STOP='<Ctrl><Alt><Super>F12'    \
make hotkeys-install
```

## Customizing per-press behaviour

The hotkey scripts read a few env vars at invocation time. Set them in your
`~/.profile` (so they propagate to GNOME-spawned processes) to customize:

| Env var | Default | Effect |
|---|---|---|
| `ER_TRIKE_TARGET` | `system-launch` | `make` target run by `trike-start` (e.g. `mavros`, `radio-stack`) |
| `ER_BAG_TARGET` | `record-bag-mavros` | `make` target run by `rosbag-start` (`record-bag` for full, `record-bag-stereo` for cameras only) |
| `ER_DOUBLE_TAP_WINDOW` | `3` | Seconds the `trike-stop` confirm timer stays armed |
| `ER_HOTKEY_LOG` | `~/earth-rover/scripts/logs/hotkeys` | Where the audit log lives |
| `ER_HOTKEY_STATE` | `/tmp/earth-rover-hotkeys-$UID` | Where double-tap timestamps are stored |
| `EARTH_ROVER_HOME` | (auto-detected) | Repo root |
| `ROS_DISTRO` | `jazzy` | ROS distro to source |
| `ROS2_WS` | `~/ros2_ws` | Workspace to overlay |

## Manual operation (no GUI)

The scripts are also useful from a TTY / SSH session:

```bash
~/earth-rover/scripts/hotkeys/trike-start.sh
~/earth-rover/scripts/hotkeys/rosbag-start.sh
~/earth-rover/scripts/hotkeys/status.sh
~/earth-rover/scripts/hotkeys/rosbag-stop.sh
~/earth-rover/scripts/hotkeys/trike-stop.sh    # first tap arms
~/earth-rover/scripts/hotkeys/trike-stop.sh    # second tap (within 3s) fires
```

`notify-send` is a no-op when there is no graphical session, but the audit
log still records every action.

## Non-GNOME desktops

The `.sh` scripts are desktop-agnostic; only `install-gnome.sh` is
GNOME/Cinnamon-specific. Bind them yourself in your environment:

- **KDE Plasma**: System Settings → Shortcuts → Custom Shortcuts → New →
  Global Shortcut → Command/URL. Paste the absolute path to
  `~/earth-rover/scripts/hotkeys/<name>.sh`.
- **Sway / i3**: in your config:
  ```
  bindsym $mod+Shift+t exec ~/earth-rover/scripts/hotkeys/trike-start.sh
  bindsym $mod+Shift+q exec ~/earth-rover/scripts/hotkeys/trike-stop.sh
  bindsym $mod+Shift+b exec ~/earth-rover/scripts/hotkeys/rosbag-start.sh
  bindsym $mod+Shift+e exec ~/earth-rover/scripts/hotkeys/rosbag-stop.sh
  ```
- **sxhkd**: add four `~/.config/sxhkd/sxhkdrc` lines pointing at the
  scripts.

## Troubleshooting

- **The hotkey "does nothing"** — check the audit log
  (`tail ~/earth-rover/scripts/logs/hotkeys/audit.log`); GNOME custom
  shortcuts run with `sh -c "<command>"`, so any error inside the script
  fails silently to the user. The audit log will show whether the script
  ran at all.
- **`notify-send`: cannot connect to the bus** — usually means the script
  is running outside the user's graphical session. Hotkeys run *inside* the
  session so this only bites you when running scripts manually from a TTY,
  in which case it is harmless.
- **`systemd-run`: Failed to add unit dependencies` — leftover failed
  transient unit. The scripts already call `systemctl --user reset-failed`
  before each start, but you can also run it by hand:
  `systemctl --user reset-failed er-hotkey-trike.service`.
- **Trike kept running after `trike-stop`** — check
  `journalctl --user -u er-hotkey-trike --since '5 min ago'`. If
  `ros2 launch` is ignoring `SIGINT`, the unit's `TimeoutStopSec=20` will
  escalate to `SIGKILL` after 20 s; you can tune via env var on install.
