# Earth Rover Startup

The rover stack is always on. A lingering `systemd --user` manager starts two
targets at boot and keeps every process in them running:

| Target | Units | Source |
|---|---|---|
| `er-deepgis.target` | `er-deepgis-mavros` (the only MAVROS), `er-deepgis-telemetry`, `er-deepgis-pose` | hand-written files in this directory |
| `er-sensors.target` | `er-grasshopper`, `er-velodyne`, `er-laser`, `er-spectrometer`, `er-radio` | generated from `mission.yaml` |

Each sensor unit runs one stage of `launch/earth_rover_system.launch.py` (every
other stage off, no delay), so the launch defaults stay the single place to tune
a sensor. Before starting, a unit waits for its hardware with `wait_for_hw.sh`
(USB id, serial device, or Velodyne packets on UDP 2368). A sensor that is not
on the rig leaves its unit `activating`; plugging it in starts it. Units restart
forever (`Restart=always`, no start limit).

Everything else is on demand: `er-record` (rosbag2), `er-rqt`, `er-kiosk`,
`er-realsense`, `er-metavision`, and `er-console.target` (rosbridge, web video,
the VCS Django app; not enabled until the VCS runs on Jazzy).

## Install or refresh units

```bash
~/earth-rover/scripts/startup/install_user_units.sh   # generate, prune stale, enable boot targets
make -C ~/earth-rover mission-up                       # start now instead of rebooting
make -C ~/earth-rover mission-status
```

Edit `mission.yaml`, never the generated `er-*.service` files. Generated units
that the manifest no longer lists are stopped and deleted on the next install.

## Day to day

```bash
journalctl --user -u er-grasshopper -f              # one sensor's log
systemctl --user restart er-velodyne                 # restart one sensor
systemctl --user stop er-sensors.target              # all sensors off (DeepGIS link stays up)
systemctl --user start er-record                     # record a bag; stop it the same way
```

`ros2 launch deepgis_vehicles earth_rover_system.launch.py` still works for bench
runs: it skips MAVROS and any stage whose unit is up or waiting
(`defer_to_units:=false` to force).

Site-specific values (e.g. `VCS_BIND_ADDRESS` for the console) go in
`~/.config/earth-rover/secrets.env` (mode 600, not in git).

## Files

- `mission.yaml`: the manifest (targets, services, tiers for the VCS chart).
- `install_user_units.sh` / `install_user_units.py`: generator and installer.
- `wait_for_hw.sh`: hardware wait used by the sensor units.
- `er-deepgis*.service`, `er-deepgis.target`: the DeepGIS link.
- `firefox_kiosk.sh` / `earth-rover-kiosk.desktop`: kiosk helper.
- `vcs_up.sh` / `vcs_down.sh` / `vcs_status.sh`, `vcs.tmuxp.yaml`: legacy VCS helpers, superseded by `er-console.target`.
- `rsync_archive.sh`, `er-rsync-archive.service`, `er-rsync-archive.timer`: SSD-to-NAS archive.
