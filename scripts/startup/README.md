# Earth Rover Startup

The rover stack is always on. A lingering `systemd --user` manager starts two
targets at boot and keeps every process in them running:

| Target | Units | Source |
|---|---|---|
| `er-deepgis.target` | `er-deepgis-mavros` (the only MAVROS), `er-deepgis-telemetry`, `er-deepgis-pose`, `er-deepgis-keyframe` | hand-written files in this directory |
| `er-sensors.target` | `er-grasshopper`, `er-velodyne`, `er-laser`, `er-spectrometer`, `er-radio` | generated from `mission.yaml` |

`er-deepgis-keyframe` posts a left-camera keyframe to DeepGIS when the rover has
driven 10 m, turned 30 degrees or 30 s have passed. With each frame it posts a
spectral product: the spectrometer reading integrated across that frame, reduced
by `scripts/spectral_product.py` to a 1 nm spectrum, the clipped spans, band
ratios, the laser range and the size of the patch the spectrometer averaged.
DeepGIS draws it beside the camera frame and gives it to the AI pilot with the
image. The spectrometer's cone and where it points in the image are the
`spectrometer_fov_deg`, `boresight_u` and `boresight_v` parameters of
`scripts/deepgis_keyframe_publisher.py`; `spectrometer_topic:=''` sends frames only.

Each sensor unit runs one stage of `launch/earth_rover_system.launch.py` (every
other stage off, no delay), so the launch defaults stay the single place to tune
a sensor. Before starting, a unit waits for its hardware with `wait_for_hw.sh`
(USB id, serial device, or Velodyne packets on UDP 2368). A sensor that is not
on the rig leaves its unit `activating`; plugging it in starts it. Units restart
forever (`Restart=always`, no start limit).

`er-console.target` (also at boot) runs the web console: `er-rosbridge` (:9090),
`er-web-video` (:8080) and `er-vcs` (the Django dashboard under gunicorn, :8000).
They bind to `VCS_BIND_ADDRESS` from `secrets.env` (default 127.0.0.1; 0.0.0.0 on
the rover, where ufw admits those ports from the cable interface only, not wifi).
One-time setup: `vehicle_control_station/setup_venv.sh` (no sudo) and, as root,
`apt install ros-jazzy-rosbridge-server ros-jazzy-web-video-server` plus the ufw
rules; until then those units wait for their packages.

Everything else is on demand: `er-record` (rosbag2, also the VCS Record button;
refuses with under 20 GB free), `er-rqt`, `er-kiosk`, `er-realsense`, `er-metavision`.

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
