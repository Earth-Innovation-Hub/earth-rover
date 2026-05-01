# Earth Rover Startup

Nothing auto-starts at boot. Launching the rover stack should be an explicit
operator action.

The legacy monolithic trike scripts have been removed. Use the `systemd --user`
mission units for today's stack, or add a new well-sequenced launch script here
when the desired startup order is finalized.

## Current Startup Path

Sequenced ROS launch file:

```bash
ros2 launch deepgis_vehicles earth_rover_system.launch.py
```

This starts MAVROS, then `radio_vio`, then Grasshopper stereo cameras, then the
spectrometer. Use `ros2 launch deepgis_vehicles earth_rover_system.launch.py
--show-args` to see stage toggles and delay arguments.

Install or refresh user units:

```bash
cd /home/jdas/earth-rover/scripts/startup
./install_user_units.sh
```

Start the UI layer only:

```bash
make -C /home/jdas/earth-rover ui-up
xdg-open http://localhost:8000/mission/
```

Start selected services:

```bash
systemctl --user start er-mavros.service
systemctl --user start er-grasshopper.service
systemctl --user start er-rtl-adsb.service er-adsb-state.service
```

Start or stop the mission graph:

```bash
make -C /home/jdas/earth-rover mission-up
make -C /home/jdas/earth-rover mission-status
make -C /home/jdas/earth-rover mission-down
```

## Files Kept

- `install_user_units.sh` / `install_user_units.py`: generate `er-*.service`
  units and `er-mission.target`.
- `firefox_kiosk.sh` / `earth-rover-kiosk.desktop`: kiosk helper.
- `vcs_down.sh` / `vcs_status.sh`: VCS helper scripts.
- `rsync_archive.sh`, `er-rsync-archive.service`, `er-rsync-archive.timer`:
  manual SSD-to-NAS archive support.
- `README.md`: this file.

## Removed Legacy Trike Scripts

The old installer, monolithic startup router, full/minimal stack scripts,
manual status/stop helpers, and legacy desktop/systemd autostart entries have
been removed to avoid maintaining two startup paths.

## Next Step

Move more mission services into `earth_rover_system.launch.py` as needed, or keep
them as individual `er-*` units when separate restart/log control is useful.
