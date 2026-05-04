# earth-rover
Affordable and Sustainable Mobility Autonomy for  4D Environmental Monitoring

Jnaneshwar Das, ASU School of Earth and Space Exploration 


## Abstract

For continuous ecological monitoring of urban or suburban environments, it is necessary to have affordable and versatile mobility autonomy. In this setting, we may wish to carry out 3D metric-semantic-topological mapping of trees, infrastructure, rocks, or other features of interest periodically, in intervals of hours to weeks. Autonomous Ground Vehicles (AGVs) from companies such as ClearPath Robotics cost about USD 25000, and need to be carried around for deployment, posing both cost and logistical challenges. 
![image](https://github.com/user-attachments/assets/07d53db6-031a-4a68-9338-75e33c5428a5)


This prototype is a 26" Schwinn adult tricycle frame, augmented with a front electric motor for traction, a steering system, rear aerodynamic solar panel assembly,  a rear mast for avionics, sensing, and compute payloads. The vehicle's large (26") wheels and low weight (65 kg), enables stability and long range (up to 50 km a day with solar), and is capable of GPS-enabled vision based autonomous navigation on paved or unpaved paths. The vehicle can execute repeatable paths that are either learned by the system from experimental drives by a rider, or through specified or optimized mission plans. At a base cost of about USD 3000, our system is affordable, and can be assembled from either COTS components, or from custom hardware. Additionally, our system can be operated manually like a standard electric tricycle, providing additional modalities for expert data collection and imitation learning. 

## System Description

- **Mass**: 65 kg with instrumentation and solar panels (no human)
- **Range**: Tested 20 km (manual operation, without solar); estimated 30 km with solar over whole day manual operation; estimated 60+ km for autonomous operation with solar
- **Power stores**: Lithium Iron-Phosphate (LiFePO4) 25Ah, 57.6 V (traction); LiFePO4 100Ah, 14.4 V (avionics, compute, sensing, autonomous traction and steering)
- **Power conversion**: 1000W inverter for 14.4V DC to 110V AC; 30A MPPT charge controller for 200W solar charging; 2 x 300W 110V AC to 12V DC power converters for computing and autonomous control; 110V to 58V DC for traction battery charging; 110V to 14.4V DC for avionics/compute battery charging
- **Traction**: Front 1000W (1.3 horsepower) direct-drive 3-phase electric motor; 1000W controller for manual ride controlled by throttle; 300W motor controller for autonomous drives, controlled by Pixhawk flight controller over PWM channel
- **Energy sequestration**: 2 x 100W ultralight panels, mounted on rear boom for aerodynamic shape; regenerative braking
- **Compute**: 11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz, 32GB RAM, 512GB internal SSD, 2TB external SSD; Google Coral Tensor Processing Unit (TPU)
- **Avionics**: Pixhawk 2.1 Flight Controller, PX4 open source autopilot software stack; Here+ GPS with RTK option
- **Sensor suite**: PointGrey Grasshopper3 with narrow angle (left) and wide angle (right) lenses; MicaSense Altum 6 band multi-spectral camera triggered by flight controller, 100m LiDARLite LASER ranger; OceanOptics FLAME UV-VIS-NIR spectrometer; Intel RealSense T265 tracking camera
- **Communication**: Ubiquiti networks 2.4GHz AirMax PicoStation access point; WiFi hotspot on onboard compute; 915 MHz telemetry radio to Pixhawk; 2.4GHz DSMX RC transmitter to Pixhawk. Optional SDR (HydraSDR, RTL-SDR) for ADS-B and spectrum monitoring — install helpers live under [`packages/radio_vio/scripts/`](packages/radio_vio/scripts/); launch `sdr.launch.py`, `adsb.launch.py`, `rtl_sdr.launch.py`, and `rtl_adsb.launch.py` via **`ros2 launch radio_vio …`**.

![image](https://github.com/user-attachments/assets/836d27e1-4022-4540-929a-9dad2fe70606)

## Mobility system

The vehicle has a 1 kW front brushless 3 phase electric motor that operates with a 48V Lithium Iron Phosphate (LiFePO4) battery. The trike can be steered manually or through a slip-drive clutch actuator system. 
The front traction motor is operated at a lower power of 300W with a different controller, for safety. This controller is commanded with a potentiometer-servo combination, providing isolation and an additional level of control since the potentiometer can be rotated like a throttle, by a test rider. 

When controlled manually, a throttle with a hall-effect sensor enables commandeering of the vehicle up to a speed of 12 m/s. Electronic braking serves as an additional source of charging. 

When steered by the Pixhawk flight controller, QGroundControl ground control station (GCS) software is used, and the traction system is switched to a 300W controller for safety, providing lower power and speeds. 

Payload mast: A load bearing mount-point at a height of 120cm from ground, is built using a spring assembly consisting of low-cost COTS carbon fiber and aluminum arrow bodies, and the trike's metal rear basket, that yields to motions in both body frame x, y, and z axes. This assembly helps decouple the mounted imaging and compute system from bumps and jerks that could interfere with data collection, or in the worst case scenario, can damage aspects of the imaging suite assembly. With the mounting assembly, the trike is able to collect data while traversing at speeds up to 12 m/s. 


## Autonomy

The electric tricycle's avionics package consists of a Pixhawk 2.1 flight controller running a rover airframe on the PX4 autopilot software stack. The vehicle is capable of GPS and IMU based waypoint missions, as a base feature, with additional computer vision capabilities through simultaneous localization and mapping 

Figure 2: QGroundControl GCS used for mission planning and situational awareness during a field trial at ASU campus (Dec 2023). 
![image](https://github.com/user-attachments/assets/08483e81-e819-4b52-bc0f-bbcd1fadd7ce)


(SLAM), ORBSLAM3, ROS, PX4, Gazebo, and PX4 SITL digital twin. 

The system's onboard computer runs ROS2, with ros nodes for all the cameras, spectrometer, lidar ranger, and MAVROS package for communicating with the Pixhawk for telemetry, and commanding the vehicle. Instrument drivers for the USB laser ranger and Ocean Optics spectrometer (including live plotting) are kept in [`packages/`](#ros-2-instrument-packages) in this repo. 

QGC allows GPS mission planning, with all operations possible without internet connectivity, on cached maps

## Mapping systems

![image](https://github.com/user-attachments/assets/163e7deb-a30d-4436-906e-73c8537e1836)

Figure 3: Realtime mapping demo, while the vehicle is manually operated by a human. The system can leverage the 3D maps and localized vehicle path to plan unmanned operations to remap routes, for instance for biomass change estimation 

Applications and Research Areas  	
Environmental Monitoring in urban and suburban settings, including biomass mapping and heat/shade modeling. 	
With its onboard mapping suite consisting of global shutter multi-focal stereo cameras, a multi-spectral camera, and a UV-VIS-NIR spectrometer, EarthRover is able to collect rich data for 3D environmental analysis. The dense datasets collected by EarthRover (> 1GBps) presents avenues for further research for optimal data fusion for multi-scale, multi-modal hyperspectral datasets, for 4D environmental change monitoring. 
Geological and Ecological Mapping for Digital Twins




Point cloud uncertainty (blue=low)

Figure 4: Mapping of a rocky feature set by the vehicle, by orbiting and collecting imagery with its PointGrey Grasshopper3 camera with narrow angle lens.  

## ADS-B and Visual-Odometry State Estimation

Earth Rover includes an SDR-based ADS-B receiver pipeline (RTL-SDR or HydraSDR fronting `dump1090`) and a coupled state estimator that uses the constellation of decoded aircraft positions to estimate the trike's own receiver location and clock bias. The estimator combines a robust active-set least-squares solver with a Kalman-smoothed receiver state, and can be fused with visual-odometry landmark observations to provide a redundant pose estimate in GNSS-denied or degraded conditions. Real-time visualization is provided by RViz, a 2D top-down plot, a glide-profile plot, and a spherical-fisheye landmark VO plot. Launch entry points live in the **`radio_vio`** package: `sdr.launch.py`, `adsb.launch.py`, `adsb_aircraft_state_vectors{,_rviz}.launch.py`, `adsb_state_vectors_plot_2d.launch.py`, `adsb_state_vectors_plot_glide.launch.py`, `landmark_vo_plot_2d.launch.py`, `landmark_vo_plot_fisheye.launch.py`, `rtl_sdr.launch.py`, `rtl_adsb.launch.py`. Vehicle-side launches remain under **`deepgis_vehicles`** (`launch/` at repo root). Internal design notes are kept under `docs/` (untracked — request a copy if you need them).

## Natural Language Interaction and lifelong learning
We anticipate interaction with a human user through command line interface, verbal interaction, and gestures. The command set may include navigational instructions such as "Follow me" which is integrated with the trike autonomy stack for visual tracking of a human leader. For mission planning however, the bulk of the commanding architecture may be natural language with generative AI in the backbone for tokenizing the instructions and generating optimal exploration plans.  

Example natural language prompts: 
Map all trees 
Count fruits along the path I take
Map as much as you can in the next 15 mins, stay within 50m of me. 
Collect imagery at this scene to improve existing maps 
Estimate plant biomass of this patch
What is the rock trait distribution along this pavement? 
Orbit those rocks I am pointing at, with a 10m radius, and show me the 3D model of the rocks.  
 
## Videos

https://drive.google.com/file/d/1M8JIJIpk5DaTXnk8shY55RajGW9rd4tN/view?usp=sharing



https://drive.google.com/file/d/1zt0DhuATs35bWYduTn2sGGhrnV1hIs_N/view?usp=sharing

https://drive.google.com/file/d/1mKV-D2FhJC4ZnpSakJQ_sJLD6O1SGgua/view?usp=sharing



https://www.youtube.com/watch?v=l2MmlcPx6kE



https://www.youtube.com/watch?v=NZj4yiCzRHI



Monitoring normalized difference vegetation index (NDVI) with side-mounted imaging suite with OceanOptics VIS-NIR spectrometer. Imagery from narrow angle and wide angle Grasshopper3 cameras also shown. 
https://youtu.be/PZGcjdSuags?si=71zC8FyeZaOeyud6

Data capture with the vehicle with a Prophesee metavision event camera (Alphacore) in the imaging suite, providing high dynamic range. 
https://www.youtube.com/watch?v=2V3Mc3UAJss

---

## Repository Structure

| Path | Description |
|------|-------------|
| **`src/`** | C++ source for the `deepgis_vehicles` ROS 2 vehicle interface node. |
| **`include/`** | Public headers for the `deepgis_vehicles` C++ node. |
| **`launch/`** | ROS 2 launch files installed by `deepgis_vehicles`, including the current sequenced system launch. |
| **`packages/radio_vio/`** | SDR / ADS-B / radio-VIO stack: RTL-SDR/HydraSDR helpers, aircraft state-vector estimation, plot publishers, and landmark VO visualizations. |
| **`packages/rtlsdr_ros2/`** | RTL-SDR ROS 2 reader, spectrum messages, and support nodes used by `radio_vio`. |
| **`packages/laser_ranger/`** | USB serial laser ranger ROS 2 package. |
| **`packages/spectrometery_ros2/`** | Ocean Optics / SeaBreeze spectrometer publisher and plot-image node. |
| **`scripts/`** | Vehicle-side helper scripts; operator **keyboard hotkeys** under [`scripts/hotkeys/`](scripts/hotkeys/), post-mission **rosbag analysis** scripts (`analyze_spectrometer_rosbag.py`, `cluster_spectrometer_rosbag.py`, `geoplot_rosbag.py` — see [Rosbag analysis scripts](#rosbag-analysis-scripts)). SDR / ADS-B / landmark-VO executables live under `packages/radio_vio/scripts/`. |
| **`scripts/startup/`** | Current `systemd --user` unit generator, kiosk helper, VCS stop/status helpers, and manual archive service files. Legacy trike startup scripts have been removed. |
| **`vehicle_control_station/`** | Django web app for real-time camera feeds, GPS/map, LiDAR, spectrometer, avionics gauges, and ROS recording. See [`vehicle_control_station/README.md`](vehicle_control_station/README.md). |
| **`config/`** | YAML and RViz configurations for MAVROS, vehicle sensors, telemetry, and the `earth_rover.perspective` rqt layout used by the system launch. |
| **`external/deepgis_vision/`** | Git submodule for Grasshopper stereo vision / AI perception launch integration. |
| **`external/metavision_driver/`** | Git submodule pinning a `darknight-007` fork of the Prophesee Metavision ROS 2 driver used for event-camera capture. |
| **`external/patches/`** | Local-machine patches applied on top of pinned upstream SHAs (Metavision driver, ORB-SLAM3 ROS 2 monocular). See [`external/patches/README.md`](external/patches/README.md). |
| **`kernelcal/`** | Git submodule for kernel-dynamics / Maximum-Caliber analysis experiments. |
| **`earth-rover.repos`** | `vcstool` manifest pinning every non-submodule upstream package (PX4, MAVROS bridge, FLIR/RealSense/event-camera drivers, ORB-SLAM3 ROS 2) at the SHAs running on the rover. |
| **`scripts/setup_workspace.sh`** | One-shot workspace bootstrapper that initializes submodules, symlinks this repo into `~/ros2_ws/src/`, runs `vcs import` against `earth-rover.repos`, applies `external/patches/`, and `colcon build`s. |
| **`Makefile`** | Developer/operator shortcuts. Run `make help` for the current target list. |
| **`docs/`** | Untracked (gitignored) local design notes — ADS-B receiver-position estimator, landmark VO, state estimation. Screenshots embedded from the bring-up section are local-only; ask if you need a copy. |

## Current ROS 2 Stack

The working runtime is ROS 2 **Jazzy**. The repo root builds the `deepgis_vehicles`
package, while the instrument and radio packages live under `packages/`.
Upstream non-submodule dependencies are pinned in
[`earth-rover.repos`](earth-rover.repos) and brought in via `vcstool`.

The reproducible end-to-end bring-up is `scripts/setup_workspace.sh`. It clones
the `external/` submodules, symlinks this repo into `~/ros2_ws/src/`, runs
`vcs import < earth-rover.repos`, applies `external/patches/`, and
`colcon build`s the result:

```bash
~/earth-rover/scripts/setup_workspace.sh
# Useful flags:
#   --no-build      stop after import + patches
#   --no-patches    skip applying external/patches/*.patch
#   --ws ~/other_ws use a different workspace root
```

For incremental rebuilds of just the first-party packages from this checkout:

```bash
cd ~/earth-rover
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select deepgis_vehicles radio_vio rtlsdr_ros2 laser_ranger spectrometery_ros2
source install/setup.bash
```

If you are building from `~/ros2_ws`, include the package paths explicitly or
symlink this repository's packages into that workspace.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select radio_vio rtlsdr_ros2 \
  --paths ~/earth-rover/packages/radio_vio ~/earth-rover/packages/rtlsdr_ros2
source install/setup.bash
```

The convenience targets in `Makefile` assume `ROS_DISTRO=jazzy`,
`ROS2_WS=~/ros2_ws`, and the current checkout as `EARTH_ROVER_HOME`. Override
those variables on the command line when needed:

```bash
make info
make build-radio
make radio-stack PRESET=full

# Camera shortcuts (run each in its own terminal)
make gh-cam-left
make gh-cam-right

# Rosbag recording
make record-bag                          # default capture
make record-bag-mavros                   # MAVROS + tf + diagnostics + adsb
make record-bag-stereo                   # stereo only (compressed by default)
make record-bag RECORD_ARGS="record_stereo_raw:=true compression_mode:=file"
```

## Keyboard shortcuts (GNOME)

System-wide shortcuts start and stop the trike stack and rosbag recording via transient `systemd --user` units (graceful `SIGINT` for `ros2 launch` / `ros2 bag`). Install once on a GNOME-based desktop (Ubuntu, Pop!\_OS, Fedora Workstation):

```bash
make hotkeys-install    # register bindings
make hotkeys-status     # verify state
make hotkeys-uninstall  # remove only Earth Rover entries
```

| Action | Default shortcut | Notes |
|--------|------------------|--------|
| Start trike (`make system-launch` by default) | **Ctrl+Alt+Super+T** | Idempotent if already running. |
| Stop trike | **Ctrl+Alt+Super+Q** | **Double-tap within 3 s** to confirm shutdown. |
| Start rosbag (`make record-bag-mavros` by default) | **Ctrl+Alt+Super+B** | Idempotent if already recording. |
| Stop rosbag | **Ctrl+Alt+Super+E** | Flushes bag metadata on `SIGINT`. |

**Tail hotkey unit logs:** `make hotkey-tail-trike` or `make hotkey-tail-bag`.

**Customize** bindings at install time (example): set `ER_BIND_TRIKE_START`, `ER_BIND_TRIKE_STOP`, `ER_BIND_BAG_START`, `ER_BIND_BAG_STOP` before `make hotkeys-install`. **Change what runs** on start: `ER_TRIKE_TARGET` (default `system-launch`) and `ER_BAG_TARGET` (default `record-bag-mavros`). Full behavior, safety notes, non-GNOME desktops audit logging, and manual script paths are documented in [`scripts/hotkeys/README.md`](scripts/hotkeys/README.md).

## Primary Bring-Up

The current all-in-one ROS launch entry point is:

```bash
ros2 launch deepgis_vehicles earth_rover_system.launch.py
```

It starts the rover in this order:

1. **MAVROS / PX4 bridge** (`fcu_url` defaults to the FTDI serial-by-id link).
2. **`radio_vio`** ADS-B + landmark VO stack (`radio_preset` selects the sub-stack).
3. **RTL-SDR standalone** spectrum analyzer (`enable_rtl_sdr` — **OFF by default**;
   do not run alongside `radio_vio` on a single dongle).
4. **Laser ranger** (USB serial, default device pinned to `/dev/serial/by-id/...`
   so unplug/replug doesn't require a launch edit).
5. **Grasshopper stereo cameras** — left and right launched as independent
   single-camera nodes (default 15 Hz each, with per-side overrides).
6. **Spectrometer** publisher and plot image.
7. **rqt** on the host display, loaded with the
   `config/earth_rover.perspective` ([`config/earth_rover.perspective`](config/earth_rover.perspective)).

Each stage has its own `enable_*` toggle and `*_delay_sec` so field tuning does
not require editing the launch file. Use `--show-args` to inspect every option:

```bash
ros2 launch deepgis_vehicles earth_rover_system.launch.py --show-args
```

Common examples:

```bash
# Field default using Makefile Pixhawk and GCS variables
make system-launch

# Headless run (no rqt GUI) without cameras
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  rqt_gui:=false enable_grasshopper:=false

# Decoder only, no plots or landmark VO
ros2 launch deepgis_vehicles earth_rover_system.launch.py radio_preset:=decoder

# Standalone RTL-SDR on a second dongle (index 1) instead of radio_vio
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  enable_radio_vio:=false \
  enable_rtl_sdr:=true rtl_sdr_device_index:=1

# Override the laser by-id path (e.g. swapped FT230X unit)
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  laser_serial_device:=/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_<SERIAL>-if00-port0

# Cameras at a different rate (or asymmetric rates per side)
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  grasshopper_frame_rate:=10.0
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  grasshopper_left_frame_rate:=20.0 grasshopper_right_frame_rate:=5.0

# Tune camera serials and per-camera YAML/calibration
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  grasshopper_left_serial:=22312692 \
  grasshopper_right_serial:=22312674 \
  grasshopper_left_parameter_file:=grasshopper_left.yaml \
  grasshopper_right_parameter_file:=grasshopper_right.yaml \
  grasshopper_left_camera_info_url:=/path/to/left.yaml \
  grasshopper_right_camera_info_url:=/path/to/right.yaml
```

The Grasshopper cameras launch from the FLIR driver workspace as two
single-camera launch files:
`spinnaker_camera_driver/launch/grasshopper_left.launch.py` and
`spinnaker_camera_driver/launch/grasshopper_right.launch.py`. Each exposes a
`frame_rate` arg that forces `frame_rate_auto:=Off` and
`frame_rate_enable:=True` so the requested rate actually takes effect on the
sensor.

### rqt GUI

Stage 7 launches `rqt` on the operator's `$DISPLAY` and loads the canonical
`earth_rover` perspective. The default layout shows the left and right
Grasshopper image streams, the spectrometer plot image, and the
`/laser_distance` time series. (A reference screenshot lives under the
gitignored `docs/` tree on operator workstations; ask if you need a copy.)

Skip the GUI on headless boots:

```bash
ros2 launch deepgis_vehicles earth_rover_system.launch.py rqt_gui:=false
```

Use a different perspective (basename in `config/`, relative path, or absolute
path):

```bash
ros2 launch deepgis_vehicles earth_rover_system.launch.py \
  rqt_perspective:=/home/jdas/earth-rover/perspectives/debug.perspective
```

The resolver searches, in order: an absolute path, the package share's
`config/`, the package share root, and finally `$EARTH_ROVER_HOME/config/` and
`$EARTH_ROVER_HOME/`.

### Rosbag recording

The trike topic set has its own gracefully-filtered recorder:

```bash
ros2 launch deepgis_vehicles record_bag.launch.py
```

Topics are grouped into classes (`mavros_state`, `mavros_imu`,
`mavros_local_position`, `mavros_global_position`, `mavros_gps_status`,
`stereo_raw`, `stereo_compressed`, `stereo_camera_info`, `spectrometer`,
`laser`, `adsb`, `tf`, `diagnostics`). Each class is independently toggled
via `record_<class>:=true|false` and `stereo_raw` is **off by default**
because of bandwidth.

By default the launcher scans live topics for `discovery_timeout_sec`
seconds and records only the ones currently published — missing topics are
logged and skipped per class so the bag still captures everything else
instead of failing the launch. Use `wait_for_topics:=true` to pre-subscribe
to topics that will appear later (e.g. cameras that come up after the
recorder).

```bash
# Capture everything live, plus topics that come up later
ros2 launch deepgis_vehicles record_bag.launch.py wait_for_topics:=true

# Heavy capture: include raw stereo + zstd file compression + 2 GiB splits
ros2 launch deepgis_vehicles record_bag.launch.py \
  record_stereo_raw:=true \
  compression_mode:=file compression_format:=zstd \
  max_bag_size:=2147483648

# MAVROS-only telemetry capture
make record-bag-mavros
```

Bags land under `~/earth-rover-bags/<bag_name>/` (default `bag_name` is
`earth_rover_<UTC_timestamp>`). Override with `bag_dir:=/data/missions
bag_name:=field_test_2026_05_01`. See `--show-args` for the full knob list.

## ROS 2 instrument packages

These packages live under `packages/` and use the same ROS 2 distro as the rest
of the stack.

### `radio_vio`

`radio_vio` contains the SDR / ADS-B / aircraft-state-vector / landmark-VO
launches. The consolidated launcher is:

```bash
ros2 launch radio_vio radio_stack.launch.py preset:=full
```

Presets:

- `decoder`: RTL-SDR + ADS-B decoder only.
- `state`: decoder plus ADS-B aircraft state vectors.
- `plots`: state vectors plus ADS-B 2D and glide plot images.
- `vio`: state vectors plus landmark VO 2D/fisheye visualizations.
- `full`: ADS-B base, state vectors, ADS-B plots, and landmark VO plots.

```bash
ros2 launch radio_vio rtl_adsb.launch.py
ros2 launch radio_vio adsb_aircraft_state_vectors.launch.py
ros2 launch radio_vio landmark_vo_plot_2d.launch.py estimated_position_topic:=/adsb/rtl_adsb_decoder_node/estimated_position
make radio-stack PRESET=plots
```

Install Python and SDR hardware dependencies as described by the helper scripts
under `packages/radio_vio/scripts/`.

#### Notes / known footguns

- **`mavros_namespace` must be absolute** (e.g. `/mavros`, not `mavros`).
  `adsb_aircraft_state_vectors_node`, `landmark_vo_plot_2d`, and
  `landmark_vo_plot_fisheye` concatenate this arg into absolute topic paths,
  and consumer nodes typically run inside the `/adsb` namespace -- a bare
  `mavros` would silently resolve to `/adsb/mavros/...` and miss every MAVROS
  topic. Nodes now normalize the value defensively, and
  `earth_rover_system.launch.py` defaults to `/mavros`.
- **MAVROS sensor topics use BEST_EFFORT QoS.** All radio_vio subscriptions
  to `/mavros/global_position/{global,raw/fix}`, `/mavros/local_position/*`,
  `/mavros/imu/data` use `qos_profile_sensor_data` / equivalent BEST_EFFORT
  profiles. Using rclpy's default RELIABLE QoS yields
  `incompatible QoS ... Last incompatible policy: RELIABILITY` and zero
  messages -- which previously left
  `/adsb/rtl_adsb_decoder_node/estimated_position` silent because no GPS
  fixes ever reached the receiver-position Kalman filter.
- **Plot publishers handle Jazzy `ExternalShutdownException`** and use
  `FigureCanvasAgg.buffer_rgba()` instead of the removed-in-matplotlib-3.10
  `tostring_rgb()`, so the four image plotters
  (`adsb_state_vectors_plot_2d`, `adsb_state_vectors_plot_glide`,
  `landmark_vo_plot_2d`, `landmark_vo_plot_fisheye`) shut down cleanly on
  Ctrl-C / SIGTERM and survive future matplotlib upgrades.

### `laser_ranger`

Reads a USB serial laser ranger and publishes:

- `std_msgs/String` — first whitespace-delimited token (`topic_raw`, default `serial_data`)
- `std_msgs/Float64` — parsed numeric distance when the token is a float (`topic_distance`, default `laser_distance`)

The system launch (`earth_rover_system.launch.py`) calls this stage by default
with the trike's FTDI FT230X pinned via `/dev/serial/by-id/`, so unplug/replug
and ttyUSB reordering against the Pixhawk no longer break startup. Standalone
invocation:

```bash
ros2 run laser_ranger laser_ranger_node
# Pinned by-id path (preferred, survives reboots / replug):
ros2 launch laser_ranger laser_ranger.launch.py \
  serial_device:=/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO01SSV5-if00-port0
# Direct tty path (bench testing only):
ros2 launch laser_ranger laser_ranger.launch.py serial_device:=/dev/ttyUSB1
```

Find the right by-id path for your unit with `ls /dev/serial/by-id/`; pass it
to the system launch via `laser_serial_device:=<path>` (or set as the new
default in `earth_rover_system.launch.py`).

Python dependency: **`python3-serial`** (`pip install pyserial` if not from apt).

### `spectrometery_ros2`

- **`Spectrometer_Data_Publisher.py`** — SeaBreeze spectrometer → `std_msgs/Float64MultiArray` on `spectrometer` (integration time, intensities, wavelengths). Parameters: `topic`, `integration_time_micros`, `publish_period_sec`.
- **`Intensity_Plot.py`** — subscribes to that topic, publishes a Matplotlib-rendered `sensor_msgs/Image` (`spectrometer_plot` by default), with optional **absorption dip** detection (vertical markers + stats). CLI flags include `--dip-prominence`, `--dip-min-distance`, `--update-rate`, `--dpi`, etc.

**Combined launch** (publisher + plot; matches the usual field command with dip tuning defaults `0.05` / `15`):

```bash
ros2 launch spectrometery_ros2 spectrometer_data_publisher.launch.py
```

Launch arguments:

| Argument | Default | Purpose |
|----------|---------|---------|
| `plot_image` | `true` | Also start `Intensity_Plot.py`; set `false` for hardware publisher only |
| `topic` | `spectrometer` | Spectrum topic for both nodes |
| `integration_time_micros` | `500000` | Exposure |
| `publish_period_sec` | `0.1` | Acquisition period |
| `dip_prominence` | `0.05` | Min dip depth vs spectrum max (passed to `Intensity_Plot`) |
| `dip_min_distance` | `15` | Min index spacing between dips |
| `plot_image_topic` | `spectrometer_plot` | Output image topic |

Dependencies: **`python3-numpy`**, **`python3-matplotlib`**, **`python3-seabreeze`** / SeaBreeze drivers for the spectrometer hardware.

## Rosbag analysis scripts

Post-mission analysis scripts that read recorded rosbag2 directories (mcap or
sqlite3) and emit figures, CSVs, and small reports next to the bag. All three
require a sourced ROS 2 environment for `rosbag2_py` / `rclpy` and run with
matplotlib's `Agg` backend (no display needed). Run them from `scripts/`.

```bash
source /opt/ros/jazzy/setup.bash    # required for rosbag2_py, rclpy
~/earth-rover/scripts/analyze_spectrometer_rosbag.py --bag <bag>
~/earth-rover/scripts/cluster_spectrometer_rosbag.py --bag <bag> --mask-lo 350 --mask-hi 850
~/earth-rover/scripts/geoplot_rosbag.py --bag <bag> --overlay laser --overlay spectra-clusters
```

### `analyze_spectrometer_rosbag.py`

Reads `std_msgs/Float64MultiArray` spectra (default topic `/spectrometer`,
layout `[integration_time_µs, intensities…, wavelengths…]` as published by
`spectrometery_ros2/Spectrometer_Data_Publisher.py`) and renders a
**time–wavelength heatmap** PNG. Useful CLI args: `--topic`, `--stride`,
`--clip-lo / --clip-hi` (percentile color clip), `--cmap`, `--figsize`,
`--dpi`. Also exposes a public `load_spectra_from_bag(bag, topic, stride)`
helper used by the other two scripts.

### `cluster_spectrometer_rosbag.py`

End-to-end **unsupervised report** for a spectrometer bag. Pipeline:

1. wavelength **mask** + **SNV** + **Savitzky–Golay** derivative
2. **PCA** (keep components covering `--pca-variance`, capped at `--max-pcs`)
3. **UMAP** → **HDBSCAN** on the PCA scores (with KMeans fallback)
4. **NMF** on non-negative raw spectra (`--nmf-components`, default 5)

Saves a `<bag>_cluster_report/` directory with PCA scree, UMAP-colored
embedding, cluster mean spectra, NMF endmembers + abundances, a 6-panel
summary, and a text report. Tunable via `--mask-lo / --mask-hi` (default
`None` — pass `350` / `850` to drop the noisy UV/IR edges of the OceanOptics
sensor), `--savgol-window/-poly/-deriv`, `--umap-neighbors / --umap-min-dist`,
`--hdbscan-min-cluster`. Exposes a pure
`compute_clusters_and_nmf(X, wl, **params) -> ClusterPipelineResult` that
`geoplot_rosbag.py` imports.

### `geoplot_rosbag.py`

Plots the trike **GPS path** from a rosbag (default
`/mavros/global_position/raw/fix`, with companion topics
`/mavros/global_position/raw/{gps_vel, satellites}` and
`/mavros/gpsstatus/gps1/raw` for fix-quality / EPH). Outputs to a
`<bag>_geoplot/` directory:

- `path_local.png` — equirectangular meters, equal aspect, color = speed (or
  `--color-by time|altitude|satellites|fix_type`).
- `path_geo.png` — raw lat/lon scatter.
- `altitude.png`, `speed.png`, `satellites.png` — GPS QoS time series.
- `summary.png` — 2×2 dashboard.
- `path.csv` — per-fix `t_unix, t_rel_s, lat, lon, alt_m, speed_mps, vspeed_mps, satellites, fix_type, eph_m, nav_status, cov_xy_m`, plus per-overlay columns when overlays are enabled.
- `report.txt` — distance, duration, lat/lon range, speed/sat/EPH stats,
  fix-type breakdown, NavSatFix sigma stats.

**Overlays** (repeatable, `--overlay <name>`) attach extra per-fix data via
nearest-neighbor / linear time alignment onto the GPS-fix timeline:

| Overlay | Topics read | Adds |
|---------|-------------|------|
| `laser` | `/laser_distance` (`std_msgs/Float64`) | `path_laser.png`, `laser.png` (time series), `laser_m` CSV column |
| `spectra-clusters` | `/spectrometer` | Runs `compute_clusters_and_nmf` internally; `path_clusters.png` (discrete tab20 categorical), `clusters_timeline.png`, `path_nmf_em<i>.png` for each endmember (continuous viridis), and `spectra_cluster` + `nmf_em<i>` CSV columns |
| `adsb-aircraft` | `/adsb/rtl_adsb_decoder_node/aircraft_list` (`std_msgs/String`, JSON) | `path_adsb.png` — trike path + per-aircraft tracks plotted in the same equirectangular meters frame, clipped to a square of side `2·--adsb-half-extent` (default `5000` → 10×10 km), tracks colored by altitude (ft) via a `LineCollection`, top-N callsigns/ICAOs annotated; `adsb_tracks.csv` (one row per deduped position update). |

Tuning for `spectra-clusters` passes through to the cluster pipeline via
`--spectra-mask-lo` (default 350), `--spectra-mask-hi` (default 850),
`--spectra-nmf-components` (default 5), `--spectra-min-cluster`,
`--spectra-stride`. Tuning for `adsb-aircraft`: `--adsb-topic`,
`--adsb-half-extent` (m), `--adsb-cmap`, `--adsb-label-top-n`. New overlays
are added by writing a `_run_*_overlay()` that uses the public
`align_to(target_t, src_t, src_v)` primitive (or, for non-time-series
overlays like aircraft tracks, just the equirectangular projection helpers)
— no other plumbing needed.

### Dependencies

The clustering / overlay paths need: `scikit-learn` (≥1.3 for built-in
HDBSCAN; `hdbscan` package also auto-detected as fallback), `umap-learn`,
`scipy`. Install with `pip install --user scikit-learn umap-learn hdbscan`
(or `apt install python3-sklearn` + pip for the others). On Ubuntu 24.04 /
Python 3.12, `numba 0.65` (a UMAP transitive dep) needs `coverage>=7.6`;
`pip install --user --upgrade coverage` if you hit
`module 'coverage.types' has no attribute 'Tracer'`.

## ORB-SLAM3 monocular (right Grasshopper, rosbag replay)

To run **ORB-SLAM3 in monocular mode** on **`/stereo/right/image_raw`** from a
recorded rosbag2 directory (Bayer `bayer_gbrg8` → `bayer_to_mono.py` →
`/stereo/right/image_mono` → `ros2 run orbslam3 mono`), use the packaged wrapper:

```bash
# One-shot (starts debayer relay + SLAM + filtered bag play; saves trajectories when the bag ends)
cd ~/earth-rover
./scripts/run_orbslam3_right_rosbag.sh \
  --bag ~/earth-rover-bags/earth_rover_20260502_083404 \
  --output-dir reports/orbslam3_right \
  [--rate 1.0] [--downscale 1] [--start 0]

# Headless / CI (no Pangolin window)
ORBSLAM3_NO_VIEWER=1 ./scripts/run_orbslam3_right_rosbag.sh --bag <bag> ...
```

**Prerequisites:**

- ROS 2 **Jazzy** on `PATH`; workspace sourced: `source ~/ros2_ws/install/setup.bash`
  (needs the `orbslam3` package from `orbslam3_ros2` — see
  [`earth-rover.repos`](earth-rover.repos) and [`scripts/setup_workspace.sh`](scripts/setup_workspace.sh)).
- ORB-SLAM3 libraries at **`ORB_SLAM3_ROOT_DIR`** (default
  `~/ORB-SLAM3-STEREO-FIXED`); `LD_LIBRARY_PATH` is extended by the script.
- **`~/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt`**
- Settings YAML — default **`config/monocular/gh.yaml`** (narrow-angle
  calibration from [`external/patches/orbslam3_gh_monocular.yaml`](external/patches/orbslam3_gh_monocular.yaml)).
  The **right** camera in the bag is the **wide-angle** Grasshopper; intrinsics
  will be approximate until a dedicated `*_wide.yaml` exists
  (see docstring in [`scripts/orbslam3_right_rosbag.launch.py`](scripts/orbslam3_right_rosbag.launch.py)).

**Outputs:** `reports/orbslam3_right/<bag_basename>/` receives
`*_CameraTrajectory.txt`, `*_KeyFrameTrajectory.txt`, and log files. The script
`cd`s there before launching so ORB-SLAM3 writes trajectories into that folder.

**Troubleshooting:** If interactive `source ~/ros2_ws/install/setup.bash` fails with
`COLCON_TRACE: unbound variable`, your shell has **Bash nounset** (`set -u`)
enabled. Colcon's `setup.bash` expands `$COLCON_TRACE` without a default.
Either run `set +u` before sourcing (then `set -u` again if you want), or
`export COLCON_TRACE=` once per session. The wrapper script
[`run_orbslam3_right_rosbag.sh`](scripts/run_orbslam3_right_rosbag.sh)
temporarily disables nounset while sourcing ROS workspaces so it works either way.

**Manual two-terminal workflow:** [`scripts/orbslam3_right_rosbag.launch.py`](scripts/orbslam3_right_rosbag.launch.py)
(debayer + mono node only) in terminal 1, then
`ros2 bag play <bag> --topics /stereo/right/image_raw` in terminal 2.

## Systemd User Units

Nothing in this repo should auto-start at boot. The remaining `systemd --user`
units are an explicit operator path for launching individual services, the
mission target, the UI layer, or the archive job.

Install or refresh the unit files:

```bash
cd ~/earth-rover/scripts/startup
./install_user_units.sh
```

Manual service examples:

```bash
systemctl --user start er-mavros.service
systemctl --user start er-grasshopper.service
systemctl --user start er-rtl-adsb.service er-adsb-state.service
```

Mission target helpers:

```bash
make mission-up
make mission-status
make mission-down
```

UI and archive helpers:

```bash
make ui-up
make ui-status
make ui-down
make archive-now
make archive-status
```

See [`scripts/startup/README.md`](scripts/startup/README.md) for the current
startup notes and the list of legacy scripts that were removed.

## MAVROS / Pixhawk

The main physical Pixhawk path on the rover is an FTDI serial-by-id link at
921600 baud:

```bash
ros2 launch mavros px4.launch \
  fcu_url:="/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600" \
  gcs_url:="udp://@192.168.0.6:14550"
```

For the `deepgis_vehicles` C++ bridge:

```bash
ros2 launch deepgis_vehicles vehicle_interface.launch.py \
  fcu_url:="/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600"
```

See [`CONNECTION_GUIDE.md`](CONNECTION_GUIDE.md) for device discovery,
permissions, and alternate baud/device paths.

## Documentation

Top-level docs tracked in this repo:

- [`CONNECTION_GUIDE.md`](CONNECTION_GUIDE.md) — Connecting to Pixhawk via MAVROS2 (serial-by-id, USB, ACM, SITL).
- [`DEEPGIS_TELEMETRY_README.md`](DEEPGIS_TELEMETRY_README.md) — DeepGIS telemetry publisher (continuous geospatial uplink from the rover).
- [`QUICK_START_TELEMETRY.md`](QUICK_START_TELEMETRY.md) — Quick-start guide for the DeepGIS telemetry stack.
- [`API_COMPLIANCE_CHANGES.md`](API_COMPLIANCE_CHANGES.md) — DeepGIS API compliance change log.
- [`scripts/startup/README.md`](scripts/startup/README.md) — Current startup notes, systemd `--user` units, kiosk helper, and removed legacy scripts.
- [`vehicle_control_station/README.md`](vehicle_control_station/README.md) — Web-based vehicle control station.

ADS-B receiver-position estimation, landmark visual odometry plotting, and
state-estimation analyses are kept as design notes under `docs/` when present.

## Cloning

This repo tracks three git submodules:

- `kernelcal` — kernel-dynamics / Maximum-Caliber experiments.
- `external/deepgis_vision` — Grasshopper stereo / AI perception fork.
- `external/metavision_driver` — Prophesee event-camera ROS 2 driver fork.

Clone with `--recurse-submodules`:

```bash
git clone --recurse-submodules git@github.com:Earth-Innovation-Hub/earth-rover.git
```

If you already cloned without `--recurse-submodules`:

```bash
cd earth-rover
git submodule update --init --recursive
```

To later pull updates (including submodule bumps):

```bash
git pull --recurse-submodules
```

For a full ROS 2 workspace (submodules + `vcs import` from
[`earth-rover.repos`](earth-rover.repos) + local patches + `colcon build`),
run [`scripts/setup_workspace.sh`](scripts/setup_workspace.sh) — see
[Current ROS 2 Stack](#current-ros-2-stack).

---

# deepgis_vehicles ROS2 Package

ROS 2 package for connecting with Pixhawk PX4 autopilot using MAVROS2. Vehicle
launch files live under `launch/` at the repo root and are installed with
`deepgis_vehicles`. The SDR / ADS-B / radio-VIO stack is the separate
`radio_vio` package under `packages/radio_vio/`.

## Overview

This package provides a ROS2 interface to communicate with Pixhawk PX4 flight controllers via MAVROS2. It includes:

- **vehicle_interface_node**: A C++ node that interfaces with MAVROS2 to:
  - Subscribe to vehicle state, position, and sensor data
  - Publish commands (position setpoints, velocity commands)
  - Provide services for arming/disarming and mode changes
  - Publish processed vehicle data as standard ROS2 topics

## Dependencies

- ROS 2 Jazzy
- MAVROS2 (`mavros`, `mavros_msgs`, `mavros_extras`)
- Standard ROS2 packages: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`
- TF2 for coordinate transformations

## Installation

### Install MAVROS2

Before building this package, you need to install MAVROS2:

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
```

**Note:** If MAVROS2 packages are not available via apt, you may need to build them from source. See the [MAVROS2 GitHub repository](https://github.com/mavlink/mavros) for instructions.

### Install GeographicLib datasets (required for MAVROS2)

MAVROS2 requires GeographicLib datasets for coordinate transformations:

```bash
sudo geographiclib-get-geoids egm96-5
```

## Building

```bash
cd ~/earth-rover
source /opt/ros/jazzy/setup.bash
colcon build --packages-select deepgis_vehicles
source install/setup.bash
```

## Usage

### Launch with PX4 SITL

For testing with PX4 SITL:

```bash
ros2 launch deepgis_vehicles vehicle_interface.launch.py \
    fcu_url:="udp://:14540@127.0.0.1:14557"
```

### Launch with Physical Pixhawk

Use serial-by-id paths when possible so USB port order does not change the
connection URL.

```bash
ls -la /dev/serial/by-id/
```

Current rover FTDI telemetry link:

```bash
ros2 launch deepgis_vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600"
```

Direct device paths also work for bench testing:

```bash
ros2 launch deepgis_vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/ttyUSB0:57600"

ros2 launch deepgis_vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/ttyACM0:57600"
```

### Launch Parameters

- `fcu_url`: Connection URL to the flight controller
  - SITL: `udp://:14540@127.0.0.1:14557`
  - Rover FTDI telemetry: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600`
  - USB: `/dev/ttyUSB0:57600`
  - ACM: `/dev/ttyACM0:57600`
  - TCP: `tcp://127.0.0.1:5760`
  - Baud rates: common values are 57600, 115200, and 921600
- `gcs_url`: Ground Control Station URL (default: `udp://@127.0.0.1:14550`)
- `tgt_system`: Target system ID (default: `1`)
- `tgt_component`: Target component ID (default: `1`)
- `mavros_namespace`: MAVROS namespace (default: `/mavros`)

## Topics

### Subscribed (from MAVROS2)

- `/mavros/state` - Vehicle state (armed, mode, connected)
- `/mavros/local_position/pose` - Local position estimate
- `/mavros/global_position/global` - Global GPS position

### Published (to MAVROS2)

- `/mavros/setpoint_velocity/cmd_vel` - Velocity commands
- `/mavros/setpoint_raw/local` - Raw position target

### Published (processed data)

- `vehicle/odometry` - Vehicle odometry (nav_msgs/Odometry)
- `vehicle/connected` - Connection status (std_msgs/Bool)

## Services

The node provides access to MAVROS2 services:

- `/mavros/cmd/arming` - Arm/disarm the vehicle
- `/mavros/set_mode` - Change flight mode

## Example: Arm and Publish a Rover Velocity Command

```bash
# In one terminal, launch the interface
ros2 launch deepgis_vehicles vehicle_interface.launch.py

# In another terminal, arm the vehicle
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Set to OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{custom_mode: 'OFFBOARD'}"

# Publish a conservative forward velocity setpoint
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}}"
```

## Monitoring

Check vehicle connection status:

```bash
ros2 topic echo /mavros/state
ros2 topic echo /vehicle/connected
ros2 topic echo /vehicle/odometry
```

## Configuration

Edit `config/mavros_config.yaml` to customize MAVROS2 parameters.

## Notes

- Ensure MAVROS2 is installed: `sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras`
- For USB connections, you may need to add your user to the `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- The node automatically handles connection status and logs important state changes
- Position setpoints should be published at a minimum rate (typically 10-30 Hz) for OFFBOARD mode

## License

BSD-3-Clause
