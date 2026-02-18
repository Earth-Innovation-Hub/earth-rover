# State Estimation: ADS-B–VO Formalism

This document describes the state estimation formalism for the Earth Rover trike, focusing on **ADS-B** (aircraft tracking) and **landmark-based Visual Odometry (VO)** integration, coordinate frames, observation models, and visualization tools.

---

## 1. Overview

The system combines:

1. **MAVROS** – vehicle pose (position, orientation) from Pixhawk/IMU/GPS
2. **ADS-B** – aircraft positions in trike-centric local ENU, derived from 1090 MHz radio
3. **Landmark-based VO** – predicted vs actual landmark projections in the camera image, with epipolar-style residuals
4. **Camera model** – Grasshopper left (Chameleon narrow-angle) pinhole model for projection

Unifying theme: **all observations (landmarks, aircraft) are expressed in a common frame and projected into the camera image using the current estimated pose**, enabling residual-based diagnostics and future filter fusion.

---

## 2. Coordinate Frames

### 2.1 Local ENU (world / map)

- **X** = East  
- **Y** = North  
- **Z** = Up  

MAVROS local position (`/mavros/local_position/pose`, `/mavros/local_position/odom`) is typically in this frame. Origin depends on configuration (often UTM or first GPS fix).

### 2.2 Trike-centric ENU (local_enu)

- **Origin** = trike position (current)
- **X** = East  
- **Y** = North  
- **Z** = Up  

Used by **ADS-B aircraft state vectors**: each aircraft position is `(east, north, up)` relative to the trike. The trike is always at `(0, 0, 0)` in this frame. This frame is published by `adsb_aircraft_state_vectors_node` with `frame_id: local_enu`.

### 2.3 Body frame (trike)

- **X** = forward (heading direction)
- **Y** = left
- **Z** = up

Rotation from world ENU to body is yaw only (around Z). Yaw = 0 typically corresponds to North in ENU.

### 2.4 Camera optical frame (ROS convention)

- **X** = right (in image)
- **Y** = down (in image)
- **Z** = forward (optical axis, viewing direction)

The camera is assumed **aligned with the trike heading**: optical axis = body X = forward direction.

Mapping: body X → cam Z, body Y → cam −X, body Z → cam −Y.

---

## 3. ADS-B Formalism

### 3.1 Data flow

```
ADS-B 1090 MHz receivers (RTL-SDR / Hydra)
        ↓
aircraft_list (lat, lon, alt, speed, heading, vertical_rate, ...)
        ↓
adsb_aircraft_state_vectors_node
  • Trike LLA from MAVROS global_position
  • lla_to_enu(lat, lon, alt, lat0, lon0, alt0) → (east, north, up)
  • Output: trike-centric ENU, origin at trike
        ↓
aircraft_state_vectors (JSON)
  • trike: { pose, twist }
  • aircraft: [ { icao, pose { position {x,y,z}, orientation }, twist, raw }, ... ]
```

### 3.2 Aircraft state in trike-centric ENU

For each aircraft:

- **Position** `(x, y, z)` = East, North, Up (m) relative to trike
- **Orientation** from heading (deg) and pitch (from climb/descent)
- **Velocity** `(vx, vy, vz)` in ENU

Altitude is above the trike reference altitude (from MAVROS global position).

### 3.3 Related estimation

Receiver (trike) position from ADS-B alone is estimated by `estimate_receiver_position` (sdr_adsb_common) and a 2D Kalman filter in `rtl_adsb_decoder_node`. See `docs/ADS_B_POSE_ESTIMATION_ANALYSIS.md`.

---

## 4. Landmark-based VO Formalism

### 4.1 Problem

Given:

- **Estimated pose** `(x, y, θ)` from odometry  
- **Landmarks** in world frame `(Lx, Ly)` (e.g., ground-plane features)  
- **Actual observations** in image `(u, v)` from a detector/tracker  

We compute **predicted observations** by projecting landmarks through the camera model and compare with actual observations.

### 4.2 Observation model

**Predicted observation** for landmark `L` at `(Lx, Ly)`:

1. Transform to body:  
   `dx = Lx − x`, `dy = Ly − y`  
   `lx_b = cos(−θ)·dx − sin(−θ)·dy`  
   `ly_b = sin(−θ)·dx + cos(−θ)·dy`

2. Vector from camera to landmark (body):  
   `(vx, vy, vz)` with landmark at `z = 0`, camera at `(cam_x, cam_y, cam_z)`

3. Body → camera optical:  
   `x_cam = −vy`, `y_cam = −vz`, `z_cam = vx`

4. Pinhole projection:  
   `u = fx·(x_cam/z_cam) + cx`  
   `v = fy·(y_cam/z_cam) + cy`

### 4.3 Epipolar-style residuals (innovation)

The **innovation** (reprojection residual) is the vector from **predicted** `(pu, pv)` to **actual** `(ou, ov)` in the image:

```
innovation = (ou − pu, ov − pv)
```

In 2D, this corresponds to the reprojection error used in bundle adjustment and EKF updates. The landmark VO plot visualizes these as cyan arrows (predicted → actual).

### 4.4 3D points (e.g., aircraft)

For 3D points at `(x_w, y_w, z_w)` (e.g., aircraft with altitude), the same transform applies with non-zero `z_w`:

- `dz = z_w − 0` (relative to ground at origin)  
- Body frame includes `lz_b = dz`  
- Camera frame: `y_cam = −vz` with `vz = lz_b − cam_z`

---

## 5. Camera Model

### 5.1 Grasshopper left (Chameleon narrow-angle)

- **Intrinsics**: `fx`, `fy`, `cx`, `cy` (pinhole)
- **Resolution**: 1280×1024 (configurable)
- **Default**: `fx = fy = 1000`, `cx = 640`, `cy = 512`

Override via `/stereo/left/camera_info` when available.

### 5.2 Extrinsics (body frame)

- `cam_offset_x`, `cam_offset_y` = camera position in body (m)
- `cam_height` = height above ground (m)

Default: camera at body origin, 1.2 m height.

### 5.3 Field of view

Horizontal FOV (rad): `2·atan(width / (2·fx))`. For `fx=1000`, width=1280 → ~53°.

---

## 6. Topics and Data Formats

### 6.1 Input topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/local_position/odom` | nav_msgs/Odometry | Raw vehicle pose (BEST_EFFORT QoS) |
| `/adsb/rtl_adsb_decoder_node/estimated_position` | std_msgs/String (JSON) | ADS-B KF estimated position (`kf_lat`, `kf_lon`) for **predictions**; yaw from odom |
| `/vo/landmarks` | std_msgs/String | JSON: `{"landmarks":[{"id":0,"x":1.0,"y":2.0},...]}` |
| `/vo/landmark_observations` | std_msgs/String | JSON: `{"observations":[{"id":0,"u":320,"v":256},...]}` |
| `/adsb/.../aircraft_state_vectors` | std_msgs/String | Trike-centric aircraft states |
| `/stereo/left/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics (optional) |

### 6.2 Output topics

| Topic | Type | Description |
|-------|------|-------------|
| `/landmark_vo_plot_2d/landmark_vo_plot_image` | sensor_msgs/Image | Visualization (predicted vs actual, aircraft, world view) |

---

## 7. Visualization: landmark_vo_plot_2d

The `landmark_vo_plot_2d` node produces a two-panel image:

1. **Left (image plane)**  
   - Green circles: predicted landmark positions (pose from KF when `estimated_position_topic` is set, else MAVROS odom)  
   - Red crosses: actual observations  
   - Cyan arrows: innovation vectors (epipolar-style residuals)  
   - Orange triangles: ADS-B aircraft (projected using trike-centric coords + MAVROS yaw)  
   - Pose label shows source: `[estimated]` (KF) or `[odom]` (MAVROS)

2. **Right (world 2D, odom frame)**  
   - Landmarks (blue) – world/odom frame  
   - Vehicle (red) – KF position when available, else MAVROS  
   - Aircraft (orange) – trike-centric positions transformed to odom (vehicle + ac_offset)  
   - Camera FOV cone (cyan)

### 7.1 Demo mode

With `demo_mode:=true`, synthetic landmarks and observations are generated when no real data is available. Useful for testing without VO/landmark publishers.

### 7.2 Verification

- Publish counter `#N` (top-right) increments each frame  
- Check rate: `ros2 topic hz /landmark_vo_plot_2d/landmark_vo_plot_image`  
- Script: `scripts/check_landmark_vo_plot.sh`

---

## 8. Launch and Configuration

```bash
# Demo mode (synthetic data)
ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py demo_mode:=true

# With ADS-B aircraft overlay (default)
ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py

# Custom odom topic
ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py odom_topic:=/vehicle/odometry
```

Config: `config/landmark_vo_plot_2d.yaml`

---

## 9. Estimated vs Raw Pose

By default (`estimated_position_topic` set), **predicted** observations use the ADS-B KF estimated position:

The node subscribes to `/adsb/rtl_adsb_decoder_node/estimated_position`.
It parses `kf_lat`, `kf_lon` from the JSON, converts to local (x, y) via lla_to_enu using a bootstrap ref from odom and global_position, and uses odom for yaw. The pose label shows `[estimated]` or `[odom]` accordingly. Set `estimated_position_topic: ""` to always use raw odom.

---

## 10. Future Directions

- **Filter fusion**: fuse odometry, landmark residuals, and ADS-B into an EKF/UKF
- **ADS-B as landmarks**: aircraft are already projected; could use as pseudo-landmarks for heading/scale
- **Multi-camera**: extend to right stereo / fisheye with per-camera models
- **SLAM integration**: landmark map with data association and loop closure

---

## 11. References

- `docs/ADS_B_POSE_ESTIMATION_ANALYSIS.md` – ADS-B receiver position KF and robust estimator
- `scripts/landmark_vo_plot_2d.py` – Implementation of projection, residuals, visualization
- `scripts/adsb_aircraft_state_vectors_node.py` – Trike-centric aircraft state vectors
