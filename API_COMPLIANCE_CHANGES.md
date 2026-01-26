# DeepGIS API Compliance Changes

This document summarizes the changes made to make the telemetry publisher fully compliant with the DeepGIS Telemetry API specification.

## Summary of Changes

The telemetry publisher has been completely rewritten to conform to the DeepGIS API specification. All data formats now match exactly what the API expects.

## 1. Session Creation Format ✅

### Before (Non-compliant)
```python
{
    'vehicle_id': 'pixhawk_001',
    'session_name': 'flight_20251124_213440',
    'start_time': '2025-11-24T21:34:40.123456',
    'metadata': {
        'source': 'ros2_mavros',
        'node_name': 'deepgis_telemetry_publisher'
    }
}
```

### After (API Compliant) ✅
```python
{
    'session_id': 'mavros_20251124_213440',
    'asset_name': 'MAVROS Vehicle',
    'project_title': 'MAVROS Data Collection',
    'flight_mode': 'AUTO',
    'mission_type': 'Telemetry Collection',
    'notes': 'Automated telemetry collection from MAVROS'
}
```

**Changes:**
- Renamed `vehicle_id` → `session_id` (now auto-generated as `mavros_YYYYMMDD_HHMMSS`)
- Renamed `session_name` → `asset_name` (vehicle/asset identifier)
- Added `project_title` field
- Added `flight_mode` field
- Added `mission_type` field
- Added `notes` field
- Removed `metadata` object

## 2. Local Position Odometry Format ✅

### Before (Non-compliant)
```python
{
    'session_id': '...',
    'timestamp': '...',
    'header': {...},
    'pose': {
        'position': {'x': 10.5, 'y': 5.2, 'z': 2.1},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    },
    'twist': {
        'linear': {'x': 1.2, 'y': 0.8, 'z': 0.1},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.01}
    }
}
```

### After (API Compliant) ✅
```python
{
    'session_id': '...',
    'timestamp': '2025-11-24T21:34:40.123456Z',
    'timestamp_usec': 1234567890000000,
    'x': 10.5,              # NED position (meters)
    'y': 5.2,
    'z': -2.1,
    'vx': 1.2,              # Velocity (m/s)
    'vy': 0.8,
    'vz': -0.1,
    'heading': 0.785,       # Converted from quaternion (radians)
    'heading_rate': 0.01,
    'position_covariance': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],  # 9 elements
    'velocity_covariance': [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5],  # 9 elements
    'ref_lat': 33.4255,     # Reference position
    'ref_lon': -111.9400,
    'ref_alt': 350.0
}
```

**Changes:**
- Flattened nested structure (removed `pose` and `twist` objects)
- Added `timestamp_usec` (microseconds since epoch)
- Converted quaternion orientation to `heading` angle (radians)
- Renamed `twist.angular.z` → `heading_rate`
- Added `position_covariance` (3x3 matrix as 9-element array)
- Added `velocity_covariance` (3x3 matrix as 9-element array)
- Added reference position fields: `ref_lat`, `ref_lon`, `ref_alt`
- Removed `header` object

**New Features:**
- **Quaternion to Heading Conversion**: Properly converts quaternion to yaw angle using `atan2`
- **Covariance Extraction**: Extracts 3x3 position/velocity covariance from 6x6 ROS covariance matrices
- **Reference Position Management**: Automatically sets and includes reference position from first GPS fix

## 3. GPS Fix Format ✅

### Before (Non-compliant)
```python
{
    'session_id': '...',
    'timestamp': '...',
    'header': {...},
    'latitude': 33.4255,
    'longitude': -111.9400,
    'altitude': 352.5,
    'position_covariance': [36 elements],
    'position_covariance_type': 2,
    'status': {
        'status': 0,
        'service': 1
    }
}
```

### After (API Compliant) ✅
```python
{
    'session_id': '...',
    'timestamp': '2025-11-24T21:34:40.123456Z',
    'timestamp_usec': 1234567890000000,
    'latitude': 33.4255,
    'longitude': -111.9400,
    'altitude': 352.5,
    'fix_type': 3,          # Mapped: 0=no fix, 3=3D, 4=SBAS, 6=RTK
    'eph': 2.5,             # Horizontal accuracy (meters)
    'epv': 3.0              # Vertical accuracy (meters)
}
```

**Changes:**
- Added `timestamp_usec` (microseconds)
- Removed `header` object
- Removed `position_covariance` array
- Removed `position_covariance_type`
- Removed `status` object
- Added `fix_type` (integer mapped from NavSatStatus)
- Added `eph` (horizontal position accuracy in meters)
- Added `epv` (vertical position accuracy in meters)

**New Features:**
- **GPS Fix Type Mapping**: Maps ROS NavSatStatus to standard GPS fix types
- **Accuracy Extraction**: Calculates horizontal/vertical accuracy from covariance matrix

## 4. Batch Upload Format ✅

### Before (Non-compliant)
```python
{
    'session_id': '...',
    'timestamp': '...',
    'local_positions': [...],
    'gps_raw': [...],
    'gps_estimated': [...]
}
```

### After (API Compliant) ✅
```python
{
    'local_position_odom': [...],
    'gps_fix_raw': [...],
    'gps_fix_estimated': [...]
}
```

**Changes:**
- Renamed `local_positions` → `local_position_odom`
- Renamed `gps_raw` → `gps_fix_raw`
- Renamed `gps_estimated` → `gps_fix_estimated`
- Removed top-level `session_id` and `timestamp` (each record has its own)

## 5. Configuration Parameters

### New Parameters (API Compliant)
- `asset_name` - Vehicle/asset name (replaces `vehicle_id`)
- `session_id` - Unique session identifier (auto-generated if empty)
- `project_title` - Project name/title
- `flight_mode` - Flight mode (MANUAL, AUTO, GUIDED, etc.)
- `mission_type` - Mission type description
- `notes` - Optional notes about the session

### Updated Default Values
- `deepgis_api_url`: Changed from `https://deepgis.org` → `http://192.168.0.186:8080`

### Removed Parameters
- `vehicle_id` (replaced by `asset_name`)
- `session_name` (replaced by `session_id`)

## 6. New Utility Functions

### `quaternion_to_heading(x, y, z, w)`
Converts quaternion orientation to heading angle (yaw) in radians.

```python
siny_cosp = 2.0 * (w * z + x * y)
cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
heading = math.atan2(siny_cosp, cosy_cosp)
```

### `extract_covariance_3x3(covariance_array, start_idx)`
Extracts 3x3 covariance matrix from ROS 6x6 covariance array.

```python
# Extracts position covariance (indices 0-2)
# Or velocity covariance (indices 3-5)
# Returns 9-element flat array
```

### `extract_gps_accuracy(msg)`
Calculates horizontal (eph) and vertical (epv) accuracy from GPS covariance.

```python
var_lat = covariance[0]
var_lon = covariance[4]
var_alt = covariance[8]
eph = sqrt((var_lat + var_lon) / 2)
epv = sqrt(var_alt)
```

### `map_gps_fix_type(status)`
Maps ROS NavSatStatus to standard GPS fix types.

```python
STATUS_NO_FIX (-1)  → 0 (No fix)
STATUS_FIX (0)      → 3 (3D fix)
STATUS_SBAS_FIX (1) → 4 (SBAS)
STATUS_GBAS_FIX (2) → 6 (RTK)
```

### `set_reference_position(msg)`
Automatically sets reference position from first valid GPS fix.

```python
# Called on first GPS message with valid fix
self.ref_lat = msg.latitude
self.ref_lon = msg.longitude
self.ref_alt = msg.altitude
```

## Usage Examples

### Basic Launch (New Parameters)
```bash
ros2 launch deepgis_vehicles full_system.launch.py \
    asset_name:="Survey Drone 01"
```

### Custom Session
```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    asset_name:="Racing Drone" \
    session_id:="race_mission_001" \
    project_title:="Drone Racing Championship" \
    flight_mode:="MANUAL" \
    mission_type:="Racing"
```

### With Custom API URL
```bash
ros2 launch deepgis_vehicles full_system.launch.py \
    deepgis_api_url:=http://192.168.0.186:8080 \
    asset_name:="Test Vehicle"
```

## Testing Checklist

- [x] Session creation with all required fields
- [x] Local position odometry with flat structure
- [x] Quaternion to heading conversion
- [x] Position/velocity covariance extraction
- [x] Reference position from GPS
- [x] GPS fix with accuracy (eph/epv)
- [x] GPS fix type mapping
- [x] Batch upload with correct field names
- [x] Timestamp in ISO format + microseconds
- [x] All data types as float/int (not numpy)

## API Endpoints

All endpoints remain the same:

- `POST /api/telemetry/session/create/`
- `POST /api/telemetry/local-position-odom/`
- `POST /api/telemetry/gps-fix-raw/`
- `POST /api/telemetry/gps-fix-estimated/`
- `POST /api/telemetry/batch/`

## Migration Notes

If you have existing launch files or configs, update:

1. Replace `vehicle_id` → `asset_name`
2. Replace `session_name` → `session_id`
3. Update `deepgis_api_url` default if needed
4. Add optional parameters: `project_title`, `flight_mode`, `mission_type`

## Verification

To verify API compliance, check logs for:

```
[INFO] [...]: Created telemetry session: mavros_20251124_213440
[INFO] [...]: Set reference position: (33.425500, -111.940000, 350.00m)
[INFO] [...]: Sent batch: 10 odom, 10 GPS raw, 10 GPS est (30 total)
```

Data format can be verified by monitoring network traffic:
```bash
# Monitor API requests
tcpdump -i any -A 'port 8080 and host 192.168.0.186'
```

## Status

✅ **Fully Compliant** - All changes implemented and tested.

The telemetry publisher now conforms 100% to the DeepGIS Telemetry API specification.

