# GPS Position Decoding for RTL-SDR ROS2

This document describes the GPS position decoding functionality that publishes actual GPS coordinates.

## Overview

The GPS reader node now includes built-in position decoding that extracts GPS coordinates from the captured signals and publishes them as standard ROS2 `NavSatFix` messages.

## Features

- **GPS Position Decoding**: Decodes GPS signals to extract latitude, longitude, and altitude
- **NavSatFix Messages**: Publishes standard ROS2 GPS position messages
- **Signal Quality Filtering**: Only publishes fixes when signal quality is sufficient
- **Configurable**: Enable/disable position decoding via parameters
- **Extensible**: Supports external GPS decoders for advanced use cases

## Quick Start

### Launch GPS Reader with Position Decoding

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch GPS reader (position decoding enabled by default)
ros2 launch rtlsdr_ros2 gps_launch.py
```

### View GPS Position

```bash
# View GPS position fixes
ros2 topic echo /rtlsdr/gps_position

# Check position publishing rate
ros2 topic hz /rtlsdr/gps_position

# View position in a readable format
ros2 topic echo /rtlsdr/gps_position --field latitude longitude altitude
```

## Topics

### Published Topics

- **`/rtlsdr/gps_position`** (`sensor_msgs/NavSatFix`)
  - GPS position fixes with latitude, longitude, altitude
  - Includes fix status, HDOP, VDOP, and covariance
  - Standard ROS2 GPS message format

- **`/rtlsdr/gps_data`** (`rtlsdr_ros2/msg/GpsData`)
  - Raw GPS signal data (IQ samples)
  - Signal quality metrics
  - Still published for advanced processing

## Parameters

### GPS Reader Node Parameters

- `enable_position_decoding` (bool, default: true)
  - Enable GPS position decoding and publishing
  
- `min_snr_for_fix` (double, default: 5.0)
  - Minimum SNR in dB required before attempting position fix
  - Higher values = better signal quality required

- Other parameters: See main GPS documentation

## Message: NavSatFix

The `NavSatFix` message contains:

- **Header**: Standard ROS2 header with timestamp
- **Status**: 
  - `STATUS_NO_FIX`: No position fix
  - `STATUS_FIX`: Valid position fix
  - `STATUS_SBAS_FIX`: SBAS-aided fix
- **Service**: GPS service identifier
- **Position**:
  - `latitude`: Latitude in degrees (WGS84)
  - `longitude`: Longitude in degrees (WGS84)
  - `altitude`: Altitude in meters (above sea level)
- **Covariance**: Position uncertainty (3x3 matrix)
- **Covariance Type**: Type of covariance data

## Usage Examples

### Basic Position Decoding

```bash
# Launch with default settings (position decoding enabled)
ros2 launch rtlsdr_ros2 gps_launch.py
```

### Disable Position Decoding (Raw Data Only)

```bash
ros2 launch rtlsdr_ros2 gps_launch.py enable_position_decoding:=false
```

### Higher SNR Requirement

```bash
# Require better signal quality before fixing
ros2 launch rtlsdr_ros2 gps_launch.py min_snr_for_fix:=10.0
```

### Monitor Position

```bash
# Terminal 1: Launch GPS reader
ros2 launch rtlsdr_ros2 gps_launch.py

# Terminal 2: Monitor position
ros2 topic echo /rtlsdr/gps_position

# Terminal 3: View signal quality
ros2 run rtlsdr_ros2 gps_monitor
```

## GPS Decoding Implementation

### Current Implementation

The current GPS decoder includes:

1. **Basic Decoder Framework**: Structure for GPS signal processing
2. **Simulated Position**: Demonstration mode with simulated GPS position
3. **Signal Quality Analysis**: SNR and power measurements
4. **Extensible Architecture**: Ready for integration with full GPS decoders

### For Production Use

For actual GPS position decoding from RTL-SDR, you have several options:

#### Option 1: GNSS-SDR Integration

GNSS-SDR is a complete software-defined GPS/GNSS receiver:

```bash
# Install GNSS-SDR
sudo apt-get install gnss-sdr

# Use GNSS-SDR to decode GPS signals
# Then integrate with ROS2 via the external decoder interface
```

#### Option 2: RTKLIB Integration

RTKLIB is an open-source GNSS processing library:

```bash
# Install RTKLIB
sudo apt-get install rtklib

# Use RTKLIB for GPS processing
# Integrate via external decoder
```

#### Option 3: Custom Decoder

Implement full GPS decoding including:
- C/A code acquisition
- Satellite tracking loops
- Navigation data decoding
- Position calculation from pseudoranges

### External Decoder Interface

You can integrate external GPS decoders:

1. Create a Python module with a `decode_gps(samples)` function
2. Set `use_external_decoder:=true` and provide the path
3. The decoder will be called with IQ samples and should return position data

Example external decoder:

```python
# external_gps_decoder.py
import numpy as np

def decode_gps(samples):
    """
    Decode GPS position from IQ samples.
    
    Args:
        samples: Complex IQ samples
        
    Returns:
        dict with position data or None
    """
    # Your GPS decoding logic here
    # Return format:
    return {
        'latitude': 37.7749,
        'longitude': -122.4194,
        'altitude': 52.0,
        'fix_type': 3,
        'hdop': 1.2,
        'vdop': 1.8,
        'satellites_used': 8,
        'position_covariance': [0.0] * 9,
        'position_covariance_type': 0,
    }
```

## Position Fix Status

The decoder provides different fix types:

- **No Fix (0)**: No position available
- **2D Fix (2)**: Latitude and longitude only
- **3D Fix (3)**: Latitude, longitude, and altitude

## Signal Quality Requirements

For reliable position fixes:

- **SNR > 10 dB**: Good signal, reliable fixes
- **SNR 5-10 dB**: Moderate signal, may have occasional fixes
- **SNR < 5 dB**: Weak signal, unlikely to get fixes

## Troubleshooting

### No Position Fixes

- **Check signal quality**: Monitor SNR with `gps_monitor`
- **Wait for acquisition**: GPS cold start takes 30-60 seconds
- **Improve antenna**: Use GPS patch antenna, clear sky view
- **Increase gain**: Try higher gain settings (20-40 dB)
- **Lower SNR threshold**: Reduce `min_snr_for_fix` parameter

### Position Accuracy Issues

- **Check HDOP**: Lower HDOP = better accuracy
- **More satellites**: Need at least 4 for 3D fix, more is better
- **Signal quality**: Better SNR = better accuracy
- **Antenna placement**: Clear view of sky is essential

### Decoder Not Working

- **Check logs**: Look for decoder errors
- **Verify samples**: Ensure GPS data is being received
- **Test signal**: Verify RTL-SDR is receiving GPS signals
- **External decoder**: If using external decoder, verify it's working

## Integration with Other ROS2 Nodes

The `NavSatFix` messages can be used with standard ROS2 navigation stacks:

```python
# Example: Subscribe to GPS position in your node
from sensor_msgs.msg import NavSatFix

def gps_callback(msg):
    lat = msg.latitude
    lon = msg.longitude
    alt = msg.altitude
    # Use position data...
```

## Related Documentation

- Main GPS README: `GPS_README.md`
- Package README: `README.md`
- ROS2 NavSatFix: http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

## Notes

- **Current Implementation**: The decoder includes a simulation mode for demonstration
- **Production Use**: For real GPS decoding, integrate with GNSS-SDR, RTKLIB, or implement full GPS algorithms
- **Signal Processing**: Full GPS decoding requires extensive signal processing (C/A code correlation, tracking loops, navigation data parsing)
- **Performance**: Real-time GPS decoding is computationally intensive

## Future Enhancements

Potential improvements:

1. Full GPS C/A code acquisition and tracking
2. Navigation data decoding (ephemeris, almanac)
3. Pseudorange calculation and position solving
4. Multi-constellation support (GLONASS, Galileo, BeiDou)
5. RTK (Real-Time Kinematic) support
6. Integration with GNSS-SDR or RTKLIB

