# GPS Reader for RTL-SDR ROS2

This document describes the GPS reader functionality added to the RTL-SDR ROS2 package.

## Overview

The GPS reader node (`gps_reader`) is specifically designed to capture GPS L1 signals (1575.42 MHz) using an RTL-SDR device. It provides:

- **GPS L1 Signal Capture**: Tuned to 1575.42 MHz (GPS L1 C/A code frequency)
- **Signal Quality Analysis**: Real-time SNR, power, and noise floor measurements
- **GPS Data Publishing**: Publishes raw IQ samples and signal metrics via ROS2 topics
- **GPS Monitoring**: Companion node for real-time signal quality monitoring

## GPS Signal Characteristics

- **GPS L1 Frequency**: 1575.42 MHz
- **Signal Strength**: Very weak (-130 dBm typical)
- **Requirements**: 
  - Good outdoor antenna (GPS patch antenna recommended)
  - Clear view of sky
  - Higher gain settings (20-40 dB)
  - Proper antenna positioning

## Quick Start

### 1. Launch GPS Reader

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch with default settings (GPS L1)
ros2 launch rtlsdr_ros2 gps_launch.py
```

### 2. Monitor GPS Signal Quality

In a separate terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rtlsdr_ros2 gps_monitor
```

### 3. View GPS Data

```bash
# View GPS data messages
ros2 topic echo /rtlsdr/gps_data

# Check publishing rate
ros2 topic hz /rtlsdr/gps_data

# View signal quality metrics
ros2 topic echo /rtlsdr/gps_data --field snr_db signal_power_db
```

## Nodes

### gps_reader

Main node that captures GPS signals from RTL-SDR.

**Published Topics:**
- `/rtlsdr/gps_data` (`rtlsdr_ros2/msg/GpsData`) - GPS signal data with IQ samples and quality metrics

**Parameters:**
- `device_index` (int, default: 0) - RTL-SDR device index
- `center_frequency` (double, default: 1575.42e6) - Center frequency in Hz
- `sample_rate` (double, default: 2.048e6) - Sample rate in Hz
- `gain` (string/double, default: 'auto') - Gain setting
- `num_samples` (int, default: 16384) - Samples per read
- `publish_rate` (double, default: 5.0) - Publishing rate in Hz
- `enable_signal_analysis` (bool, default: true) - Enable signal quality analysis

### gps_monitor

Monitoring node that displays GPS signal quality metrics.

**Subscribed Topics:**
- `/rtlsdr/gps_data` (`rtlsdr_ros2/msg/GpsData`) - GPS signal data

## Message: GpsData

The `GpsData` message contains:

- **Header**: Standard ROS2 header with timestamp
- **Device Info**: Device index and serial number
- **Signal Parameters**: Frequency, sample rate, gain, bandwidth
- **IQ Samples**: Raw in-phase and quadrature samples
- **Signal Quality Metrics**:
  - `signal_power_db`: Average signal power in dB
  - `noise_floor_db`: Noise floor in dB
  - `snr_db`: Signal-to-noise ratio in dB
  - `carrier_power_db`: Carrier power in dB
- **Metadata**: Timestamp and sample counter

## Usage Examples

### Basic GPS Capture

```bash
ros2 launch rtlsdr_ros2 gps_launch.py
```

### High Gain Configuration

For weak GPS signals, use higher gain:

```bash
ros2 launch rtlsdr_ros2 gps_launch.py gain:=40.0
```

### Custom Sample Rate

```bash
ros2 launch rtlsdr_ros2 gps_launch.py sample_rate:=2.4e6 num_samples:=32768
```

### Monitor Signal Quality

```bash
# Terminal 1: Run GPS reader
ros2 launch rtlsdr_ros2 gps_launch.py

# Terminal 2: Monitor signal quality
ros2 run rtlsdr_ros2 gps_monitor
```

## Signal Quality Interpretation

- **SNR > 10 dB**: Good GPS signal quality
- **SNR 5-10 dB**: Moderate signal quality
- **SNR < 5 dB**: Weak signal - check antenna and positioning

## GPS Acquisition Tips

1. **Antenna**: Use a GPS patch antenna designed for 1575.42 MHz
2. **Location**: Outdoor location with clear view of sky
3. **Positioning**: Antenna should face upward toward sky
4. **Gain**: Start with auto gain, increase to 20-40 dB if needed
5. **Wait Time**: Allow 30-60 seconds for GPS signal acquisition
6. **Sample Rate**: Use 2.048 MSps or higher for better processing

## Integration with GPS Decoders

The raw IQ samples published by `gps_reader` can be used with GPS decoding software such as:

- **RTKLIB**: Open-source GNSS processing library
- **GNSS-SDR**: Software-defined GPS/GNSS receiver
- **gps-sdr-sim**: GPS signal simulator and decoder

To use with external decoders:

1. Record GPS data:
   ```bash
   ros2 topic echo /rtlsdr/gps_data > gps_data.log
   ```

2. Convert to format required by decoder (typically complex64 binary)

3. Process with decoder software

## Troubleshooting

### No GPS Signal Detected

- **Check antenna**: Ensure GPS patch antenna is connected
- **Location**: Move to outdoor location with clear sky view
- **Gain**: Try increasing gain to 30-40 dB
- **Frequency**: Verify tuned to 1575.42 MHz
- **Wait**: GPS signals are weak - wait 30-60 seconds

### Low SNR

- Increase gain setting
- Improve antenna positioning
- Move to better location (outdoor, clear sky)
- Check antenna connection and quality

### Device Not Found

- Ensure RTL-SDR is connected
- Check with `rtl_test`
- Verify device index (try 0, 1, etc.)

## Configuration Files

- **Launch File**: `launch/gps_launch.py`
- **Config File**: `config/gps_config.yaml`

## Related Documentation

- Main package README: `README.md`
- Quick Start Guide: `QUICKSTART.md`
- Package Summary: `PACKAGE_SUMMARY.txt`

## Notes

- GPS signals are very weak and require proper antenna setup
- RTL-SDR dongles can receive GPS L1, but decoding requires additional software
- This node captures raw GPS signals - actual GPS position decoding requires specialized software
- For best results, use a dedicated GPS patch antenna

