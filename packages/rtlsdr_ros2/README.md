# RTL-SDR ROS2 Package

A ROS2 package for interfacing with RTL-SDR (RTL2832U) software defined radio devices. This package allows you to read IQ samples and publish spectrum analysis data to ROS2 topics.

## Features

- 🔊 **Raw IQ Sample Publishing**: Publishes in-phase and quadrature samples for signal processing
- 📊 **Real-time Spectrum Analysis**: FFT-based spectrum analysis with configurable FFT size
- ⚙️ **Configurable Parameters**: Easy configuration via ROS2 parameters or YAML config files
- 🎛️ **Multiple Device Support**: Support for multiple RTL-SDR devices via device index
- 🚀 **High Performance**: Efficient data acquisition and publishing

## Prerequisites

### Hardware
- RTL-SDR compatible USB dongle (RTL2832U based)
- USB port with adequate power supply

### Software
1. **RTL-SDR drivers and libraries**:
```bash
sudo apt-get update
sudo apt-get install rtl-sdr librtlsdr-dev
```

2. **Python dependencies**:
```bash
pip3 install pyrtlsdr numpy
```

3. **ROS2** (Humble, Iron, or later recommended)

### Udev Rules (Linux)
To access RTL-SDR without root privileges, add udev rules:

```bash
sudo bash -c 'cat > /etc/udev/rules.d/20-rtlsdr.rules << EOF
# RTL-SDR
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2832", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", MODE:="0666"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug your RTL-SDR device after adding rules.

## Installation

1. Clone or create this package in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
# Package should already be here
```

2. Install Python dependencies:
```bash
cd ~/ros2_ws/src/rtlsdr_ros2
pip3 install -r requirements.txt
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select rtlsdr_ros2
source install/setup.bash
```

## Usage

### Quick Start

Launch the RTL-SDR reader node with default settings (100 MHz FM radio band):
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py
```

### Launch with Custom Parameters

Tune to a specific frequency (e.g., 433.92 MHz ISM band):
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=433.92e6 gain:=20.0
```

ADS-B aircraft tracking (1090 MHz):
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=1090.0e6 sample_rate:=2.048e6
```

### Using Configuration File

Edit the configuration file at `config/rtlsdr_config.yaml`, then launch:
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py use_config:=true
```

### Running the Node Directly

```bash
ros2 run rtlsdr_ros2 rtlsdr_reader
```

## Topics

### Published Topics

- **`/rtlsdr/signal`** (`rtlsdr_ros2/RtlsdrSignal`)
  - Raw IQ samples from the RTL-SDR device
  - Contains I/Q samples, device info, and tuner settings
  
- **`/rtlsdr/spectrum`** (`rtlsdr_ros2/RtlsdrSpectrum`)
  - FFT spectrum analysis data
  - Contains frequency bins, magnitudes, power in dB, and peak detection

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_index` | int | 0 | RTL-SDR device index (0 for first device) |
| `center_frequency` | double | 100.0e6 | Center frequency in Hz |
| `sample_rate` | double | 2.048e6 | Sample rate in Hz |
| `gain` | string/double | 'auto' | Gain setting ('auto' or value in dB) |
| `num_samples` | int | 1024 | Number of samples per read |
| `publish_rate` | double | 10.0 | Publishing rate in Hz |
| `publish_raw_iq` | bool | true | Enable raw IQ sample publishing |
| `publish_spectrum` | bool | true | Enable spectrum data publishing |
| `fft_size` | int | 1024 | FFT size for spectrum analysis |

## Message Definitions

### RtlsdrSignal.msg
Contains raw IQ samples with metadata:
- Header information
- Device index and serial number
- Tuner settings (frequency, sample rate, gain)
- I/Q sample arrays
- Timestamp

### RtlsdrSpectrum.msg
Contains spectrum analysis data:
- Header information
- Device index and serial number
- Tuner settings
- Frequency bins and power levels
- Peak frequency detection
- Statistics (peak power, average power)

## Examples

### Common Frequency Bands

**FM Radio (88-108 MHz)**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=98.0e6
```

**Weather Satellites (137 MHz)**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=137.5e6 sample_rate:=2.048e6
```

**Air Traffic Control (118-137 MHz)**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=127.5e6
```

**ISM Band 433 MHz**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=433.92e6
```

### Visualizing Data

View raw topics:
```bash
ros2 topic echo /rtlsdr/signal
ros2 topic echo /rtlsdr/spectrum
```

Monitor publishing rate:
```bash
ros2 topic hz /rtlsdr/signal
ros2 topic hz /rtlsdr/spectrum
```

View spectrum peak frequency:
```bash
ros2 topic echo /rtlsdr/spectrum --field peak_frequency
```

## Troubleshooting

### Device Not Found
- Ensure RTL-SDR is properly connected
- Check with `rtl_test` command
- Verify udev rules are configured
- Try unplugging and replugging the device

### Permission Denied
- Add udev rules as described in prerequisites
- Alternatively, run with sudo (not recommended)

### No Data Publishing
- Check that the device is not in use by another application
- Verify parameters are within device capabilities
- Check ROS2 logs: `ros2 run rtlsdr_ros2 rtlsdr_reader --ros-args --log-level debug`

### Poor Signal Quality
- Adjust gain setting (try values between 0-50 dB)
- Use external antenna appropriate for target frequency
- Move away from interference sources

## Performance Tips

1. **Reduce publish rate** if system is overloaded:
   ```bash
   ros2 launch rtlsdr_ros2 rtlsdr_launch.py publish_rate:=5.0
   ```

2. **Disable unnecessary publishers**:
   ```bash
   ros2 launch rtlsdr_ros2 rtlsdr_launch.py publish_raw_iq:=false
   ```

3. **Adjust sample size** based on your needs:
   ```bash
   ros2 launch rtlsdr_ros2 rtlsdr_launch.py num_samples:=512 fft_size:=512
   ```

## Development

### Building from Source
```bash
cd ~/ros2_ws
colcon build --packages-select rtlsdr_ros2 --symlink-install
```

### Running Tests
```bash
cd ~/ros2_ws
colcon test --packages-select rtlsdr_ros2
```

## License

This package is licensed under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## References

- [RTL-SDR Blog](https://www.rtl-sdr.com/)
- [pyrtlsdr Documentation](https://pyrtlsdr.readthedocs.io/)
- [ROS2 Documentation](https://docs.ros.org/)

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review RTL-SDR device specifications
3. Consult ROS2 community resources

## Acknowledgments

This package uses:
- `pyrtlsdr` library for RTL-SDR interfacing
- ROS2 framework for robotics integration
- NumPy for signal processing

