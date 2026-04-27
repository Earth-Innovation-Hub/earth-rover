# RTL-SDR Spectrum Data Message
# This message contains FFT spectrum analysis of RTL-SDR signal

std_msgs/Header header

# Device information
uint32 device_index        # Index of the RTL-SDR device
string serial_number       # Serial number of the device

# Tuner settings
float64 center_frequency   # Center frequency in Hz
float64 sample_rate        # Sample rate in Hz
float64 bandwidth          # Bandwidth in Hz

# Spectrum data
float32[] frequencies      # Frequency bins in Hz
float32[] magnitudes       # Magnitude (power) at each frequency bin
float32[] power_db         # Power in dB at each frequency bin
uint32 fft_size            # FFT size used for spectrum analysis

# Statistics
float32 peak_frequency     # Frequency of peak signal in Hz
float32 peak_power_db      # Power of peak signal in dB
float32 average_power_db   # Average power across spectrum in dB

