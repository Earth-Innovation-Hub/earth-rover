# RTL-SDR Signal Data Message
# This message contains the IQ samples captured from RTL-SDR device

std_msgs/Header header

# Device information
uint32 device_index        # Index of the RTL-SDR device
string serial_number       # Serial number of the device

# Tuner settings
float64 center_frequency   # Center frequency in Hz
float64 sample_rate        # Sample rate in Hz
float64 gain               # Gain in dB (0 for auto gain)
uint32 bandwidth           # Bandwidth in Hz

# Signal data
float32[] i_samples        # In-phase (real) samples
float32[] q_samples        # Quadrature (imaginary) samples
uint32 num_samples         # Number of samples in this message

# Timestamp information
float64 timestamp_sec      # Timestamp in seconds since epoch

