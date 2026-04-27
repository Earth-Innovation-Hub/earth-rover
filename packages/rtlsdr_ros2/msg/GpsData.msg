# GPS Data Message from RTL-SDR
# Contains GPS signal data captured from RTL-SDR device

std_msgs/Header header

# Device information
uint32 device_index        # Index of the RTL-SDR device
string serial_number       # Serial number of the device

# GPS signal parameters
float64 center_frequency   # Center frequency in Hz (typically 1575.42e6 for GPS L1)
float64 sample_rate        # Sample rate in Hz
float64 gain               # Gain in dB
uint32 bandwidth           # Bandwidth in Hz

# GPS signal data (raw IQ samples)
float32[] i_samples        # In-phase (real) samples
float32[] q_samples        # Quadrature (imaginary) samples
uint32 num_samples         # Number of samples in this message

# GPS signal quality metrics
float32 signal_power_db    # Average signal power in dB
float32 noise_floor_db     # Noise floor in dB
float32 snr_db             # Signal-to-noise ratio in dB
float32 carrier_power_db   # Carrier power in dB

# Timestamp information
float64 timestamp_sec      # Timestamp in seconds since epoch
uint64 sample_counter      # Sample counter for tracking continuity

