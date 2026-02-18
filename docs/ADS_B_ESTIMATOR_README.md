# ADS-B Receiver Position Estimator

This document describes the math and equations used by the ADS-B–based trike (receiver) position estimator: **robust weighted centroid** in `sdr_adsb_common.py` and **2D constant-velocity Kalman filter** in `rtl_adsb_decoder_node.py`.

---

## 1. Overview

The receiver position is estimated from ADS-B aircraft positions using a two-stage pipeline:

1. **Robust centroid** – weighted average of aircraft positions with outlier rejection
2. **Kalman filter** – temporal smoothing and uncertainty propagation

```
Aircraft (lat, lon, alt, age)  →  estimate_receiver_position()  →  (raw_lat, raw_lon, σ_major)
                                                      ↓
                                        _kf_update_receiver_position()
                                                      ↓
                                        (kf_lat, kf_lon, kf_radius_km)
```

---

## 2. Coordinate Conversion: LLA → ENU

Aircraft positions from ADS-B are in WGS84 (lat, lon, alt). We use a local tangent-plane (ENU) around a reference point \((lat_0, lon_0, alt_0)\).

### 2.1 LLA to ENU (meters)

\[
\begin{aligned}
x_{\text{east}} &= R_E \cos(lat_0) \cdot \Delta\lambda \\
y_{\text{north}} &= R_E \cdot \Delta\phi \\
z_{\text{up}} &= alt - alt_0
\end{aligned}
\]

where \(\Delta\phi = (lat - lat_0)\) in radians, \(\Delta\lambda = (lon - lon_0)\) in radians, and \(R_E \approx 6\,371\,000\) m (Earth radius).

### 2.2 ENU to LLA (inverse)

\[
\begin{aligned}
lat &= lat_0 + \frac{y}{R_E} \cdot \frac{180}{\pi} \\
lon &= lon_0 + \frac{x}{R_E \cos(lat_0)} \cdot \frac{180}{\pi}
\end{aligned}
\]

---

## 3. Robust Weighted Centroid

### 3.1 Active Set

Only aircraft with **age** \(\leq \tau_{\text{active}}\) (default 15 s) are used:

\[
\mathcal{S} = \{ i : t_{\text{now}} - t_i^{\text{update}} \leq \tau_{\text{active}} \}
\]

### 3.2 Per-Aircraft Weights

Two heuristic factors:

**Altitude weight** (low altitude ⇒ stronger geometric constraint):
\[
w_{\text{alt},i} = \frac{1}{\sqrt{alt_i + 500}}
\]

**Age weight** (exponential decay for staleness):
\[
w_{\text{age},i} = \exp\left(-\frac{age_i}{\tau_{\text{decay}}}\right)
\]

with \(\tau_{\text{decay}}\) default 30 s.

**Combined weight**:
\[
w_i = w_{\text{alt},i} \cdot w_{\text{age},i}
\]

### 3.3 Reference Point (Weighted Geographic Mean)

\[
lat_0 = \frac{\sum_i w_i \, lat_i}{\sum_i w_i}, \quad lon_0 = \frac{\sum_i w_i \, lon_i}{\sum_i w_i}
\]

### 3.4 First-Pass Centroid in ENU

Convert each aircraft to \((x_i, y_i)\) in meters, then:

\[
\bar{x} = \frac{\sum_i w_i x_i}{\sum_i w_i}, \quad \bar{y} = \frac{\sum_i w_i y_i}{\sum_i w_i}
\]

### 3.5 Outlier Gate (MAD)

Distance from centroid:
\[
r_i = \sqrt{(x_i - \bar{x})^2 + (y_i - \bar{y})^2}
\]

Median and MAD:
\[
r_{\text{med}} = \text{median}(r_i), \quad \text{MAD} = \text{median}(|r_i - r_{\text{med}}|) + 1
\]

Inlier threshold:
\[
\rho = r_{\text{med}} + 2.5 \cdot \text{MAD}, \quad \text{inlier } i \Leftrightarrow r_i \leq \rho
\]

### 3.6 Second-Pass Centroid and Covariance

Using inliers only:

\[
mx = \frac{\sum_{i \in \mathcal{I}} w_i x_i}{\sum_{i \in \mathcal{I}} w_i}, \quad my = \frac{\sum_{i \in \mathcal{I}} w_i y_i}{\sum_{i \in \mathcal{I}} w_i}
\]

Weighted sample covariance:
\[
\Sigma = \frac{1}{\sum w_i} \sum_{i \in \mathcal{I}} w_i \begin{pmatrix} (x_i - mx)^2 & (x_i-mx)(y_i-my) \\ (x_i-mx)(y_i-my) & (y_i - my)^2 \end{pmatrix}
\]

Eigenvalues of \(\Sigma\) give \(\sigma_{\text{minor}}^2\) and \(\sigma_{\text{major}}^2\). The scalar \(\sigma_{\text{major}}\) (m) is passed to the Kalman filter as measurement uncertainty.

### 3.7 Measurement Sigma for KF

\[
\sigma_{\text{meas}} = \max(100,\, \sigma_{\text{major}} \cdot \kappa_{\text{radio}})
\]

where \(\kappa_{\text{radio}} \in [0.7, 1.3]\) scales by radio quality (high quality ⇒ smaller \(\kappa\), trust measurement more).

---

## 4. Kalman Filter

### 4.1 State

\[
\mathbf{x} = \begin{pmatrix} x \\ y \\ v_x \\ v_y \end{pmatrix} \in \mathbb{R}^4
\]

Position \((x, y)\) and velocity \((v_x, v_y)\) in local ENU (meters).

### 4.2 Process Model (Constant Velocity + White Acceleration)

\[
\mathbf{x}_{k|k-1} = \mathbf{A} \mathbf{x}_{k-1|k-1}
\]

\[
\mathbf{A} = \begin{pmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}
\]

Process noise (discrete white acceleration, \(q = \sigma_a^2\)):

\[
\mathbf{Q} = q \begin{pmatrix}
\frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0 \\
0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} \\
\frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0 \\
0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2
\end{pmatrix}
\]

Default \(\sigma_a = 1.5\) m/s².

**Predict**:
\[
\mathbf{P}_{k|k-1} = \mathbf{A} \mathbf{P}_{k-1|k-1} \mathbf{A}^T + \mathbf{Q}
\]

### 4.3 Measurement Model

\[
\mathbf{z} = \mathbf{H} \mathbf{x} + \mathbf{v}, \quad \mathbf{H} = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{pmatrix}
\]

\[
\mathbf{R} = \begin{pmatrix} r & 0 \\ 0 & r \end{pmatrix}, \quad r = \max(100, \sigma_{\text{meas}})^2
\]

### 4.4 Innovation and NIS

\[
\mathbf{y} = \mathbf{z} - \mathbf{H} \mathbf{x}_{k|k-1}
\]

\[
\mathbf{S} = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}
\]

**Normalized Innovation Squared (NIS)**:
\[
\text{NIS} = \mathbf{y}^T \mathbf{S}^{-1} \mathbf{y}
\]

Under Gaussian assumptions, NIS is \(\chi^2(2)\) distributed. We reject the update if NIS > 9.21 (99% gate).

### 4.5 Update (if accepted)

\[
\mathbf{K} = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}^{-1}
\]

\[
\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K} \mathbf{y}
\]

Joseph form (numerically stable):
\[
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K} \mathbf{H})^T + \mathbf{K} \mathbf{R} \mathbf{K}^T
\]

### 4.6 Output Radius

\[
\text{radius\_km} = \max\left(1,\, \frac{2 \sqrt{\lambda_{\max}(\mathbf{P}_{pos})}}{1000}\right)
\]

where \(\mathbf{P}_{pos}\) is the 2×2 position block of \(\mathbf{P}\); \(\lambda_{\max}\) is the largest eigenvalue (≈ 2σ along major axis).

---

## 5. Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_aircraft` | 4 | Minimum aircraft for centroid |
| `active_window_s` | 15 | Max age (s) for aircraft inclusion |
| `time_decay_s` | 30 | Age decay constant (s) |
| `kf_process_accel_mps2` | 1.5 | Process noise (m/s²) |
| `kf_innovation_gate` | 9.21 | χ²(2, 0.99) NIS gate |

---

## 6. Topics and Output

- **Input**: Aircraft list from ADS-B decoder (lat, lon, alt, icao, last_update)
- **Output**: `~/estimated_position` (JSON) with `kf_lat`, `kf_lon`, `kf_radius_km`, `kf_nis`, `kf_update_accepted`, etc.

See `docs/ADS_B_POSE_ESTIMATION_ANALYSIS.md` for design notes and improvement options.
