# ADS-B–Based Trike Pose Estimation: Kalman Filter and Robust Estimator Analysis

This document analyzes the receiver (trike) position estimation pipeline: **robust centroid estimator** in `scripts/sdr_adsb_common.py` and **2D constant-velocity Kalman filter** in `scripts/rtl_sdr/rtl_adsb_decoder_node.py`. It summarizes what is used, what is not, noise assumptions, and concrete simplifications/improvements.

---

## 1. Pipeline Overview

```
Aircraft positions (lat, lon, alt, age) from ADS-B
        ↓
estimate_receiver_position()  [sdr_adsb_common]
  • Active window (age ≤ active_window_s)
  • Weights: 1/√(alt+500) × exp(-age/time_decay_s)
  • Weighted centroid in ENU
  • Outlier gate: median + 2.5×MAD
  • Inlier-weighted centroid + 2×2 covariance → sigma_major, sigma_minor
        ↓
(raw_lat, raw_lon, sigma_major_km, used_icaos)
        ↓
Radio quality for used_icaos → sigma_scale ∈ [0.7, 1.3]
meas_sigma_m = sigma_major_km × sigma_scale × 1000
        ↓
_kf_update_receiver_position(meas_lat, meas_lon, meas_sigma_m)
  • 4-state KF: x = [x, y, vx, vy] in ENU
  • Predict: constant velocity + white acceleration process noise
  • Update: z = [x, y], R = diag(r_var, r_var), r_var = max(100, meas_sigma_m)²
  • Innovation gate: reject update if NIS > kf_innovation_gate (χ²(2, 99%) = 9.21)
        ↓
(kf_lat, kf_lon, kf_radius_km, NIS, update_accepted)
```

- **Estimation rate**: Every 5 s (estimate_timer).
- **Output**: Published on `~/estimated_position` (JSON); VCS can show “KF smoothed” or “Raw robust”.

---

## 2. What Radio / ADS-B Info Is Used

| Data | Used? | Where / How |
|------|--------|--------------|
| Aircraft lat, lon | ✅ | Robust centroid (weighted), then KF measurement |
| Aircraft altitude | ✅ | Weight 1/√(alt_ft + 500) (low alt = stronger constraint) |
| Aircraft last_update (age) | ✅ | Active window (≤ active_window_s); weight exp(-age/time_decay_s) |
| **sigma_major** from centroid | ✅ | Drives KF measurement noise R and radius display |
| **sigma_minor** | ❌ | Returned in JSON but **not used in KF** (R is isotropic) |
| **used_icaos** | ✅ | Radio quality over these ICAOs → sigma_scale |
| Message rate / freshness (per aircraft) | ✅ | Only to compute radio_quality → sigma_scale ∈ [0.7, 1.3] |
| Aircraft **speed, heading, vertical_rate** | ❌ | **Not used** in receiver position estimation |
| RSSI / SNR | ❌ | Not available in dump1090 SBS; fields set to null in radio summary |
| ADS-B **NIC/NUCp** (integrity/accuracy) | ❌ | Not parsed or used |
| Receiver velocity (e.g. from Doppler) | ❌ | KF state is [x,y,vx,vy] but only position is observed; velocity is inferred from dynamics only |

So: **only aircraft position + altitude + age** feed the geometry; **velocity is tracked but not used** for the receiver estimate. **Sigma_minor and ellipse orientation** are not used in the KF.

---

## 3. Noise Assumptions

### 3.1 Robust estimator (`estimate_receiver_position`)

- **Weights** are heuristic, not from a formal measurement model:
  - `w_alt = 1/√(alt_ft + 500)` (low altitude ⇒ higher weight).
  - `w_age = exp(-age_s / time_decay_s)` (fresher ⇒ higher weight).
- **No explicit per-aircraft measurement noise**: the 2×2 covariance is the **sample (weighted) covariance** of aircraft positions in ENU, i.e. spread of the constellation, not a stated uncertainty of the “receiver position” measurement.
- **Outlier rejection**: one-shot MAD (median + 2.5×MAD of distances). No probabilistic (e.g. χ²) gating.
- **sigma_major / sigma_minor**: from eigenvalues of that covariance; used as if they represented uncertainty of the centroid (which is only approximate).

So the robust stage **does not assume a clear measurement noise model**; it produces a single (lat, lon) plus spread (sigma_major, sigma_minor) and confidence heuristics.

### 3.2 Kalman filter

- **State**: 4D, constant velocity in 2D ENU: `[x, y, vx, vy]`.
- **Process noise**: White acceleration; `Q` from `q_acc²` (default `kf_process_accel_mps2 = 1.5 m/s²`). Standard discrete-time model for position/velocity.
- **Measurement**: `z = [x, y]` in ENU; **R = diag(r_var, r_var)** with `r_var = max(100, meas_sigma_m)²` (meters). So:
  - **Isotropic** (same variance in x and y).
  - **sigma_minor and ellipse orientation** are ignored.
- **Innovation gating**: Update applied only if NIS ≤ 9.21 (χ²(2, 0.99)); otherwise prediction-only step.
- **dt**: Clamped to [0.1, 30] s between updates.

Assumptions: **measurement errors in x and y are independent, equal variance, and fully characterized by one number (meas_sigma_m)**. Process noise is fixed (no adaptation from NIS or reference GPS).

---

## 4. What Can Be Simplified

1. **Single sigma for KF**
   - Today only `sigma_major_km` is turned into `meas_sigma_m`; `sigma_minor_km` is unused. Either:
     - **Simplify**: Document that “we use a single conservative sigma” and drop sigma_minor from the KF path (still keep in JSON for UI), or
     - **Improve**: Use anisotropic R (or full 2×2 R) from sigma_major, sigma_minor and ellipse angle so the KF trusts the estimate more in the direction of smaller spread.

2. **Radio quality scaling**
   - `sigma_scale = max(0.7, min(1.3, 1.30 - 0.60 * radio_quality_mean))` is a simple linear map. Could be replaced by a single tunable “measurement noise scale” parameter if you want less moving parts, or kept and documented as “high radio quality ⇒ trust measurement more (smaller R).”

3. **Two-stage vs one-stage**
   - Current: robust centroid (handles outliers) → one scalar “measurement” every 5 s → KF. Simpler alternative: **one EKF that takes multiple aircraft positions as observations** with per-aircraft R (e.g. from altitude/range), and one robust gating step. That would be a larger refactor; the current two-stage design is a reasonable **simplification** (robust step hides outliers; KF only sees one pseudo-measurement).

4. **Parameters**
   - `estimator_active_window_s`, `estimator_time_decay_s`, `kf_process_accel_mps2`, `kf_innovation_gate` are all tunable; consider moving to a small config block and documenting default vs “aggressive” vs “smooth” presets.

---

## 5. What Can Be Improved

### 5.1 Use more radio/ADS-B information

- **Aircraft velocity (speed, heading)**  
  Use it to **propagate aircraft positions to a common time** (e.g. “now”) before computing the centroid. Right now, positions at different `last_update` times are mixed; propagating to a reference time would make the geometry more consistent and could reduce bias.

- **sigma_minor and orientation**  
  Pass the full 2×2 covariance (or sigma_major, sigma_minor, angle) into the KF and set **R** accordingly (e.g. rotate from principal axes). That uses the existing “radio info” (constellation shape) more optimally than a single isotropic sigma.

- **ADS-B quality (NIC/NUCp)**  
  If the decoder (pyModeS/dump1090) exposes NIC/NUCp, use them to set **per-aircraft** position uncertainty and then either:
  - weight the centroid by inverse variance, or
  - feed multiple aircraft positions as separate measurements into one filter with per-aircraft R.

- **RSSI/SNR (when available)**  
  In IQ or other backends that provide per-frame RSSI/SNR, use them to weight or to inflate R for weak signals.

### 5.2 Noise and consistency

- **Explicit measurement noise in the robust stage**  
  Model per-aircraft uncertainty (e.g. from altitude/range and/or NIC), then compute a **weighted least-squares** centroid with proper variances and derive sigma_major/sigma_minor from the covariance of the estimate (not just sample covariance of inputs). That would make the “measurement” passed to the KF better justified.

- **Anisotropic R in the KF**  
  Use `R = R_2x2` from sigma_major, sigma_minor and angle so the KF’s noise assumptions match the geometry.

- **Reference GPS**  
  When `/mavros/global_position/raw/fix` is available, **fuse it as an additional measurement** in the KF (or a separate observation) instead of only using it for error logging. That would improve accuracy when GPS is good and make the filter “ADS-B + GPS” rather than “ADS-B only with GPS for diagnostics.”

### 5.3 Adaptivity and tuning

- **Process noise from NIS**  
  If NIS is persistently high, increase `Q` (or `kf_process_accel_mps2`); if NIS is low, slightly decrease. This keeps the filter consistent without manual tuning.

- **Innovation gate**  
  Optionally use a **sequence test** (e.g. reject only if NIS is large for several steps) to avoid single bad measurements from killing the update, or use a slightly higher gate (e.g. 99.5%) if reject ratio is too high in the field.

- **NIS / reject ratio in UI**  
  You already publish `kf_nis`, `kf_update_accepted`, `kf_reject_ratio`; the VCS could show a small “KF health” indicator (e.g. green/yellow/red from NIS median and reject ratio) so operators can see when the filter is confident vs struggling.

---

## 6. Summary Table

| Aspect | Current | Simplified option | Improvement option |
|--------|--------|--------------------|--------------------|
| Measurement to KF | Single (lat, lon), isotropic R from sigma_major | Keep; document “one conservative sigma” | Anisotropic R from sigma_major/minor + angle |
| Aircraft velocity | Unused | — | Propagate positions to common time; optional velocity constraint |
| sigma_minor | In JSON only | Drop from KF path in code comments | Use in R (ellipse) |
| Radio quality | sigma_scale 0.7–1.3 | One global scale param | Keep; optionally feed into per-aircraft R in a multi-obs EKF |
| NIC/NUCp | Unused | — | Use when decoder provides them (per-aircraft weight or R) |
| Robust stage noise | Heuristic weights, sample cov | — | Per-aircraft uncertainty model → WLS centroid |
| Reference GPS | Diagnostics only | — | Fuse as second measurement in KF |
| Process noise | Fixed 1.5 m/s² | — | Adapt from NIS or from GPS velocity when available |
| Innovation gate | χ²(2, 99%) | — | Optional sequence test or slightly higher gate |

---

## 7. Files and Symbols

- **Robust estimator**: `scripts/sdr_adsb_common.py` → `estimate_receiver_position()`
- **KF and integration**: `scripts/rtl_sdr/rtl_adsb_decoder_node.py` → `_estimate_position_callback()`, `_kf_update_receiver_position()`, `_radio_quality_for_icaos()`
- **Parameters**: `kf_process_accel_mps2`, `kf_innovation_gate`, `estimator_active_window_s`, `estimator_time_decay_s` (node); `min_aircraft`, `time_decay_s`, `active_window_s` (estimator)
- **VCS**: `vehicle_control_station/.../vcs-app.js` (useKalmanEstimate, radius from kf_radius_km or sigma_major)

This gives a direct map from “what radio info is used” and “what noise assumptions are made” to concrete simplifications and improvements for the trike ADS-B pose estimation pipeline.
