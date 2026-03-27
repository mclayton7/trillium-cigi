# Realism Round 2 — Protocol Fidelity, Sensor Simulation, Physics Refinements

Date: 2026-03-26
Status: Draft

## Overview

Ten improvements across three categories that bring the simulator closer to real EO/IR gimbal behavior. All changes are backward-compatible via config defaults that preserve existing behavior where noted.

---

## 1. Protocol Fidelity

### 1A. Fix gate_x_pos / gate_y_pos to CIGI v3.3 spec

**Problem:** `gate_x_pos`/`gate_y_pos` in `SensorExtendedResponse` currently carry gimbal pan/tilt angles (degrees). The CIGI v3.3 spec defines these as the image-plane tracking gate centroid position, normalized to the sensor FOV.

**Change:**
- In `to_sensor_extended_response()`: set `gate_x_pos` and `gate_y_pos` to the tracking gate centroid position in the image plane, normalized to ±1.0 within the current FOV.
  - Track mode: use `track_target[0]` and `track_target[1]` directly (already fractional offsets).
  - Other modes (Position, Rate, Geopoint, Disabled): gate position is (0.0, 0.0) — bore-sighted.
- In `to_orion.rs` (`sensor_response_to_telemetry`): stop extracting pan/tilt from `gate_x_pos`/`gate_y_pos`. Pan/tilt are already populated via `to_telemetry()` on the Orion TCP path.
- Document the change in CLAUDE.md (remove the old "gate_x/y carry pan/tilt" convention).

**Files:** `src/simulator/mod.rs` (to_sensor_extended_response), `src/convert/to_orion.rs`, CLAUDE.md

### 1B. Forward camera selection and zoom in orion_cmd_to_sensor_control

**Problem:** `orion_cmd_to_sensor_control()` hardcodes `sensor_id=0` and never sets `ac_coupling` (zoom). The CIGI scene generator receives no camera or zoom information.

**Change:**
- Map `cmd.camera_index` → `sc.sensor_id` (0, 1, or 2).
- Map `cmd.zoom_level` → `sc.ac_coupling` (0.0–1.0) in Position/Rate modes.
- In Geopoint mode, `ac_coupling` and `noise` continue to carry lat/lon (existing behavior).
- Guard: if `OrionCmdPacket` lacks `camera_index` or `zoom_level` fields (check generated types), add them to the simulator state and populate from `SensorControl.sensor_id` / `SensorControl.ac_coupling` during `apply_sensor_control`.

**Files:** `src/convert/to_cigi.rs`, possibly `src/simulator/mod.rs`

### 1C. Populate IgControl timestamps and frame counter tracking

**Problem:** `make_ig_control()` always sets `last_rcvd_ig_frame_ctr=0`, `timestamp_valid=false`, `timestamp=0.0`. The IG cannot detect dropped host frames or perform dead-reckoning.

**Change:**
- Track the last `StartOfFrame.frame_ctr` received from the IG in the main loop state.
- Pass it to `make_ig_control()` as `last_rcvd_ig_frame_ctr`.
- Set `timestamp_valid=true` and populate `timestamp` from the elapsed time since startup (seconds as f32, matching CIGI v3.3 timestamp format).
- `make_ig_control` gains two new parameters: `last_ig_frame: u32` and `timestamp: f32`.

**Files:** `src/main.rs`, `src/convert/to_cigi.rs`

### 1D. Compute gate_x_size / gate_y_size from zoom

**Problem:** Gate sizes are hardcoded to 20 pixels regardless of zoom, camera, or tracking state.

**Change:**
- Add `track_gate_size_deg: f32` to Config (default 1.0 degree — angular extent of the tracking gate).
- Compute gate size in pixels: `gate_size_px = track_gate_size_deg / current_hfov_deg * SENSOR_RESOLUTION` where `SENSOR_RESOLUTION = 640` (constant, reasonable default for EO/IR).
- Set `gate_x_size` and `gate_y_size` to this computed value in `to_sensor_extended_response()`.
- When not tracking (Disabled, Position, Rate, Geopoint), gate size represents a reference reticle size — use the same formula.

**Config keys:** `track_gate_size_deg` (default 1.0)

**Files:** `src/config.rs`, `src/simulator/mod.rs`

---

## 2. Sensor Simulation

### 2A. Laser rangefinder simulation

**Problem:** No range measurement. `range_source` always defaults to `RangeSrcNone` in telemetry. Real Orion gimbals have integrated laser rangefinders.

**Change:**
- Add `laser_enabled: bool` field to `GimbalSimulator` (default `true`).
- Each tick, when the look-point is valid (not None), compute slant range: `slant_range = euclidean_distance(platform_ecef, look_point_ecef)`. This requires `geo::geodetic_to_ecef` for both points.
- Store `slant_range_m: f64` on the simulator.
- In `to_telemetry()`:
  - If `laser_enabled` and `slant_range_m > 0.0` and `slant_range_m <= laser_max_range_m`: set `range_source = RangeSrcLaser`.
  - Otherwise: `range_source = RangeSrcNone`.
- Add `inject_laser_fault()` / `clear_laser_fault()` to `FaultState`. When active, force `range_source = RangeSrcNone` and `slant_range = 0.0`.
- Add `laser_max_range_m: f64` to Config (default 20000.0).

**Config keys:** `laser_max_range_m` (default 20000.0)

**Files:** `src/simulator/mod.rs`, `src/faults.rs`, `src/config.rs`, `src/geo.rs` (expose `geodetic_to_ecef` as pub if not already)

### 2B. Dynamic track confidence and target size

**Problem:** `PrimaryTrackData` has hardcoded `size=0.05` and `confidence=0.9`. Real trackers vary these with range, zoom, and target offset.

**Change:**

**Target size** — angular size of the tracked target in the image:
- `angular_size = track_target_size_m / slant_range_m` (radians).
- Normalized to FOV: `size = angular_size / current_hfov_rad`.
- Clamped to [0.01, 0.5].
- Requires slant range from 2A.
- Add `track_target_size_m: f32` to Config (default 2.0 — person/small vehicle).

**Confidence** — likelihood of track hold:
- `offset_fraction = track_offset_magnitude / track_loss_threshold` (how close to FOV edge).
- `size_factor = (angular_size / min_resolvable_rad).min(1.0)` where `min_resolvable_rad = current_hfov_rad * 0.005` (half a percent of FOV — target too small to resolve).
- `confidence = 0.95 * (1.0 - offset_fraction.powi(2)) * size_factor`.
- Track loss also triggers when `confidence < 0.3` (in addition to existing FOV-edge threshold).

**Config keys:** `track_target_size_m` (default 2.0)

**Files:** `src/simulator/mod.rs`, `src/config.rs`

### 2C. Sensor status refinement

**Problem:** Position, Rate, and Geopoint all map to CIGI sensor_status 0 (Searching). The CIGI host gets no distinction between settled pointing and active slewing.

**Change in `orion_mode_to_sensor_status()`:**
- Position/Geopoint with error < 0.01 rad on both axes → status 0 (Tracking/Locked)
- Position/Geopoint while slewing (error >= 0.01 rad) → status 2 (Slewing — CIGI "Searching" semantics)
- Rate mode → status 2 (Slewing)
- Track mode (active) → status 1 (Tracking)
- Track mode (coasting/lost) → status 3 (Breaklock)
- Disabled/Fault → status 3 (Breaklock)

This requires the status mapper to know the current pointing error, so it gains `pan_error: f32` and `tilt_error: f32` parameters (or a `settled: bool` flag computed by the simulator).

**Files:** `src/convert/to_cigi.rs`, `src/simulator/mod.rs`

---

## 3. Physics Refinements

### 3A. Cross-axis gyroscopic coupling

**Problem:** Pan and tilt axes are fully independent. Real gimbals experience gyroscopic coupling torques at high slew rates.

**Change:**
- In `tick()`, after computing pan_rate and tilt_rate for the current tick, apply coupling:
  ```
  pan_coupling  = gyro_coupling_factor * tilt_rate * pan_rate * dt
  tilt_coupling = gyro_coupling_factor * pan_rate * tilt_rate * dt
  ```
  (Symmetrical first-order model — the coupling torque is proportional to the product of the two rates.)
- Add the coupling as a small angular offset: `pan += pan_coupling`, `tilt += tilt_coupling`.
- Add `gyro_coupling_factor: f32` to Config (default 0.0 — disabled for the Orion's well-compensated direct-drive motors). Non-zero values (e.g., 0.02) model imperfect compensation.

**Config keys:** `gyro_coupling_factor` (default 0.0)

**Files:** `src/config.rs`, `src/simulator/mod.rs`

### 3B. Coordinated geopoint slew

**Problem:** In geopoint mode, pan and tilt slew independently to their targets. This produces an L-shaped LOS trajectory instead of a straight line. Real gimbals coordinate axes so both arrive simultaneously.

**Change:**
- In the `OrionModeGeopoint` branch of `tick()`, after computing `target_pan` and `target_tilt` from `inverse_geopoint`:
  1. Compute per-axis error: `pan_err = |target_pan - pan|`, `tilt_err = |target_tilt - tilt|`.
  2. Compute per-axis time at full rate: `pan_time = pan_err / effective_slew_rate`, `tilt_time = tilt_err / effective_slew_rate`.
  3. Take `max_time = max(pan_time, tilt_time)`.
  4. Compute coordinated rates: `pan_coord_rate = pan_err / max_time`, `tilt_coord_rate = tilt_err / max_time` (capped at `effective_slew_rate`).
  5. Pass per-axis rates to `tick_axis_trap` instead of the global `effective_slew_rate`.
- Guard: if either error is < 0.001 rad (essentially settled), skip coordination and use the global rate (avoid division by near-zero).
- `tick_axis_trap` gains an optional `max_rate` parameter override, or the geopoint branch calls it with the coordinated rate.

**Files:** `src/simulator/mod.rs`

### 3C. Multi-frequency jitter

**Problem:** Single 10 Hz sinusoid + white noise. Real gimbals have multiple structural resonances (typically 10-50 Hz for the payload, 40-200 Hz for the gimbal head).

**Change:**
- Add a second sinusoidal component with independent frequency and amplitude.
- Config keys: `jitter_freq_2_hz` (default 47.0 Hz), `jitter_amplitude_2_deg` (default 0.01 deg).
- In `tick()`, compute a second sinusoid at the second frequency with its own phase accumulator (`jitter_phase_2`). Sum both sinusoids before adding white noise.
- The two frequencies being non-harmonic (10 Hz and 47 Hz) produces beat patterns that look realistic on a spectrum analyzer.
- Add `jitter_phase_2: f32` to the simulator state.

**Config keys:** `jitter_freq_2_hz` (default 47.0), `jitter_amplitude_2_deg` (default 0.01)

**Files:** `src/config.rs`, `src/simulator/mod.rs`

---

## Dependency Order

Tasks are mostly independent but some have data dependencies:

- **2A (laser rangefinder)** must precede **2B (dynamic track size)** — track size depends on slant range.
- **1A (gate fix)** should precede **1D (gate sizing)** — gate sizing builds on the corrected gate semantics.
- **1B (camera/zoom forwarding)** is independent.
- **1C (timestamps)** is independent.
- **2C (sensor status)** is independent.
- **3A, 3B, 3C** are all independent of each other and of the sensor/protocol tasks.

Suggested execution order:
1. 1A (gate fix) — foundational protocol correction
2. 1B (camera/zoom forward)
3. 1C (timestamps)
4. 2A (laser rangefinder)
5. 1D (gate sizing) — depends on 1A
6. 2B (track confidence) — depends on 2A
7. 2C (sensor status)
8. 3A (cross-axis coupling)
9. 3B (coordinated geopoint slew)
10. 3C (multi-frequency jitter)

## New Config Keys Summary

| Key | Type | Default | Category |
|-----|------|---------|----------|
| `track_gate_size_deg` | f32 | 1.0 | Sensor |
| `laser_max_range_m` | f64 | 20000.0 | Sensor |
| `track_target_size_m` | f32 | 2.0 | Sensor |
| `gyro_coupling_factor` | f32 | 0.0 | Physics |
| `jitter_freq_2_hz` | f32 | 47.0 | Vibration |
| `jitter_amplitude_2_deg` | f32 | 0.01 | Vibration |
