# Realism Roadmap: CIGI Trillium Gimbal Simulator

## Context

This is a CIGI v3.3 UDP software simulator for the Trillium Orion EO/IR gimbal family. It lets CIGI hosts (image generators, ground stations, test tools) drive a virtual gimbal without physical hardware. Currently the simulator models only a single simplified behavior: linear slew toward a target pan/tilt at a fixed max rate (60°/s). All other Orion features — tracking, geolocation, vibration, limits, fault injection, camera modeling — are either stubs or absent.

This roadmap is organized into phases by fidelity impact vs. implementation complexity.

---

## Phase 1: Gimbal Kinematics (High Impact, Low Complexity)

### 1.1 Mechanical Angle Limits
**File:** `src/simulator/mod.rs`

Enforce hardware-accurate joint limits instead of allowing ±180° pan and ±180° tilt:
- Pan: typically ±170° (depends on Orion model)
- Tilt: typically -110° to +30° (nose-down bias)
- Clamp `target_pan` and `target_tilt` on command receipt
- Emit a status flag when a limit is hit (maps to `GeolocateTelemetryCorePacket` status bits)

### 1.2 Acceleration / Deceleration Profiles
**File:** `src/simulator/mod.rs`

Replace instantaneous slew with a trapezoidal or S-curve velocity profile:
- Add `pan_rate: f32` and `tilt_rate: f32` fields to `GimbalSimulator`
- Apply configurable `MAX_ACCEL` (e.g., 300 °/s²) per tick
- Decelerate as the gimbal approaches the target (point-to-point timing accuracy)
- Makes the simulator observable via rate-of-change in telemetry

### 1.3 Rate Mode Behavior
**File:** `src/simulator/mod.rs`, `src/convert/to_orion.rs`

`OrionModeRate` currently does nothing. Map CIGI sensor gain/level to a continuous rate command (°/s) instead of a position target. This is the second most common operational mode after Position.

---

## Phase 2: Stabilization & Platform Dynamics (High Impact, Medium Complexity)

### 2.1 Gimbal Vibration / Jitter
**File:** `src/simulator/mod.rs`

Add stochastic perturbation to simulate mechanical vibration and platform disturbance:
- Low-frequency sinusoidal oscillation (simulates airframe vibration, e.g., 5–20 Hz)
- White noise floor (simulates bearing/motor noise, e.g., ±0.01°)
- Amplitude proportional to slew rate (active slewing → more jitter)
- Output appears in pan/tilt telemetry and line-of-sight fields

### 2.2 Platform Motion Compensation
**File:** `src/simulator/mod.rs`, `src/convert/to_cigi.rs`

The `EntityControl` packet delivers platform lat/lon/alt but it's only echoed back. Extend it to:
- Accept platform roll/pitch/yaw (aircraft attitude) from CIGI
- Compute inertially-stabilized gimbal angles relative to NED frame
- Populate `gimbal_quat` in `GeolocateTelemetryCorePacket` correctly (currently identity or zero)
- This enables geolocation math to work correctly in Phase 3

### 2.3 INS/IMU Simulation
**File:** `src/simulator/mod.rs`

Populate the INS-related telemetry fields:
- `ins_quat` in `GeolocateTelemetryCorePacket` (platform attitude quaternion)
- `vel_ned` (simulated platform velocity from EntityControl delta)
- Enables testing of INS-dependent ground station features

---

## Phase 3: Geolocation (High Impact, Medium-High Complexity)

### 3.1 WGS84 Line-of-Sight Projection
**File:** new `src/geo.rs`, used by `src/simulator/mod.rs`

Given platform lat/lon/alt and gimbal pan/tilt, compute where the gimbal is pointing on the Earth's surface:
- Convert geodetic position → ECEF
- Compute LOS vector in NED from gimbal angles + platform attitude
- Rotate NED LOS to ECEF
- Ray-cast against WGS84 ellipsoid (or flat-Earth approximation for initial implementation)
- Populate `los_ecef` and `GeolocateTelemetryCorePacket` target lat/lon/alt fields

This is the most used feature by real Orion customers — geolocation accuracy is how the system is evaluated.

### 3.2 Geopoint Mode
**File:** `src/simulator/mod.rs`, `src/convert/to_orion.rs`

`OrionModeGeopoint` (0x60) aims the gimbal at a specified ground coordinate. Implement the inverse:
- Accept target lat/lon/alt from CIGI
- Solve the inverse geolocation problem to determine required pan/tilt
- Set those as the position target and enter position tracking

---

## Phase 4: Track Mode (Medium Impact, High Complexity)

### 4.1 Simulated Object Tracking
**File:** `src/simulator/mod.rs`

`OrionModeTrack` (0x31) currently does nothing. A minimal implementation:
- Accept a "track target" offset from scene center (pixel coordinates via CIGI)
- Apply a simulated proportional controller to drive pan/tilt toward the virtual target
- Populate `PrimaryTrackData` fields in telemetry (track status, residual error, target centroid)
- Simulate track loss after large slew or if target exits FOV

---

## Phase 5: Camera Model (Medium Impact, Medium Complexity)

### 5.1 FOV / Zoom Simulation
**File:** `src/simulator/mod.rs`, `src/convert/to_cigi.rs`

Replace hard-coded HFOV=30°/VFOV=22.5° with a zoom model:
- Define a zoom level or focal length command input
- Map zoom to HFOV/VFOV via a configurable lens model (e.g., min/max FOV)
- Report updated HFOV/VFOV per frame in `GeolocateTelemetryCorePacket`

### 5.2 Multi-Camera / Sensor Selection
**File:** `src/simulator/mod.rs`

The Orion supports EO + IR + narrow/wide camera banks. Simulate:
- Camera index field in telemetry (`camera_index`)
- Per-camera FOV table (narrow, wide, FLIR)
- Camera switch latency (brief blackout or frame drop)

---

## Phase 6: Fault Injection & Diagnostics (Lower Priority, Medium Complexity)

### 6.1 Fault Simulation
**File:** `src/simulator/mod.rs`, new `src/faults.rs`

Use the already-generated `OrionFaultPacket`, `OrionDiagnosticsPacket`, and `OrionPerformancePacket` types to:
- Emit periodic diagnostics (temperature, voltage, vibration RMS)
- Support a fault injection API (e.g., trigger GPS loss, motor fault, IMU dropout)
- Test ground station fault handling and alerting

---

## Phase 7: Configuration & Infrastructure (Enabling, Low Complexity)

### 7.1 Runtime Configuration File
**File:** new `config.toml` + parser in `main.rs`

Externalize all hard-coded constants:
- Network port (currently 8008)
- Max slew rate, acceleration limit, angle limits
- Camera FOV table
- Vibration noise parameters
- Platform model selection

### 7.2 Orion TCP Bridge Mode
**File:** new `src/bridge.rs`

Add an optional `--gimbal-ip <ip>` flag that routes commands to a real Orion gimbal over TCP (Orion uses TCP port 8008 for actual hardware). This makes the simulator a protocol proxy for hardware-in-the-loop testing.

---

## Verification Strategy

Each phase can be tested by:
1. Running the simulator and pointing a CIGI client (or `nc`/`socat`) at UDP port 8008
2. Sending `SensorControl` packets and verifying `SensorExtendedResponse` telemetry
3. Existing unit tests in `src/simulator/mod.rs` cover slew rate; extend them for each new behavior
4. For geolocation: compare computed target lat/lon against known ground truth scenarios

---

## Key Files

| File | Role |
|------|------|
| `src/simulator/mod.rs` | Core state machine — most changes happen here |
| `src/main.rs` | 50 Hz event loop, UDP I/O |
| `src/convert/to_orion.rs` | CIGI command → Orion packet mapping |
| `src/convert/to_cigi.rs` | Orion telemetry → CIGI response |
| `OrionPublicProtocol.xml` | Protocol spec; generated types provide all wire formats |
| `build.rs` | Code generator — don't modify unless adding XML support |
