// Gimbal state machine — simulates Trillium Orion gimbal dynamics.
//
// Phases implemented:
//   1.1  Mechanical angle limits (±170° pan, −110° to +30° tilt)
//   1.2  Trapezoidal acceleration/deceleration profile
//   1.3  Rate mode (OrionModeRate)
//   2.1  Vibration / jitter (sinusoidal + LCG white noise)
//   2.2  Platform motion compensation (gimbal_quat from attitude)
//   2.3  INS / IMU simulation (ins_quat, vel_ned)
//   3.1  WGS84 LOS projection (los_ecef, look-point lat/lon/alt)
//   3.2  Geopoint mode (OrionModeGeopoint — inverse geolocation)
//   4.1  Track mode (proportional controller + track loss)
//   5.1  FOV / zoom simulation
//   5.2  Multi-camera selection with switch latency
//   6.1  Fault state integration

use std::f32::consts::PI;

use sim_core::cigi::messages::{EntityControl, SensorControl, SensorExtendedResponse, StartOfFrame};
use crate::config::Config;
use crate::faults::{FaultState, lcg_noise_f32};
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode, PrimaryTrackData};
use sim_core::geo;

// ─────────────────────────────────────────── constants (kept for tests) ──

#[allow(dead_code)]
/// Default maximum slew rate (rad/s) = 60 °/s.  Matches Config::default().
pub const MAX_SLEW_RATE: f32 = std::f32::consts::FRAC_PI_3; // 60°/s = π/3

/// Camera switch blackout duration (frames at 50 Hz).
const CAMERA_SWITCH_FRAMES: u8 = 10; // 200 ms

// ─────────────────────────────────────────── GimbalSimulator ──

/// Full simulated Trillium Orion gimbal state.
#[derive(Debug, Clone)]
pub struct GimbalSimulator {
    // ── Kinematics ──────────────────────────────────────────────────
    /// Current inertial pan angle (rad, azimuth from north).
    pub pan: f32,
    /// Current inertial tilt / depression angle (rad, + down).
    pub tilt: f32,
    /// Current angular velocity: pan axis (rad/s).
    pub pan_rate: f32,
    /// Current angular velocity: tilt axis (rad/s).
    pub tilt_rate: f32,
    /// Commanded position target: pan (rad).
    pub target_pan: f32,
    /// Commanded position target: tilt (rad).
    pub target_tilt: f32,
    /// Commanded rate: pan (rad/s), used in OrionModeRate.
    pub pan_rate_cmd: f32,
    /// Commanded rate: tilt (rad/s), used in OrionModeRate.
    pub tilt_rate_cmd: f32,

    // ── Limits ──────────────────────────────────────────────────────
    pub at_pan_limit: bool,
    pub at_tilt_limit: bool,

    // ── Mode ────────────────────────────────────────────────────────
    pub mode: OrionMode,

    // ── Camera / FOV ────────────────────────────────────────────────
    pub hfov: f32,               // radians
    pub vfov: f32,               // radians
    pub zoom_level: f32,         // 0.0 = wide, 1.0 = narrow
    pub camera_index: i8,        // 0 = EO wide, 1 = EO narrow, 2 = IR
    camera_switch_remaining: u8, // frames of blackout remaining after camera switch

    // ── Platform ────────────────────────────────────────────────────
    pub pos_lat: f64,           // radians
    pub pos_lon: f64,           // radians
    pub pos_alt: f64,           // metres
    prev_pos_lat: f64,
    prev_pos_lon: f64,
    prev_pos_alt: f64,
    pub platform_roll: f32,    // radians
    pub platform_pitch: f32,   // radians
    pub platform_yaw: f32,     // radians (heading)
    pub vel_ned: [f32; 3],     // m/s, NED

    // ── Geolocation ─────────────────────────────────────────────────
    /// Computed look-point (where LOS hits Earth), radians / metres.
    pub look_lat: f64,
    pub look_lon: f64,
    pub look_alt: f64,
    /// Geopoint commanded target (for OrionModeGeopoint).
    pub geopoint_lat: f64,
    pub geopoint_lon: f64,
    pub geopoint_alt: f64,

    // ── Track mode ──────────────────────────────────────────────────
    /// Track target offset from image centre (−0.5 to +0.5, fractional).
    pub track_target: [f32; 2],
    pub track_active: bool,
    /// Previous frame track offsets for derivative calculation.
    prev_track_x: f32,
    prev_track_y: f32,

    // ── Vibration ────────────────────────────────────────────────────
    jitter_phase: f32, // primary resonance accumulated cycles
    jitter_phase_2: f32, // second resonance accumulated cycles
    noise_seed: u32,   // LCG state
    /// Instantaneous jitter added to pan for this frame's telemetry.
    pan_jitter: f32,
    tilt_jitter: f32,
    /// Jitter-perturbed pan/tilt (clean angle + jitter).  Used for
    /// look-point computation and telemetry so the reported LOS reflects
    /// the physical vibration of the sensor.
    pub pan_jittered: f32,
    pub tilt_jittered: f32,

    // ── Laser rangefinder ───────────────────────────────────────────
    /// Laser rangefinder enabled (default true). Disable via `laser_fault`.
    pub laser_enabled: bool,
    /// Most recent computed slant range (metres). 0.0 when no valid LOS hit.
    pub slant_range_m: f64,

    // ── Settling ────────────────────────────────────────────────────
    /// True when in Position/Geopoint mode and both axes are within
    /// 0.01 rad of target. Fed to CIGI `sensor_status` for Locked vs Slewing.
    pub settled: bool,

    // ── Diagnostics / faults ────────────────────────────────────────
    pub faults: FaultState,

    // ── Timing ──────────────────────────────────────────────────────
    pub frame_ctr: u32,
    pub system_time_ms: u32,

    // ── CIGI IG state ───────────────────────────────────────────────
    pub ig_mode: u8,
    pub host_frame_ctr: u32,

    // ── Configuration ───────────────────────────────────────────────
    pub config: Config,
}

impl Default for GimbalSimulator {
    fn default() -> Self {
        let cfg = Config::default();
        let (hfov, vfov) = cfg.fov_at_zoom_for_camera(0, 0.0);
        Self {
            pan: 0.0,
            tilt: 0.0,
            pan_rate: 0.0,
            tilt_rate: 0.0,
            target_pan: 0.0,
            target_tilt: 0.0,
            pan_rate_cmd: 0.0,
            tilt_rate_cmd: 0.0,
            at_pan_limit: false,
            at_tilt_limit: false,
            mode: OrionMode::OrionModeDisabled,
            hfov,
            vfov,
            zoom_level: 0.0,
            camera_index: 0,
            camera_switch_remaining: 0,
            pos_lat: 0.0,
            pos_lon: 0.0,
            pos_alt: 0.0,
            prev_pos_lat: 0.0,
            prev_pos_lon: 0.0,
            prev_pos_alt: 0.0,
            platform_roll: 0.0,
            platform_pitch: 0.0,
            platform_yaw: 0.0,
            vel_ned: [0.0; 3],
            look_lat: 0.0,
            look_lon: 0.0,
            look_alt: 0.0,
            geopoint_lat: 0.0,
            geopoint_lon: 0.0,
            geopoint_alt: 0.0,
            track_target: [0.0; 2],
            track_active: false,
            prev_track_x: 0.0,
            prev_track_y: 0.0,
            jitter_phase: 0.0,
            jitter_phase_2: 0.0,
            noise_seed: 0xDEAD_BEEF,
            pan_jitter: 0.0,
            tilt_jitter: 0.0,
            pan_jittered: 0.0,
            tilt_jittered: 0.0,
            laser_enabled: true,
            slant_range_m: 0.0,
            settled: false,
            faults: FaultState::default(),
            frame_ctr: 0,
            system_time_ms: 0,
            ig_mode: 0,
            host_frame_ctr: 0,
            config: cfg,
        }
    }
}

// ─────────────────────────────────────────── constructors ──

impl GimbalSimulator {
    /// Create a simulator with a specific configuration.
    pub fn with_config(cfg: Config) -> Self {
        let (hfov, vfov) = cfg.fov_at_zoom_for_camera(0, 0.0);
        Self {
            hfov,
            vfov,
            config: cfg,
            ..Default::default()
        }
    }
}

// ─────────────────────────────────────────── command handlers ──

impl GimbalSimulator {
    /// Apply a CIGI SensorControl command.
    ///
    /// Mapping:
    /// - `sensor_state` 0 → Disabled
    /// - `sensor_state` 2 → Track
    /// - `sensor_state` 4 → Geopoint (extension; ac_coupling/noise encode lat/lon)
    /// - `sensor_state` 1 + `track_mode` bit 0 → Rate; otherwise → Position
    /// - `sensor_id` selects the camera (0–2)
    /// - `ac_coupling` (0–1) is zoom level for position/rate modes
    pub fn apply_sensor_control(&mut self, sc: &SensorControl) {
        // ── Mode ──────────────────────────────────────────────────
        self.mode = match sc.sensor_state {
            0 => OrionMode::OrionModeDisabled,
            2 => OrionMode::OrionModeTrack,
            4 => OrionMode::OrionModeGeopoint,
            _ => {
                if sc.track_mode & 0x01 != 0 {
                    OrionMode::OrionModeRate
                } else {
                    OrionMode::OrionModePosition
                }
            }
        };

        // ── Targets ───────────────────────────────────────────────
        match self.mode {
            OrionMode::OrionModeRate => {
                // gain/level → ±max_slew_rate (rad/s)
                self.pan_rate_cmd = (sc.gain * 2.0 - 1.0) * self.config.max_slew_rate;
                self.tilt_rate_cmd = (sc.level * 2.0 - 1.0) * self.config.max_slew_rate;
            }
            OrionMode::OrionModeTrack => {
                // In track mode, gain/level encode the track centroid offset (−0.5 to +0.5).
                self.track_target = [sc.gain - 0.5, sc.level - 0.5];
                self.track_active = true;
                // Seed previous track values so the first tick's derivative is zero.
                self.prev_track_x = self.track_target[0];
                self.prev_track_y = self.track_target[1];
            }
            OrionMode::OrionModeGeopoint => {
                // ac_coupling/noise encode full-Earth lat/lon as 0–1 fractions.
                self.geopoint_lat = (sc.ac_coupling as f64 * 180.0 - 90.0).to_radians();
                self.geopoint_lon = (sc.noise as f64 * 360.0 - 180.0).to_radians();
                self.geopoint_alt = self.config.geopoint_alt_m;
            }
            _ => {
                // Position mode: gain/level → pan/tilt targets in ±π, then clamped.
                let raw_pan = (sc.gain * 2.0 - 1.0) * PI;
                let raw_tilt = (sc.level * 2.0 - 1.0) * PI;
                self.set_target(raw_pan, raw_tilt);
            }
        }

        // ── Zoom ──────────────────────────────────────────────────
        if matches!(
            self.mode,
            OrionMode::OrionModePosition | OrionMode::OrionModeRate
        ) {
            self.apply_zoom(sc.ac_coupling);
        }

        // ── Camera selection ──────────────────────────────────────
        let new_cam = sc.sensor_id as i8;
        if new_cam != self.camera_index {
            self.camera_index = new_cam;
            self.camera_switch_remaining = CAMERA_SWITCH_FRAMES;
            self.update_camera_fov();
        }
    }

    /// Apply a CIGI EntityControl (platform position + attitude update).
    pub fn apply_entity_control(&mut self, ec: &EntityControl) {
        use std::f64::consts::PI as PId;

        self.prev_pos_lat = self.pos_lat;
        self.prev_pos_lon = self.pos_lon;
        self.prev_pos_alt = self.pos_alt;

        self.pos_lat = ec.lat_or_x * PId / 180.0;
        self.pos_lon = ec.lon_or_y * PId / 180.0;
        self.pos_alt = ec.alt_or_z;

        self.platform_roll = ec.roll.to_radians();
        self.platform_pitch = ec.pitch.to_radians();
        self.platform_yaw = ec.yaw.to_radians();

        // Approximate NED velocity from position delta (assumes ~50 Hz update rate).
        self.vel_ned = geo::ned_velocity(
            self.prev_pos_lat, self.prev_pos_lon, self.prev_pos_alt,
            self.pos_lat, self.pos_lon, self.pos_alt,
            0.02,
        );
    }

    // ── Internal helpers ──────────────────────────────────────────

    /// Store pan/tilt targets, enforcing limits.
    ///
    /// For continuous-pan gimbals the commanded angle is wrapped to the
    /// shortest angular path from the current position, so the servo never
    /// takes the long way round.  For limited-pan gimbals the raw value is
    /// clamped and the limit flag is set.
    fn set_target(&mut self, raw_pan: f32, raw_tilt: f32) {
        if self.config.is_continuous_pan() {
            // Shortest path: find delta in (−π, π] then add to current pan.
            let delta = wrap_angle(raw_pan - self.pan);
            self.target_pan = self.pan + delta;
            self.at_pan_limit = false;
        } else {
            self.at_pan_limit = raw_pan.abs() > self.config.pan_limit;
            self.target_pan = raw_pan.clamp(-self.config.pan_limit, self.config.pan_limit);
        }
        self.at_tilt_limit =
            raw_tilt < self.config.tilt_min || raw_tilt > self.config.tilt_max;
        self.target_tilt = raw_tilt.clamp(self.config.tilt_min, self.config.tilt_max);
    }

    fn apply_zoom(&mut self, zoom: f32) {
        let new_zoom = zoom.clamp(0.0, 1.0);
        if (new_zoom - self.zoom_level).abs() > 0.001 {
            self.zoom_level = new_zoom;
            let (h, v) = self.config.fov_at_zoom_for_camera(self.camera_index, self.zoom_level);
            self.hfov = h;
            self.vfov = v;
        }
    }

    fn update_camera_fov(&mut self) {
        // Preserve the current zoom across camera switches — without this,
        // any non-zero zoom held before a switch silently snaps back to wide.
        let (h, v) = self
            .config
            .fov_at_zoom_for_camera(self.camera_index, self.zoom_level);
        self.hfov = h;
        self.vfov = v;
    }
}

// ─────────────────────────────────────────── physics tick ──

impl GimbalSimulator {
    /// Advance the physics simulation by `dt_secs`.
    pub fn tick(&mut self, dt_secs: f64) {
        let dt = dt_secs as f32;

        // Fault state tick
        self.faults.tick(dt);

        // ── Camera switch blackout ─────────────────────────────────
        if self.camera_switch_remaining > 0 {
            self.camera_switch_remaining -= 1;
        }

        // ── Motion (disabled when motor fault is active) ───────────
        // Thermal throttling: halve effective slew rate when thermal warning is active.
        let effective_slew_rate = if self.faults.thermal_warning {
            self.config.max_slew_rate * 0.5
        } else {
            self.config.max_slew_rate
        };

        if !self.faults.motor_fault {
            match self.mode {
                OrionMode::OrionModePosition
                | OrionMode::OrionModePositionNoLimits => {
                    tick_axis_trap(
                        &mut self.pan,
                        &mut self.pan_rate,
                        self.target_pan,
                        dt,
                        effective_slew_rate,
                        self.config.max_accel,
                    );
                    tick_axis_trap(
                        &mut self.tilt,
                        &mut self.tilt_rate,
                        self.target_tilt,
                        dt,
                        effective_slew_rate,
                        self.config.max_accel,
                    );
                }

                OrionMode::OrionModeRate => {
                    // Clamp commanded rate to effective slew rate.
                    let pan_cmd = self.pan_rate_cmd.clamp(-effective_slew_rate, effective_slew_rate);
                    let tilt_cmd = self.tilt_rate_cmd.clamp(-effective_slew_rate, effective_slew_rate);
                    // Ramp actual rate toward commanded rate, limited by max_accel.
                    self.pan_rate = ramp_rate(self.pan_rate, pan_cmd, self.config.max_accel, dt);
                    self.tilt_rate = ramp_rate(self.tilt_rate, tilt_cmd, self.config.max_accel, dt);

                    // Integrate actual (ramped) rate into position.
                    self.pan += self.pan_rate * dt;
                    if !self.config.is_continuous_pan() {
                        self.pan = self.pan.clamp(-self.config.pan_limit, self.config.pan_limit);
                    }
                    self.tilt = (self.tilt + self.tilt_rate * dt)
                        .clamp(self.config.tilt_min, self.config.tilt_max);
                }

                OrionMode::OrionModeGeopoint => {
                    // Compute required pan/tilt to reach commanded geo-target.
                    let (tp, tt) = geo::inverse_geopoint(
                        self.pos_lat, self.pos_lon, self.pos_alt,
                        self.platform_yaw,
                        self.platform_roll, self.platform_pitch,
                        self.config.stabilization_quality,
                        self.geopoint_lat, self.geopoint_lon, self.geopoint_alt,
                        &self.config.gimbal_mount,
                    );
                    self.set_target(tp, tt);

                    // Coordinate pan and tilt so both arrive together, yielding
                    // a straight-line LOS trajectory instead of L-shaped motion.
                    // Skip coordination once either axis is within 1 mrad of its
                    // target to avoid division by near-zero near settle.
                    let pan_err = (self.target_pan - self.pan).abs();
                    let tilt_err = (self.target_tilt - self.tilt).abs();
                    let (pan_rate_lim, tilt_rate_lim) =
                        if pan_err > 0.001 && tilt_err > 0.001 {
                            let pan_time = pan_err / effective_slew_rate;
                            let tilt_time = tilt_err / effective_slew_rate;
                            let max_time = pan_time.max(tilt_time);
                            (
                                (pan_err / max_time).min(effective_slew_rate),
                                (tilt_err / max_time).min(effective_slew_rate),
                            )
                        } else {
                            (effective_slew_rate, effective_slew_rate)
                        };

                    tick_axis_trap(
                        &mut self.pan,
                        &mut self.pan_rate,
                        self.target_pan,
                        dt,
                        pan_rate_lim,
                        self.config.max_accel,
                    );
                    tick_axis_trap(
                        &mut self.tilt,
                        &mut self.tilt_rate,
                        self.target_tilt,
                        dt,
                        tilt_rate_lim,
                        self.config.max_accel,
                    );
                }

                OrionMode::OrionModeTrack => {
                    if self.track_active {
                        let kp = self.config.track_p_gain;
                        let kd = self.config.track_d_gain;
                        // Derivative of track offset (change per second).
                        let dx = (self.track_target[0] - self.prev_track_x) / dt;
                        let dy = (self.track_target[1] - self.prev_track_y) / dt;
                        // PD controller: image offset → angular rate.
                        let pr = (kp * self.track_target[0] + kd * dx) * self.hfov;
                        let tr = (kp * self.track_target[1] + kd * dy) * self.vfov;
                        self.pan += pr * dt;
                        self.tilt += tr * dt;
                        if !self.config.is_continuous_pan() {
                            self.pan = self.pan.clamp(-self.config.pan_limit, self.config.pan_limit);
                        }
                        self.tilt = self.tilt.clamp(self.config.tilt_min, self.config.tilt_max);
                        self.pan_rate = pr;
                        self.tilt_rate = tr;
                        // Track loss when target approaches FOV edge OR
                        // computed confidence drops below the break-lock floor.
                        let threshold = self.config.track_loss_threshold;
                        let (_size, confidence) = self.track_size_confidence();
                        if self.track_target[0].abs() > threshold
                            || self.track_target[1].abs() > threshold
                            || confidence < 0.3
                        {
                            self.track_active = false;
                        }
                    }
                    // Update previous track offsets for next derivative.
                    self.prev_track_x = self.track_target[0];
                    self.prev_track_y = self.track_target[1];
                }

                _ => {
                    // Disabled / fault / other modes: no motion.
                    self.pan_rate = 0.0;
                    self.tilt_rate = 0.0;
                }
            }
        } else {
            self.pan_rate = 0.0;
            self.tilt_rate = 0.0;
        }

        // ── Cross-axis gyroscopic coupling ────────────────────────
        // Opposite-sign perturbations model first-order precession: when
        // both axes slew simultaneously, pan and tilt see equal-magnitude
        // but opposite angular offsets. The product term keeps the coupling
        // dormant unless both axes are actually moving.
        if self.config.gyro_coupling_factor != 0.0 {
            let coupling = self.config.gyro_coupling_factor
                * self.pan_rate
                * self.tilt_rate
                * dt;
            self.pan += coupling;
            self.tilt -= coupling;
        }

        // ── Settled detection ─────────────────────────────────────
        self.settled = matches!(
            self.mode,
            OrionMode::OrionModePosition
                | OrionMode::OrionModeGeopoint
                | OrionMode::OrionModePositionNoLimits
        ) && (self.pan - self.target_pan).abs() < 0.01
          && (self.tilt - self.target_tilt).abs() < 0.01;

        // ── Vibration & jitter ────────────────────────────────────
        self.jitter_phase =
            (self.jitter_phase + self.config.jitter_freq * dt) % 1.0;
        let slew_mag = self.pan_rate.hypot(self.tilt_rate);
        let amp = self.config.jitter_amplitude * (1.0 + slew_mag / self.config.max_slew_rate);
        let sinusoidal = amp * (self.jitter_phase * 2.0 * PI).sin();
        let n1 = lcg_noise_f32(&mut self.noise_seed) * self.config.noise_floor;
        let n2 = lcg_noise_f32(&mut self.noise_seed) * self.config.noise_floor;
        // Second structural resonance (fixed amplitude, not slew-dependent).
        self.jitter_phase_2 =
            (self.jitter_phase_2 + self.config.jitter_freq_2 * dt) % 1.0;
        let amp2 = self.config.jitter_amplitude_2;
        let sin2_pan = amp2 * (self.jitter_phase_2 * 2.0 * PI).sin();
        let sin2_tilt = amp2 * ((self.jitter_phase_2 + 0.25) * 2.0 * PI).sin();

        self.pan_jitter = sinusoidal + sin2_pan + n1;
        // Tilt jitter is correlated but at a 90° phase offset for realism.
        let tilt_sin = amp * ((self.jitter_phase + 0.25) * 2.0 * PI).sin();
        self.tilt_jitter = tilt_sin + sin2_tilt + n2;

        // Store jitter-perturbed angles for look-point and telemetry.
        self.pan_jittered = self.pan + self.pan_jitter;
        self.tilt_jittered = self.tilt + self.tilt_jitter;

        // ── Geolocation ───────────────────────────────────────────
        if let Some([ll, lo, la]) = geo::compute_look_point(
            self.pos_lat, self.pos_lon, self.pos_alt,
            self.platform_yaw,
            self.platform_roll, self.platform_pitch,
            self.config.stabilization_quality,
            self.pan_jittered, self.tilt_jittered,
            self.config.refraction_enabled,
            self.config.terrain_elevation_m,
            &self.config.gimbal_mount,
        ) {
            self.look_lat = ll;
            self.look_lon = lo;
            self.look_alt = la;

            // Slant range = Euclidean distance from platform to look-point in ECEF.
            let [px, py, pz] =
                geo::geodetic_to_ecef(self.pos_lat, self.pos_lon, self.pos_alt);
            let [lx, ly, lz] = geo::geodetic_to_ecef(ll, lo, la);
            let dx = lx - px;
            let dy = ly - py;
            let dz = lz - pz;
            self.slant_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        } else {
            self.slant_range_m = 0.0;
        }

        // ── Timing ───────────────────────────────────────────────
        self.frame_ctr = self.frame_ctr.wrapping_add(1);
        self.system_time_ms =
            self.system_time_ms.wrapping_add((dt_secs * 1000.0) as u32);
    }
}

// ─────────────────────────────────────────── trapezoidal profile ──

/// Single-axis trapezoidal velocity profile.
///
/// Accelerates toward `target`, respecting `max_rate` and `max_accel`.
/// Decelerates in advance so that it arrives at target with zero velocity.
/// Prevents overshoot with a final clamp.
fn tick_axis_trap(
    pos: &mut f32,
    rate: &mut f32,
    target: f32,
    dt: f32,
    max_rate: f32,
    max_accel: f32,
) {
    let err = target - *pos;
    if err.abs() < 1e-6 {
        *rate = 0.0;
        return;
    }
    let dir = err.signum();

    // Stopping distance from current speed (v² / 2a).
    let stop_dist = (*rate * *rate) / (2.0 * max_accel + 1e-12);

    // Decelerate only when the stopping distance at current speed ≥ remaining error.
    let new_rate = if stop_dist >= err.abs() {
        // Brake — don't reverse direction.
        let decel = *rate - dir * max_accel * dt;
        if decel * dir < 0.0 { 0.0 } else { decel }
    } else {
        // Accelerate toward target.
        (*rate + dir * max_accel * dt).clamp(-max_rate, max_rate)
    };

    *rate = new_rate;
    *pos += *rate * dt;

    // Prevent overshoot.
    if dir > 0.0 && *pos > target {
        *pos = target;
        *rate = 0.0;
    } else if dir < 0.0 && *pos < target {
        *pos = target;
        *rate = 0.0;
    }
}

/// Ramp `current` toward `target` rate, limited by `max_accel` (rad/s²).
///
/// Returns the new rate after applying at most `max_accel * dt` change.
fn ramp_rate(current: f32, target: f32, max_accel: f32, dt: f32) -> f32 {
    let delta = target - current;
    let max_delta = max_accel * dt;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

// ─────────────────────────────────────────── hardware telemetry ──

impl GimbalSimulator {
    /// Overwrite simulator state with a telemetry packet received from a real
    /// Orion gimbal (bridge mode).  Only observable fields are updated; command
    /// targets are left untouched so the CIGI host's commands remain in effect.
    ///
    /// Derived state (`pan_jittered`, `tilt_jittered`, `look_lat/lon/alt`,
    /// `slant_range_m`) is recomputed from the new pose. Without that, any
    /// telemetry assembled before the next `tick()` would carry the *previous*
    /// simulator's jittered angles and look-point.
    pub fn apply_hardware_telemetry(&mut self, telem: &GeolocateTelemetryCorePacket) {
        self.pan = telem.pan;
        self.tilt = telem.tilt;
        self.hfov = telem.hfov;
        self.vfov = telem.vfov;
        self.mode = telem.mode;
        self.pos_lat = telem.pos_lat;
        self.pos_lon = telem.pos_lon;
        self.pos_alt = telem.pos_alt;
        self.vel_ned = telem.vel_ned;
        self.camera_index = telem.camera_index;
        self.system_time_ms = telem.system_time;

        // The wire pan/tilt already include the real gimbal's jitter — there
        // is no separate inertial-frame value to add jitter to. Treat the
        // reported pose as the jittered value.
        self.pan_jittered = self.pan;
        self.tilt_jittered = self.tilt;
        self.pan_jitter = 0.0;
        self.tilt_jitter = 0.0;

        // Recompute the look-point and slant range from the new pose so that
        // subsequent `to_telemetry()` / `to_sensor_extended_response()` calls
        // before the next `tick()` aren't reporting the previous look-point.
        if let Some([ll, lo, la]) = geo::compute_look_point(
            self.pos_lat, self.pos_lon, self.pos_alt,
            self.platform_yaw,
            self.platform_roll, self.platform_pitch,
            self.config.stabilization_quality,
            self.pan_jittered, self.tilt_jittered,
            self.config.refraction_enabled,
            self.config.terrain_elevation_m,
            &self.config.gimbal_mount,
        ) {
            self.look_lat = ll;
            self.look_lon = lo;
            self.look_alt = la;
            let [px, py, pz] =
                geo::geodetic_to_ecef(self.pos_lat, self.pos_lon, self.pos_alt);
            let [lx, ly, lz] = geo::geodetic_to_ecef(ll, lo, la);
            let dx = lx - px;
            let dy = ly - py;
            let dz = lz - pz;
            self.slant_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        } else {
            self.slant_range_m = 0.0;
        }
    }
}

// ─────────────────────────────────────────── telemetry builders ──

impl GimbalSimulator {
    /// Build an Orion `GeolocateTelemetryCorePacket` from current state.
    pub fn to_telemetry(&self) -> GeolocateTelemetryCorePacket {
        // We need a local copy of the noise seed for degraded GPS noise.
        // This avoids requiring &mut self while keeping deterministic output
        // per frame (seeded from the simulator's noise_seed snapshot).
        let mut noise_seed = self.noise_seed;

        let mut pkt = GeolocateTelemetryCorePacket::default();

        pkt.system_time = self.system_time_ms;
        if self.faults.gps_loss {
            pkt.pos_lat = 0.0;
            pkt.pos_lon = 0.0;
            pkt.pos_alt = 0.0;
        } else if self.faults.degraded_gps {
            let std = self.faults.gps_noise_std;
            pkt.pos_lat = self.pos_lat + lcg_noise_f32(&mut noise_seed) as f64 * std;
            pkt.pos_lon = self.pos_lon + lcg_noise_f32(&mut noise_seed) as f64 * std;
            pkt.pos_alt = self.pos_alt + lcg_noise_f32(&mut noise_seed) as f64 * std;
        } else {
            pkt.pos_lat = self.pos_lat;
            pkt.pos_lon = self.pos_lon;
            pkt.pos_alt = self.pos_alt;
        }

        // Encoder fault: report frozen pan/tilt instead of actual values.
        if self.faults.encoder_fault {
            pkt.pan = wrap_angle(self.faults.frozen_pan);
            pkt.tilt = self.faults.frozen_tilt;
        } else {
            // Report jitter-perturbed pan/tilt (pre-computed in tick()).
            // Normalise to (−π, π] so the wire value is always in range.
            pkt.pan = wrap_angle(self.pan_jittered);
            pkt.tilt = self.tilt_jittered;
        }
        pkt.hfov = self.hfov;
        pkt.vfov = self.vfov;
        pkt.mode = self.mode;
        pkt.camera_index = self.camera_index;

        // NED velocity (zeroed when GPS is lost).
        pkt.vel_ned = if self.faults.gps_loss || self.faults.imu_dropout {
            [0.0; 3]
        } else {
            self.vel_ned
        };

        // Gimbal orientation quaternion in the inertial NED frame.
        // Computed from (pan, tilt) using Rz(pan) * Ry(−tilt).
        pkt.gimbal_quat = gimbal_quat(self.pan, self.tilt);

        // Platform attitude quaternion (ZYX Euler: yaw → pitch → roll).
        pkt.ins_quat = if self.faults.imu_dropout {
            None
        } else {
            Some(platform_quat(
                self.platform_roll,
                self.platform_pitch,
                self.platform_yaw,
            ))
        };

        // LOS unit vector in ECEF.
        pkt.los_ecef = geo::los_ecef(
            self.pos_lat,
            self.pos_lon,
            self.platform_yaw,
            self.platform_roll,
            self.platform_pitch,
            self.config.stabilization_quality,
            self.pan,
            self.tilt,
            &self.config.gimbal_mount,
        );

        // Range source. Use the laser when it is healthy and the current
        // slant range is within the configured maximum range.
        if self.laser_enabled
            && !self.faults.laser_fault
            && self.slant_range_m > 0.0
            && self.slant_range_m <= self.config.laser_max_range_m
        {
            pkt.range_source = crate::orion::RangeDataSrc::RangeSrcLaser;
        } else {
            pkt.range_source = crate::orion::RangeDataSrc::RangeSrcNone;
        }

        // Track data. Size and confidence scale with slant range, FOV, and
        // how close the target is to the FOV edge.
        if matches!(self.mode, OrionMode::OrionModeTrack) {
            pkt.has_track_data = 1;
            let (size, confidence) = self.track_size_confidence();
            pkt.primary_track_data = Some(PrimaryTrackData {
                pos: self.track_target,
                size,
                confidence: if self.track_active { confidence } else { 0.0 },
                coasting: if self.track_active { 0 } else { 1 },
                active: self.track_active as u32,
            });
        }

        pkt
    }

    /// Compute dynamic track size (fraction of FOV) and confidence [0..1] based
    /// on slant range, current HFOV, target size, and offset from FOV centre.
    fn track_size_confidence(&self) -> (f32, f32) {
        let hfov = self.hfov.max(1e-6);
        let angular_size = if self.slant_range_m > 0.0 {
            (self.config.track_target_size_m as f64 / self.slant_range_m) as f32
        } else {
            // No valid range — fall back to a nominal 0.5° target so we still
            // emit a plausible, non-zero size.
            0.5_f32.to_radians()
        };
        let size = (angular_size / hfov).clamp(0.01, 0.5);

        let offset_mag = self.track_target[0].hypot(self.track_target[1]);
        let threshold = self.config.track_loss_threshold.max(1e-6);
        let offset_fraction = (offset_mag / threshold).clamp(0.0, 1.0);

        let min_resolvable = hfov * 0.005;
        let size_factor = (angular_size / min_resolvable).clamp(0.0, 1.0);

        let confidence = 0.95 * (1.0 - offset_fraction.powi(2)) * size_factor;
        (size, confidence.clamp(0.0, 1.0))
    }

    /// Build a CIGI SensorExtendedResponse from current state.
    ///
    /// `entity_lat/lon/alt` fields are populated with the WGS84 look-point
    /// (where the gimbal LOS intersects the Earth), not the platform position.
    pub fn to_sensor_extended_response(&self) -> SensorExtendedResponse {
        let telem = self.to_telemetry();
        let mut resp = crate::convert::to_cigi::telemetry_to_sensor_extended_response(
            &telem,
            0,
            0,
            self.settled,
            self.track_active,
        );

        // Override entity position with computed look-point.
        // Zero out (0,0,0) during camera switch blackout, when the look-point
        // ray missed the Earth, or when GPS is lost (the look-point is derived
        // from the platform position which `to_telemetry` already zeroed —
        // leaking the cached `self.look_*` here would defeat that gate).
        let suppress = self.camera_switch_remaining > 0
            || self.look_alt >= 1e6
            || self.faults.gps_loss;
        if suppress {
            resp.entity_lat = 0.0;
            resp.entity_lon = 0.0;
            resp.entity_alt = 0.0;
        } else {
            let deg = 180.0 / std::f64::consts::PI;
            resp.entity_lat = self.look_lat * deg;
            resp.entity_lon = self.look_lon * deg;
            resp.entity_alt = self.look_alt;
        }

        // Gate size derived from FOV and configured tracking gate angular size.
        const SENSOR_RESOLUTION: u16 = 640;
        let hfov_deg = self.hfov.to_degrees().max(1e-6);
        let gate_px = (self.config.track_gate_size_deg / hfov_deg
            * SENSOR_RESOLUTION as f32) as u16;
        let gate_px = gate_px.clamp(1, SENSOR_RESOLUTION);
        resp.gate_x_size = gate_px;
        resp.gate_y_size = gate_px;

        // Per CIGI v3.3, gate_x_pos/gate_y_pos are the image-plane tracking gate
        // centroid position, not gimbal pan/tilt. In track mode they follow the
        // fractional track target offsets; in all other modes the gate is
        // bore-sighted.
        if self.mode == OrionMode::OrionModeTrack {
            resp.gate_x_pos = self.track_target[0];
            resp.gate_y_pos = self.track_target[1];
        } else {
            resp.gate_x_pos = 0.0;
            resp.gate_y_pos = 0.0;
        }

        resp
    }

    /// Build a CIGI StartOfFrame from current state.
    pub fn to_start_of_frame(&self) -> StartOfFrame {
        StartOfFrame {
            ig_status: 0,
            ig_mode: self.ig_mode,
            timestamp_valid: false,
            earth_ref_model: false,
            minor_version: 3,
            db_number: 0,
            ig_frame_ctr: self.frame_ctr,
            timestamp: self.system_time_ms as f64 / 1000.0,
            last_host_frame_number: self.host_frame_ctr,
        }
    }
}

// ─────────────────────────────────────────── angle utilities ──

/// Wrap an angle into (−π, π].
#[inline]
fn wrap_angle(a: f32) -> f32 {
    let pi2 = 2.0 * PI;
    ((a + PI).rem_euclid(pi2)) - PI
}

// ─────────────────────────────────────────── quaternion helpers ──

/// Gimbal orientation quaternion [w, x, y, z] in the inertial NED frame.
///
/// Derived from Rz(pan) * Ry(−tilt):
///   w =  cos(pan/2) * cos(tilt/2)
///   x =  sin(pan/2) * sin(tilt/2)
///   y = −cos(pan/2) * sin(tilt/2)
///   z =  sin(pan/2) * cos(tilt/2)
fn gimbal_quat(pan: f32, tilt: f32) -> [f32; 4] {
    let (sp2, cp2) = ((pan / 2.0).sin(), (pan / 2.0).cos());
    let (st2, ct2) = ((tilt / 2.0).sin(), (tilt / 2.0).cos());
    [cp2 * ct2, sp2 * st2, -cp2 * st2, sp2 * ct2]
}

/// Platform attitude quaternion [w, x, y, z] from ZYX Euler angles.
fn platform_quat(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let (cr, sr) = ((roll / 2.0).cos(), (roll / 2.0).sin());
    let (cp, sp) = ((pitch / 2.0).cos(), (pitch / 2.0).sin());
    let (cy, sy) = ((yaw / 2.0).cos(), (yaw / 2.0).sin());
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

// ─────────────────────────────────────────── tests ──

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slew_toward_target() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 1.0;
        sim.target_tilt = 0.5;

        sim.tick(1.0);

        assert!(sim.pan > 0.0);
        assert!(sim.pan <= MAX_SLEW_RATE + 1e-5);
        assert!(sim.pan <= 1.0 + 1e-5);

        // Tick until convergence
        for _ in 0..100 {
            sim.tick(0.1);
        }
        assert!((sim.pan - 1.0).abs() < 1e-4, "pan={}", sim.pan);
        assert!((sim.tilt - 0.5).abs() < 1e-4, "tilt={}", sim.tilt);
    }

    #[test]
    fn apply_sensor_control_sets_mode() {
        let mut sim = GimbalSimulator::default();
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            gain: 0.75, // → pan target = 0.5π
            level: 0.5, // → tilt target = 0.0
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        assert!(matches!(sim.mode, OrionMode::OrionModePosition));
        assert!(
            (sim.target_pan - 0.5 * std::f32::consts::PI).abs() < 1e-4,
            "target_pan={}",
            sim.target_pan
        );
        assert!(sim.target_tilt.abs() < 1e-4);
    }

    #[test]
    fn sensor_response_end_to_end() {
        let mut sim = GimbalSimulator::default();
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            gain: 0.75,
            level: 0.5,
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        for _ in 0..100 {
            sim.tick(0.1);
        }
        // Pan target was 0.5π rad ≈ 90°; tilt target was 0 rad. In position mode
        // the gate is bore-sighted — verify the actual gimbal angles directly.
        assert!((sim.pan - 0.5 * std::f32::consts::PI).abs() < 0.05, "sim.pan={}", sim.pan);
        assert!(sim.tilt.abs() < 0.05, "sim.tilt={}", sim.tilt);
        let sr = sim.to_sensor_extended_response();
        // Per CIGI v3.3, gate_x/y_pos are image-plane centroids, not pan/tilt.
        // Bore-sighted (0.0) outside track mode.
        assert_eq!(sr.gate_x_pos, 0.0);
        assert_eq!(sr.gate_y_pos, 0.0);
    }

    #[test]
    fn angle_limits_enforced() {
        let mut sim = GimbalSimulator::default();
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            gain: 1.0,  // → raw pan = +π ≈ 180°, exceeds ±170° limit
            level: 0.0, // → raw tilt = −π ≈ −180°, exceeds −110° limit
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        assert!(sim.at_pan_limit, "should be at pan limit");
        assert!(sim.at_tilt_limit, "should be at tilt limit");
        let cfg = Config::default();
        assert!(
            sim.target_pan <= cfg.pan_limit + 1e-5,
            "pan clamped: {}",
            sim.target_pan
        );
        assert!(
            sim.target_tilt >= cfg.tilt_min - 1e-5,
            "tilt clamped: {}",
            sim.target_tilt
        );
    }

    #[test]
    fn rate_mode_integrates() {
        let mut sim = GimbalSimulator::default();
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            track_mode: 1, // → rate mode
            gain: 0.75,    // → pan_rate_cmd = +0.25 * max_slew_rate
            level: 0.5,    // → tilt_rate_cmd = 0
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        assert!(matches!(sim.mode, OrionMode::OrionModeRate));
        let rate = sim.pan_rate_cmd;
        assert!(rate > 0.0, "pan_rate_cmd should be positive");
        sim.tick(1.0);
        assert!(
            (sim.pan - rate).abs() < 1e-4,
            "pan should have integrated to rate*dt: {}",
            sim.pan
        );
    }

    #[test]
    fn rate_mode_accel_limited() {
        // With a small dt, the rate should ramp toward the commanded rate
        // rather than jumping to it instantly.
        let mut sim = GimbalSimulator::default();
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            track_mode: 1, // → rate mode
            gain: 1.0,     // → pan_rate_cmd = +max_slew_rate (~1.047 rad/s)
            level: 0.5,    // → tilt_rate_cmd = 0
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        assert!(matches!(sim.mode, OrionMode::OrionModeRate));

        let commanded = sim.pan_rate_cmd;
        assert!(commanded > 0.0);

        // One tick at 50 Hz (0.02 s). max_accel = 300 deg/s^2 = 5.236 rad/s^2.
        // After one tick: rate = max_accel * dt = 5.236 * 0.02 = 0.1047 rad/s.
        // Commanded rate = max_slew_rate = 1.047 rad/s.
        // So the rate should be well below the commanded rate after one tick.
        sim.tick(0.02);
        let rate_after_one = sim.pan_rate;
        assert!(
            rate_after_one < commanded,
            "rate ({}) should be less than commanded ({}) after 1 tick",
            rate_after_one,
            commanded
        );
        assert!(
            rate_after_one > 0.0,
            "rate should be positive and ramping up"
        );

        // After enough ticks, the rate should reach the commanded value.
        for _ in 0..500 {
            sim.tick(0.02);
        }
        assert!(
            (sim.pan_rate - commanded).abs() < 1e-4,
            "rate ({}) should have converged to commanded ({})",
            sim.pan_rate,
            commanded
        );
    }

    #[test]
    fn camera_switch_blackout() {
        let mut sim = GimbalSimulator::default();
        // Command camera 1
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            sensor_id: 1,
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        assert_eq!(sim.camera_index, 1);
        assert!(sim.camera_switch_remaining > 0, "should be in blackout");
        // Entity position should be zeroed during blackout
        let resp = sim.to_sensor_extended_response();
        assert_eq!(resp.entity_lat, 0.0);
        // After the blackout expires the switch resolves
        for _ in 0..CAMERA_SWITCH_FRAMES {
            sim.tick(0.02);
        }
        assert_eq!(sim.camera_switch_remaining, 0);
    }

    #[test]
    fn accel_profile_smooth() {
        // Verify that pan velocity increases gradually (not instantaneously).
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 1.0;

        // After a single small dt, rate should be small, not yet at max.
        sim.tick(0.001);
        let rate_after_1ms = sim.pan_rate;
        assert!(
            rate_after_1ms < MAX_SLEW_RATE,
            "rate_after_1ms={} should be less than max",
            rate_after_1ms
        );
        assert!(rate_after_1ms > 0.0, "rate should be positive");
    }

    #[test]
    fn gimbal_quat_unit_length() {
        let q = gimbal_quat(0.5, 0.3);
        let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((len - 1.0).abs() < 1e-6, "quaternion not unit: len={}", len);
    }

    #[test]
    fn platform_quat_unit_length() {
        let q = platform_quat(0.1, 0.2, 1.5);
        let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((len - 1.0).abs() < 1e-6, "quaternion not unit: len={}", len);
    }

    // ── Geopoint mode ─────────────────────────────────────────────────────

    #[test]
    fn geopoint_mode_slews_toward_target() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeGeopoint;
        // Platform at 35°N looking south at a target at 30°N (same longitude).
        sim.pos_lat = 35.0_f64.to_radians();
        sim.pos_lon = 0.0;
        sim.pos_alt = 1000.0;
        sim.geopoint_lat = 30.0_f64.to_radians();
        sim.geopoint_lon = 0.0;
        sim.geopoint_alt = 0.0;

        sim.tick(0.5);

        // Tilt should be positive (depression, + down convention) to reach a ground target.
        assert!(sim.tilt > 0.0, "tilt should be positive (down), got {}", sim.tilt);
    }

    #[test]
    fn geopoint_nonzero_altitude_changes_tilt() {
        // With a higher target altitude, the required tilt (depression) should be less
        // because the target is closer in elevation to the platform.
        let mut sim_low = GimbalSimulator::default();
        sim_low.mode = OrionMode::OrionModeGeopoint;
        sim_low.pos_lat = 35.0_f64.to_radians();
        sim_low.pos_lon = 0.0;
        sim_low.pos_alt = 1000.0;
        sim_low.geopoint_lat = 30.0_f64.to_radians();
        sim_low.geopoint_lon = 0.0;
        sim_low.geopoint_alt = 0.0;

        let mut sim_high = GimbalSimulator::default();
        sim_high.mode = OrionMode::OrionModeGeopoint;
        sim_high.pos_lat = 35.0_f64.to_radians();
        sim_high.pos_lon = 0.0;
        sim_high.pos_alt = 1000.0;
        sim_high.geopoint_lat = 30.0_f64.to_radians();
        sim_high.geopoint_lon = 0.0;
        sim_high.geopoint_alt = 500.0; // target at 500m elevation

        // Run enough ticks for the targets to converge.
        for _ in 0..100 {
            sim_low.tick(0.02);
            sim_high.tick(0.02);
        }

        // Higher target altitude → less tilt depression (smaller tilt value).
        assert!(
            sim_high.tilt < sim_low.tilt,
            "higher target alt should require less tilt depression: high={} vs low={}",
            sim_high.tilt, sim_low.tilt
        );
    }

    #[test]
    fn geopoint_config_alt_used_as_default() {
        // Verify that Config::geopoint_alt_m is applied when entering geopoint mode.
        let mut cfg = Config::default();
        cfg.geopoint_alt_m = 250.0;
        cfg.platform_lat = 35.0_f64.to_radians();
        cfg.platform_lon = 0.0;
        cfg.platform_alt = 1000.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.mode = OrionMode::OrionModeGeopoint;
        // Simulate receiving a SensorControl that sets geopoint mode.
        let sc = sim_core::cigi::messages::SensorControl {
            view_id: 0,
            sensor_id: 0,
            sensor_state: 4, // Geopoint mode
            polarity: false,
            line_of_sight_enable: false,
            track_mode: 0,
            response_type: false,
            auto_gain: false,
            track_polarity: false,
            gain: 0.0,
            level: 0.0,
            ac_coupling: 0.5,  // lat fraction → 0° lat
            noise: 0.5,       // lon fraction → 0° lon
        };
        sim.apply_sensor_control(&sc);
        assert!(
            (sim.geopoint_alt - 250.0).abs() < 1e-6,
            "geopoint_alt should be set from config: got {}",
            sim.geopoint_alt
        );
    }

    // ── Track mode ────────────────────────────────────────────────────────

    #[test]
    fn track_mode_proportional_controller() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        // Target offset: 0.2 to the right (positive pan offset)
        sim.track_target = [0.2, 0.0];

        let pan_before = sim.pan;
        sim.tick(0.02);
        assert!(sim.pan > pan_before, "pan should increase for rightward offset");
    }

    #[test]
    fn track_mode_loss_at_fov_edge() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        // Target beyond 0.45 threshold → track loss
        sim.track_target = [0.46, 0.0];

        sim.tick(0.02);

        assert!(!sim.track_active, "track should be lost when target exceeds threshold");
    }

    #[test]
    fn track_mode_stays_active_below_threshold() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        // Offset well below both the 0.45 FOV-edge threshold and the
        // 0.3 confidence floor (dynamic confidence model).
        sim.track_target = [0.2, 0.0];

        sim.tick(0.02);

        assert!(sim.track_active, "track should remain active below threshold");
    }

    #[test]
    fn track_mode_pd_differs_from_pure_p() {
        // With a changing track offset, the derivative term should produce
        // a different rate than pure proportional would.
        let mut cfg = Config::default();
        cfg.track_p_gain = 3.0;
        cfg.track_d_gain = 0.5;
        let mut sim_pd = GimbalSimulator::with_config(cfg.clone());
        sim_pd.mode = OrionMode::OrionModeTrack;
        sim_pd.track_active = true;

        // First tick: set initial offset.
        sim_pd.track_target = [0.1, 0.0];
        sim_pd.tick(0.02);
        let _pan_after_first = sim_pd.pan;

        // Second tick: offset increases → derivative should add to rate.
        sim_pd.track_target = [0.2, 0.0];
        sim_pd.tick(0.02);
        let pan_pd = sim_pd.pan;

        // Compare with pure-P (K_D = 0).
        cfg.track_d_gain = 0.0;
        let mut sim_p = GimbalSimulator::with_config(cfg);
        sim_p.mode = OrionMode::OrionModeTrack;
        sim_p.track_active = true;

        sim_p.track_target = [0.1, 0.0];
        sim_p.tick(0.02);
        sim_p.track_target = [0.2, 0.0];
        sim_p.tick(0.02);
        let pan_p = sim_p.pan;

        // PD should move further than pure-P when offset is increasing.
        assert!(
            (pan_pd - pan_p).abs() > 1e-6,
            "PD controller should produce different result from pure-P: pd={}, p={}",
            pan_pd,
            pan_p
        );
        assert!(
            pan_pd > pan_p,
            "PD should move further when offset is increasing: pd={}, p={}",
            pan_pd,
            pan_p
        );
    }

    #[test]
    fn track_mode_configurable_loss_threshold() {
        // With a higher threshold, an offset of 0.3 should NOT cause track
        // loss (0.3 < 0.5 threshold AND dynamic confidence stays above 0.3).
        let mut cfg = Config::default();
        cfg.track_loss_threshold = 0.5;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.3, 0.0];

        sim.tick(0.02);
        assert!(sim.track_active, "track should stay active with higher threshold");

        // With a lower threshold, an offset of 0.3 exceeds it → track loss.
        let mut cfg2 = Config::default();
        cfg2.track_loss_threshold = 0.25;
        let mut sim2 = GimbalSimulator::with_config(cfg2);
        sim2.mode = OrionMode::OrionModeTrack;
        sim2.track_active = true;
        sim2.track_target = [0.3, 0.0];

        sim2.tick(0.02);
        assert!(!sim2.track_active, "track should be lost with lower threshold");
    }

    // ── Vibration ─────────────────────────────────────────────────────────

    #[test]
    fn vibration_nonzero_after_tick() {
        let mut sim = GimbalSimulator::default();
        sim.tick(0.02);
        // At least one of pan_jitter / tilt_jitter should be nonzero
        assert!(
            sim.pan_jitter != 0.0 || sim.tilt_jitter != 0.0,
            "expected nonzero jitter"
        );
    }

    #[test]
    fn vibration_amplitude_scales_with_slew_rate() {
        // Higher slew rate → higher jitter amplitude.
        let mut sim_still = GimbalSimulator::default();
        sim_still.noise_seed = 0xDEAD_BEEF;
        sim_still.tick(0.02);
        let still_jitter = sim_still.pan_jitter.abs();

        let mut sim_fast = GimbalSimulator::default();
        sim_fast.noise_seed = 0xDEAD_BEEF;
        sim_fast.mode = OrionMode::OrionModeRate;
        sim_fast.pan_rate_cmd = sim_fast.config.max_slew_rate;
        sim_fast.tick(0.02);
        let fast_jitter = sim_fast.pan_jitter.abs();

        assert!(
            fast_jitter >= still_jitter,
            "fast jitter ({fast_jitter}) should be >= still jitter ({still_jitter})"
        );
    }

    // ── Motor fault ───────────────────────────────────────────────────────

    #[test]
    fn motor_fault_stops_motion() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 1.0;
        sim.faults.inject_motor_fault();
        for _ in 0..10 {
            sim.tick(0.02);
        }
        assert_eq!(sim.pan, 0.0, "motor fault: pan should not move");
    }

    // ── GPS loss ──────────────────────────────────────────────────────────

    #[test]
    fn gps_loss_zeroes_position_in_telemetry() {
        let mut sim = GimbalSimulator::default();
        // Give the sim a non-zero position
        sim.pos_lat = 0.5;
        sim.pos_lon = 0.3;
        sim.pos_alt = 200.0;
        sim.faults.inject_gps_loss();
        let telem = sim.to_sensor_extended_response();
        assert_eq!(telem.entity_lat, 0.0, "GPS loss: entity_lat should be 0");
        assert_eq!(telem.entity_lon, 0.0, "GPS loss: entity_lon should be 0");
        assert_eq!(telem.entity_alt, 0.0, "GPS loss: entity_alt should be 0");
    }

    #[test]
    fn gps_loss_zeroes_look_point_in_steady_state() {
        // Once `tick()` has populated `look_lat/lon/alt` with a real ray-cast
        // hit, injecting GPS loss must still zero `entity_*` in the SER —
        // otherwise the look-point leaks the true platform position.
        let mut cfg = Config::default();
        cfg.jitter_amplitude = 0.0;
        cfg.noise_floor = 0.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pos_lat = 0.5;
        sim.pos_lon = -1.5;
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModePosition;
        sim.target_tilt = 0.8; // look down so the LOS hits the ground

        for _ in 0..30 {
            sim.tick(0.02);
        }
        // Sanity check: clean SER reports a non-zero look-point.
        let clean = sim.to_sensor_extended_response();
        assert!(
            clean.entity_lat != 0.0 || clean.entity_lon != 0.0 || clean.entity_alt != 0.0,
            "look-point should be populated before GPS loss"
        );

        sim.faults.inject_gps_loss();
        let faulted = sim.to_sensor_extended_response();
        assert_eq!(faulted.entity_lat, 0.0);
        assert_eq!(faulted.entity_lon, 0.0);
        assert_eq!(faulted.entity_alt, 0.0);
    }

    // ── apply_hardware_telemetry refresh ─────────────────────────────────

    #[test]
    fn apply_hardware_telemetry_refreshes_look_point() {
        // Without the post-apply recompute, telemetry assembled before the
        // next `tick()` would carry the previous simulator's look-point.
        let mut sim = GimbalSimulator::default();
        sim.pos_lat = 0.5;
        sim.pos_lon = -1.5;
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModePosition;
        sim.target_tilt = 0.8;
        for _ in 0..30 {
            sim.tick(0.02);
        }
        let prev_look = (sim.look_lat, sim.look_lon, sim.look_alt);

        // Build a telemetry packet that points the gimbal at a different
        // pan/tilt and apply it. Without the refresh, look_* would be stale.
        let mut hw = sim.to_telemetry();
        hw.pan = 0.5; // change yaw enough to move the look-point
        hw.tilt = 0.6;
        sim.apply_hardware_telemetry(&hw);

        let new_look = (sim.look_lat, sim.look_lon, sim.look_alt);
        assert!(
            (new_look.0 - prev_look.0).abs() > 1e-6
                || (new_look.1 - prev_look.1).abs() > 1e-6,
            "look-point not refreshed after apply_hardware_telemetry: \
             prev={prev_look:?} new={new_look:?}"
        );
    }

    // ── Camera switch preserves zoom ─────────────────────────────────────

    #[test]
    fn camera_switch_preserves_zoom() {
        // A camera switch coincident with the same zoom level (Position mode)
        // must yield FOV scaled to the new camera's narrow end, not its wide
        // end — i.e. `update_camera_fov` honours the in-flight `zoom_level`
        // that `apply_zoom` has just set.
        let mut sim = GimbalSimulator::default();
        // Start on cam 0 at wide.
        sim.camera_index = 0;
        sim.zoom_level = 0.0;

        // Switch to cam 2 (IR) at full narrow zoom in a single command.
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1, // Position
            sensor_id: 2,
            gain: 0.5,
            level: 0.5,
            ac_coupling: 1.0, // narrow zoom
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);

        let (expected_h, expected_v) =
            sim.config.fov_at_zoom_for_camera(2, 1.0);
        assert!(
            (sim.hfov - expected_h).abs() < 1e-6,
            "hfov after switch={} expected (cam2 narrow)={}",
            sim.hfov,
            expected_h
        );
        assert!(
            (sim.vfov - expected_v).abs() < 1e-6,
            "vfov after switch={} expected={}",
            sim.vfov,
            expected_v
        );
        // And it must NOT have snapped back to cam 2's wide-end FOV — that
        // was the pre-fix behaviour.
        let (cam2_wide_h, _) = sim.config.fov_at_zoom_for_camera(2, 0.0);
        assert!(
            (sim.hfov - cam2_wide_h).abs() > 1e-3,
            "hfov={} unexpectedly matches cam2 wide ({}); zoom was lost",
            sim.hfov,
            cam2_wide_h
        );
    }

    // ── Continuous pan shortest path ──────────────────────────────────────

    #[test]
    fn continuous_pan_shortest_path() {
        use std::f32::consts::PI;
        let mut cfg = Config::default();
        cfg.pan_limit = std::f32::consts::TAU; // ≥ TAU-0.001 → is_continuous_pan() == true
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pan = PI; // currently at +π

        // Commanded to -3.0 rad. The naive target would be -3.0, but shortest
        // path from +π is to go to -3.0 + 2π ≈ +3.28 (just past +π going CCW is wrong;
        // actually wrap_angle(-3.0 - π) is in (-π, π]).
        let sc = sim_core::cigi::messages::SensorControl {
            sensor_state: 1,
            // Encode -3.0 rad as gain: g = (target/π + 1) * 0.5 = (-3/π + 1)*0.5
            gain: ((-3.0_f32 / PI) + 1.0) * 0.5,
            level: 0.5,
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);

        // target_pan should be pan + wrap(raw - pan) = π + wrap(-3 - π)
        let raw = -3.0_f32;
        let delta = wrap_angle(raw - sim.pan); // needs to be in (-π, π]
        let expected = PI + delta;
        assert!(
            (sim.target_pan - expected).abs() < 0.01,
            "target_pan={} expected≈{}",
            sim.target_pan,
            expected
        );
    }

    #[test]
    fn jitter_affects_look_point() {
        // Set up two simulators at altitude with a down-looking tilt so the
        // LOS hits the ground.  One has high jitter amplitude; the other zero.
        let mut cfg_jitter = Config::default();
        cfg_jitter.jitter_amplitude = 0.05; // ~2.9 degrees — very noticeable
        cfg_jitter.noise_floor = 0.02;

        let mut cfg_clean = Config::default();
        cfg_clean.jitter_amplitude = 0.0;
        cfg_clean.noise_floor = 0.0;

        let mut sim_jitter = GimbalSimulator::with_config(cfg_jitter);
        let mut sim_clean = GimbalSimulator::with_config(cfg_clean);

        // Place both at 1 km altitude, looking straight down.
        for s in [&mut sim_jitter, &mut sim_clean] {
            s.pos_lat = 0.5_f64; // ~28.6 deg N
            s.pos_lon = -1.5_f64;
            s.pos_alt = 1000.0;
            s.mode = OrionMode::OrionModePosition;
            s.target_pan = 0.3;
            s.target_tilt = 0.8; // ~46 deg down
        }

        // Run several ticks so the look-point converges.
        for _ in 0..50 {
            sim_jitter.tick(0.02);
            sim_clean.tick(0.02);
        }

        // The jitter-perturbed simulator should have a different look-point.
        let dlat = (sim_jitter.look_lat - sim_clean.look_lat).abs();
        let dlon = (sim_jitter.look_lon - sim_clean.look_lon).abs();
        assert!(
            dlat > 1e-9 || dlon > 1e-9,
            "look-point should differ with jitter: dlat={dlat}, dlon={dlon}"
        );

        // Also verify the jittered fields are populated.
        assert!(
            (sim_jitter.pan_jittered - sim_jitter.pan).abs() > 1e-6,
            "pan_jittered should differ from clean pan"
        );
        // Clean sim should have zero jitter offset.
        assert!(
            (sim_clean.pan_jittered - sim_clean.pan).abs() < 1e-9,
            "zero-jitter sim: pan_jittered should equal pan"
        );
    }

    // ── Gimbal mount ──────────────────────────────────────────────────────

    #[test]
    fn gimbal_mount_translation_shifts_look_point() {
        // A simulator configured with a 20 m forward mount translation should
        // see its nadir look-point shifted 20 m north of the platform lat/lon
        // (platform level, yaw=0). Same setup without the mount serves as the
        // baseline.
        let mut baseline_cfg = Config::default();
        baseline_cfg.jitter_amplitude = 0.0;
        baseline_cfg.noise_floor = 0.0;

        let mut mount_cfg = baseline_cfg.clone();
        mount_cfg.gimbal_mount = sim_core::geo::GimbalMount {
            translation_body_m: [20.0, 0.0, 0.0],
            rotation_body_rad: [0.0; 3],
        };

        let mut sim_base = GimbalSimulator::with_config(baseline_cfg);
        let mut sim_mount = GimbalSimulator::with_config(mount_cfg);

        for s in [&mut sim_base, &mut sim_mount] {
            s.pos_lat = 0.5_f64;
            s.pos_lon = -1.5_f64;
            s.pos_alt = 2000.0;
            s.platform_yaw = 0.0;
            s.platform_roll = 0.0;
            s.platform_pitch = 0.0;
            s.mode = OrionMode::OrionModePosition;
            s.target_pan = 0.0;
            s.target_tilt = std::f32::consts::FRAC_PI_2; // nadir
        }

        // Converge both.
        for _ in 0..80 {
            sim_base.tick(0.02);
            sim_mount.tick(0.02);
        }

        // The mount-equipped simulator's look-point should be ~20 m north,
        // which translates to a positive latitude shift of ~20/6.371e6 rad.
        // Jitter is off and the gimbal converged, so the difference should
        // be driven purely by the mount translation. Allow a generous margin
        // to absorb the small NED-frame approximation drift.
        let dn_m = (sim_mount.look_lat - sim_base.look_lat) * 6_371_000.0;
        assert!(
            (dn_m - 20.0).abs() < 0.5,
            "expected ~20 m north shift, got {:.3} m",
            dn_m
        );
    }

    #[test]
    fn gimbal_mount_rotation_changes_geopoint_target() {
        // Same platform state + same commanded geopoint target, one sim with
        // a zero mount and one with a 5° mount yaw offset. The required
        // pan/tilt (inverse geopoint) must differ by the mount yaw offset.
        let mut cfg_a = Config::default();
        cfg_a.jitter_amplitude = 0.0;
        cfg_a.noise_floor = 0.0;

        let mut cfg_b = cfg_a.clone();
        cfg_b.gimbal_mount = sim_core::geo::GimbalMount {
            translation_body_m: [0.0; 3],
            rotation_body_rad: [0.0, 0.0, 5.0_f64.to_radians()],
        };

        let mut sim_a = GimbalSimulator::with_config(cfg_a);
        let mut sim_b = GimbalSimulator::with_config(cfg_b);

        for s in [&mut sim_a, &mut sim_b] {
            s.pos_lat = 0.5;
            s.pos_lon = -1.5;
            s.pos_alt = 2000.0;
            s.mode = OrionMode::OrionModeGeopoint;
            s.geopoint_lat = 0.5001;
            s.geopoint_lon = -1.4999;
            s.geopoint_alt = 0.0;
        }

        // Single tick recomputes set_target from inverse_geopoint.
        sim_a.tick(0.02);
        sim_b.tick(0.02);

        // The mount-yaw-offset sim should command a pan that is 5° less than
        // the baseline (its boresight already points 5° further along yaw).
        let delta = sim_a.target_pan - sim_b.target_pan;
        assert!(
            (delta - 5.0_f32.to_radians()).abs() < 1e-4,
            "expected 5° pan delta, got {}°",
            delta.to_degrees()
        );
    }

    // ── Degraded GPS ──────────────────────────────────────────────────────

    #[test]
    fn degraded_gps_adds_noise_to_position() {
        let mut sim = GimbalSimulator::default();
        sim.pos_lat = 0.5;
        sim.pos_lon = -1.0;
        sim.pos_alt = 500.0;
        sim.tick(0.02); // ensure noise_seed is advanced

        sim.faults.inject_degraded_gps(0.001);
        let telem = sim.to_telemetry();

        // Position should be close to true but not exact.
        assert!(
            (telem.pos_lat - 0.5).abs() < 0.01,
            "degraded GPS: lat should be near true value, got {}",
            telem.pos_lat
        );
        assert!(
            (telem.pos_lat - 0.5).abs() > 1e-12,
            "degraded GPS: lat should have noise"
        );
        assert!(telem.pos_alt != 0.0, "degraded GPS: alt should not be zero");
    }

    #[test]
    fn degraded_gps_does_not_zero_position() {
        let mut sim = GimbalSimulator::default();
        sim.pos_lat = 0.5;
        sim.pos_lon = -1.0;
        sim.pos_alt = 500.0;
        sim.faults.inject_degraded_gps(0.0001);
        let telem = sim.to_telemetry();

        // Unlike gps_loss, positions should NOT be zero.
        assert!(telem.pos_lat != 0.0);
        assert!(telem.pos_lon != 0.0);
        assert!(telem.pos_alt != 0.0);
    }

    // ── Encoder fault ─────────────────────────────────────────────────────

    #[test]
    fn encoder_fault_freezes_reported_pan_tilt() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 1.0;
        sim.target_tilt = 0.5;

        // Move the gimbal partway.
        for _ in 0..5 {
            sim.tick(0.02);
        }
        let frozen_pan = sim.pan;
        let frozen_tilt = sim.tilt;

        // Inject encoder fault at current position.
        sim.faults.inject_encoder_fault(frozen_pan, frozen_tilt);

        // Continue moving internally.
        for _ in 0..50 {
            sim.tick(0.02);
        }

        // Internal state should have moved.
        assert!(
            (sim.pan - frozen_pan).abs() > 0.01,
            "internal pan should have moved: pan={} frozen={}",
            sim.pan, frozen_pan
        );

        // Telemetry should still report frozen values.
        let telem = sim.to_telemetry();
        assert!(
            (telem.pan - wrap_angle(frozen_pan)).abs() < 1e-4,
            "reported pan={} should be frozen at {}",
            telem.pan, frozen_pan
        );
        assert!(
            (telem.tilt - frozen_tilt).abs() < 1e-4,
            "reported tilt={} should be frozen at {}",
            telem.tilt, frozen_tilt
        );
    }

    #[test]
    fn encoder_fault_clear_resumes_reporting() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 1.0;
        sim.faults.inject_encoder_fault(0.0, 0.0);

        for _ in 0..50 {
            sim.tick(0.02);
        }

        sim.faults.clear_encoder_fault();
        sim.tick(0.02);
        let telem = sim.to_telemetry();

        // After clearing, reported pan should match actual (jittered) pan.
        assert!(
            (telem.pan - wrap_angle(sim.pan_jittered)).abs() < 1e-4,
            "after clear: reported pan={} should match actual={}",
            telem.pan, sim.pan_jittered
        );
    }

    // ── Thermal throttling ────────────────────────────────────────────────

    #[test]
    fn thermal_throttle_reduces_slew_rate() {
        // Two identical simulators: one with thermal warning, one without.
        let mut sim_normal = GimbalSimulator::default();
        sim_normal.mode = OrionMode::OrionModePosition;
        sim_normal.target_pan = 1.0;

        let mut sim_hot = GimbalSimulator::default();
        sim_hot.mode = OrionMode::OrionModePosition;
        sim_hot.target_pan = 1.0;
        sim_hot.faults.inject_thermal();

        // Run both for the same time.
        for _ in 0..20 {
            sim_normal.tick(0.02);
            sim_hot.tick(0.02);
        }

        // The throttled sim should have moved less.
        assert!(
            sim_hot.pan < sim_normal.pan,
            "thermal throttle: hot pan={} should be less than normal pan={}",
            sim_hot.pan, sim_normal.pan
        );
    }

    #[test]
    fn thermal_throttle_limits_rate_mode() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeRate;
        sim.pan_rate_cmd = sim.config.max_slew_rate; // full speed command
        sim.faults.inject_thermal();

        // Run until rate settles.
        for _ in 0..200 {
            sim.tick(0.02);
        }

        // Rate should be clamped to half of max_slew_rate.
        let half_max = sim.config.max_slew_rate * 0.5;
        assert!(
            (sim.pan_rate - half_max).abs() < 0.01,
            "thermal throttle rate mode: pan_rate={} should be near half_max={}",
            sim.pan_rate, half_max
        );
    }

    // ── Gyroscopic coupling ────────────────────────────────────────────────

    #[test]
    fn gyro_coupling_zero_factor_no_effect() {
        let mut cfg = Config::default();
        cfg.gyro_coupling_factor = 0.0;
        cfg.jitter_amplitude = 0.0;
        cfg.jitter_amplitude_2 = 0.0;
        cfg.noise_floor = 0.0;
        let mut sim = GimbalSimulator::with_config(cfg.clone());
        sim.mode = OrionMode::OrionModeRate;
        sim.pan_rate_cmd = 0.5;
        sim.tilt_rate_cmd = 0.3;
        for _ in 0..10 {
            sim.tick(0.02);
        }
        let pan_baseline = sim.pan;
        let tilt_baseline = sim.tilt;

        let mut sim2 = GimbalSimulator::with_config(cfg);
        sim2.mode = OrionMode::OrionModeRate;
        sim2.pan_rate_cmd = 0.5;
        sim2.tilt_rate_cmd = 0.3;
        for _ in 0..10 {
            sim2.tick(0.02);
        }
        assert!((sim2.pan - pan_baseline).abs() < 1e-8);
        assert!((sim2.tilt - tilt_baseline).abs() < 1e-8);
    }

    #[test]
    fn gyro_coupling_one_axis_zero_no_effect() {
        let mut cfg = Config::default();
        cfg.gyro_coupling_factor = 0.5;
        cfg.jitter_amplitude = 0.0;
        cfg.jitter_amplitude_2 = 0.0;
        cfg.noise_floor = 0.0;
        let mut sim = GimbalSimulator::with_config(cfg.clone());
        sim.mode = OrionMode::OrionModeRate;
        sim.pan_rate_cmd = 0.5;
        sim.tilt_rate_cmd = 0.0;

        let mut sim_no = {
            let mut c = cfg.clone();
            c.gyro_coupling_factor = 0.0;
            GimbalSimulator::with_config(c)
        };
        sim_no.mode = OrionMode::OrionModeRate;
        sim_no.pan_rate_cmd = 0.5;
        sim_no.tilt_rate_cmd = 0.0;

        for _ in 0..50 {
            sim.tick(0.02);
            sim_no.tick(0.02);
        }
        assert!((sim.pan - sim_no.pan).abs() < 1e-8);
        assert!((sim.tilt - sim_no.tilt).abs() < 1e-8);
    }

    #[test]
    fn gyro_coupling_opposite_signs() {
        // When both axes slew, the coupling pushes pan and tilt by the same
        // magnitude with opposite signs (genuine cross-axis perturbation).
        let mut cfg_base = Config::default();
        cfg_base.jitter_amplitude = 0.0;
        cfg_base.jitter_amplitude_2 = 0.0;
        cfg_base.noise_floor = 0.0;
        cfg_base.gyro_coupling_factor = 0.0;
        let mut sim_base = GimbalSimulator::with_config(cfg_base);
        sim_base.mode = OrionMode::OrionModeRate;
        sim_base.pan_rate_cmd = 0.5;
        sim_base.tilt_rate_cmd = 0.3;
        for _ in 0..50 { sim_base.tick(0.02); }

        let mut cfg_c = Config::default();
        cfg_c.jitter_amplitude = 0.0;
        cfg_c.jitter_amplitude_2 = 0.0;
        cfg_c.noise_floor = 0.0;
        cfg_c.gyro_coupling_factor = 0.1;
        let mut sim_c = GimbalSimulator::with_config(cfg_c);
        sim_c.mode = OrionMode::OrionModeRate;
        sim_c.pan_rate_cmd = 0.5;
        sim_c.tilt_rate_cmd = 0.3;
        for _ in 0..50 { sim_c.tick(0.02); }

        let pan_diff = sim_c.pan - sim_base.pan;
        let tilt_diff = sim_c.tilt - sim_base.tilt;
        assert!(pan_diff.abs() > 1e-6, "pan diff should be non-zero: {}", pan_diff);
        // Opposite signs — pan_diff ≈ −tilt_diff.
        assert!((pan_diff + tilt_diff).abs() < 1e-6,
            "expected opposite-sign coupling: pan_diff={}, tilt_diff={}",
            pan_diff, tilt_diff);
    }

    // ── Multi-frequency jitter ─────────────────────────────────────────────

    #[test]
    fn second_jitter_frequency_nonzero() {
        let mut cfg = Config::default();
        // Disable the primary and noise; keep only the secondary resonance.
        cfg.jitter_amplitude = 0.0;
        cfg.noise_floor = 0.0;
        cfg.jitter_amplitude_2 = 0.01;
        cfg.jitter_freq_2 = 47.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.tick(0.02);
        assert!(sim.pan_jitter != 0.0 || sim.tilt_jitter != 0.0);
    }

    // ── Coordinated geopoint slew ──────────────────────────────────────────

    #[test]
    fn geopoint_axes_arrive_together() {
        // With coordinated rates both axes should cross their target windows
        // within a few ticks of each other even when one needs a much larger
        // angular travel. Use a geopoint that requires a large pan swing and a
        // small tilt delta relative to the platform's initial LOS.
        let mut cfg = Config::default();
        cfg.platform_alt = 1000.0;
        cfg.jitter_amplitude = 0.0;
        cfg.jitter_amplitude_2 = 0.0;
        cfg.noise_floor = 0.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.mode = OrionMode::OrionModeGeopoint;
        sim.pos_alt = 1000.0;
        // Target ~5 km east of origin so pan swings hard but tilt is modest.
        sim.geopoint_lat = 0.0;
        sim.geopoint_lon = 5_000.0 / 6_378_000.0; // ~5 km east in radians
        sim.geopoint_alt = 0.0;

        let mut pan_settled = None;
        let mut tilt_settled = None;
        for i in 0..400 {
            sim.tick(0.02);
            if pan_settled.is_none() && (sim.pan - sim.target_pan).abs() < 0.01 {
                pan_settled = Some(i);
            }
            if tilt_settled.is_none() && (sim.tilt - sim.target_tilt).abs() < 0.01 {
                tilt_settled = Some(i);
            }
        }
        let pan_i = pan_settled.expect("pan should settle");
        let tilt_i = tilt_settled.expect("tilt should settle");
        // Without coordination the slower axis would settle much earlier; with
        // coordination the two converge within ~50 ticks of each other.
        assert!((pan_i as i32 - tilt_i as i32).abs() < 50,
            "axes should arrive together: pan={}, tilt={}", pan_i, tilt_i);
    }

    // ── Laser rangefinder ──────────────────────────────────────────────────

    #[test]
    fn laser_produces_slant_range() {
        // Park the gimbal looking straight down from 1000 m and let one tick
        // recompute the look-point + slant range.
        let mut sim = GimbalSimulator::default();
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModeDisabled;
        sim.tilt = (90.0_f32).to_radians(); // +90° = nadir (looking straight down)
        sim.tick(0.02);
        assert!(sim.slant_range_m > 100.0,
            "slant_range_m should be ≈1000 m when looking down from 1000 m, got {}",
            sim.slant_range_m);
        let telem = sim.to_telemetry();
        assert!(matches!(telem.range_source, crate::orion::RangeDataSrc::RangeSrcLaser));
    }

    #[test]
    fn laser_fault_disables_range_source() {
        let mut sim = GimbalSimulator::default();
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModeDisabled;
        sim.tilt = (90.0_f32).to_radians(); // +90° = nadir (looking straight down)
        sim.tick(0.02);
        sim.faults.inject_laser_fault();
        let telem = sim.to_telemetry();
        assert!(matches!(telem.range_source, crate::orion::RangeDataSrc::RangeSrcNone));
    }

    // ── FOV-derived gate size ─────────────────────────────────────────────

    #[test]
    fn gate_size_scales_with_fov() {
        let mut cfg = Config::default();
        cfg.track_gate_size_deg = 1.0;
        let mut sim_wide = GimbalSimulator::with_config(cfg.clone());
        sim_wide.zoom_level = 0.0;
        let (h, v) = sim_wide.config.fov_at_zoom_for_camera(0, 0.0);
        sim_wide.hfov = h; sim_wide.vfov = v;
        let gate_wide = sim_wide.to_sensor_extended_response().gate_x_size;

        let mut sim_narrow = GimbalSimulator::with_config(cfg);
        sim_narrow.zoom_level = 1.0;
        let (h, v) = sim_narrow.config.fov_at_zoom_for_camera(0, 1.0);
        sim_narrow.hfov = h; sim_narrow.vfov = v;
        let gate_narrow = sim_narrow.to_sensor_extended_response().gate_x_size;

        // Narrower FOV → same angular gate occupies a larger fraction → more
        // pixels.
        assert!(gate_narrow > gate_wide,
            "narrow gate px ({}) should exceed wide gate px ({})",
            gate_narrow, gate_wide);
    }

    // ── Gate position spec compliance ─────────────────────────────────────

    #[test]
    fn gate_pos_zero_outside_track_mode() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.pan = 0.5;
        sim.tilt = -0.3;
        let r = sim.to_sensor_extended_response();
        assert_eq!(r.gate_x_pos, 0.0);
        assert_eq!(r.gate_y_pos, 0.0);
    }

    #[test]
    fn gate_pos_follows_track_target() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.12, -0.08];
        let r = sim.to_sensor_extended_response();
        assert!((r.gate_x_pos - 0.12).abs() < 1e-6);
        assert!((r.gate_y_pos - (-0.08)).abs() < 1e-6);
    }

    // ── Settled flag ──────────────────────────────────────────────────────

    #[test]
    fn settled_flag_set_when_converged() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 0.0;
        sim.target_tilt = 0.0;
        sim.tick(0.02);
        assert!(sim.settled);
    }

    #[test]
    fn settled_flag_false_while_slewing() {
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 1.0;
        sim.target_tilt = 0.0;
        sim.tick(0.02);
        assert!(!sim.settled);
    }
}
