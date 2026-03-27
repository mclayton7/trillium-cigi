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

use crate::cigi::messages::{EntityControl, SensorControl, SensorExtendedResponse, StartOfFrame};
use crate::config::Config;
use crate::faults::{FaultState, lcg_noise_f32};
use crate::geo;
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode, PrimaryTrackData, RangeDataSrc};

// ─────────────────────────────────────────── constants (kept for tests) ──

#[allow(dead_code)]
/// Default maximum slew rate (rad/s) = 60 °/s.  Matches Config::default().
pub const MAX_SLEW_RATE: f32 = 1.047_198; // 60°/s in rad/s

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

    // ── Laser rangefinder ────────────────────────────────────────────
    /// Slant range from platform to look-point (metres). 0.0 when no look-point.
    pub slant_range_m: f64,
    /// Whether the laser rangefinder is enabled.
    pub laser_enabled: bool,

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
    jitter_phase: f32, // accumulated cycles
    noise_seed: u32,   // LCG state
    /// Instantaneous jitter added to pan for this frame's telemetry.
    pan_jitter: f32,
    tilt_jitter: f32,
    /// Jitter-perturbed pan/tilt (clean angle + jitter).  Used for
    /// look-point computation and telemetry so the reported LOS reflects
    /// the physical vibration of the sensor.
    pub pan_jittered: f32,
    pub tilt_jittered: f32,

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
            slant_range_m: 0.0,
            laser_enabled: true,
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
            noise_seed: 0xDEAD_BEEF,
            pan_jitter: 0.0,
            tilt_jitter: 0.0,
            pan_jittered: 0.0,
            tilt_jittered: 0.0,
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
        let (h_deg, v_deg) = self.config.camera_fov(self.camera_index);
        self.hfov = h_deg.to_radians();
        self.vfov = v_deg.to_radians();
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
                        self.geopoint_lat, self.geopoint_lon, self.geopoint_alt,
                    );
                    self.set_target(tp, tt);
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
                        // Track loss when target approaches FOV edge.
                        let threshold = self.config.track_loss_threshold;
                        if self.track_target[0].abs() > threshold
                            || self.track_target[1].abs() > threshold
                        {
                            self.track_active = false;
                        }
                        // Track loss when confidence drops below 0.3
                        // (target too small to resolve at current range/zoom).
                        if self.track_active {
                            let offset_mag = (self.track_target[0].powi(2) + self.track_target[1].powi(2)).sqrt();
                            let offset_fraction = offset_mag / self.config.track_loss_threshold;
                            let angular_size = if self.slant_range_m > 1.0 {
                                self.config.track_target_size_m as f64 / self.slant_range_m
                            } else {
                                0.5
                            };
                            let min_resolvable = self.hfov * 0.005;
                            let size_factor = (angular_size as f32 / min_resolvable).min(1.0);
                            let confidence = (0.95 * (1.0 - offset_fraction.powi(2)) * size_factor).clamp(0.0, 1.0);
                            if confidence < 0.3 {
                                self.track_active = false;
                            }
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

        // ── Vibration & jitter ────────────────────────────────────
        self.jitter_phase =
            (self.jitter_phase + self.config.jitter_freq * dt) % 1.0;
        let slew_mag = self.pan_rate.hypot(self.tilt_rate);
        let amp = self.config.jitter_amplitude * (1.0 + slew_mag / self.config.max_slew_rate);
        let sinusoidal = amp * (self.jitter_phase * 2.0 * PI).sin();
        let n1 = lcg_noise_f32(&mut self.noise_seed) * self.config.noise_floor;
        let n2 = lcg_noise_f32(&mut self.noise_seed) * self.config.noise_floor;
        self.pan_jitter = sinusoidal + n1;
        // Tilt jitter is correlated but at a 90° phase offset for realism.
        let tilt_sin = amp * ((self.jitter_phase + 0.25) * 2.0 * PI).sin();
        self.tilt_jitter = tilt_sin + n2;

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
        ) {
            self.look_lat = ll;
            self.look_lon = lo;
            self.look_alt = la;

            // Slant range: Euclidean distance in ECEF between platform and look-point.
            let platform_ecef = geo::geodetic_to_ecef(self.pos_lat, self.pos_lon, self.pos_alt);
            let look_ecef = geo::geodetic_to_ecef(ll, lo, la);
            self.slant_range_m = ((platform_ecef[0] - look_ecef[0]).powi(2)
                + (platform_ecef[1] - look_ecef[1]).powi(2)
                + (platform_ecef[2] - look_ecef[2]).powi(2))
            .sqrt();
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
        );

        // Laser rangefinder.
        if self.laser_enabled
            && !self.faults.laser_fault
            && self.slant_range_m > 0.0
            && self.slant_range_m <= self.config.laser_max_range_m
        {
            pkt.range_source = RangeDataSrc::RangeSrcLaser;
        }

        // Track data.
        if matches!(self.mode, OrionMode::OrionModeTrack) {
            pkt.has_track_data = 1;

            // Dynamic target size (normalized to FOV).
            let angular_size = if self.slant_range_m > 1.0 {
                self.config.track_target_size_m as f64 / self.slant_range_m
            } else {
                0.5 // fallback when no valid range
            };
            let size = (angular_size as f32 / self.hfov).clamp(0.01, 0.5);

            // Dynamic confidence based on offset, range, and resolvability.
            let offset_mag = (self.track_target[0].powi(2) + self.track_target[1].powi(2)).sqrt();
            let offset_fraction = offset_mag / self.config.track_loss_threshold;
            let min_resolvable = self.hfov * 0.005; // half percent of FOV
            let size_factor = (angular_size as f32 / min_resolvable).min(1.0);
            let confidence = if self.track_active {
                (0.95 * (1.0 - offset_fraction.powi(2)) * size_factor).clamp(0.0, 1.0)
            } else {
                0.0
            };

            pkt.primary_track_data = Some(PrimaryTrackData {
                pos: self.track_target,
                size,
                confidence,
                coasting: if self.track_active { 0 } else { 1 },
                active: self.track_active as u32,
            });
        }

        pkt
    }

    /// Build a CIGI SensorExtendedResponse from current state.
    ///
    /// `entity_lat/lon/alt` fields are populated with the WGS84 look-point
    /// (where the gimbal LOS intersects the Earth), not the platform position.
    pub fn to_sensor_extended_response(&self) -> SensorExtendedResponse {
        let telem = self.to_telemetry();
        let mut resp =
            crate::convert::to_cigi::telemetry_to_sensor_extended_response(&telem, 0, 0);

        // Override entity position with computed look-point.
        // During camera switch blackout, report NaN-equivalent (0,0,0).
        if self.camera_switch_remaining == 0 && self.look_alt < 1e6 {
            let deg = 180.0 / std::f64::consts::PI;
            resp.entity_lat = self.look_lat * deg;
            resp.entity_lon = self.look_lon * deg;
            resp.entity_alt = self.look_alt;
        } else {
            resp.entity_lat = 0.0;
            resp.entity_lon = 0.0;
            resp.entity_alt = 0.0;
        }

        // Compute gate size from FOV and configured tracking gate angular size.
        const SENSOR_RESOLUTION: u16 = 640;
        let gate_px = (self.config.track_gate_size_deg / self.hfov.to_degrees()
            * SENSOR_RESOLUTION as f32) as u16;
        let gate_px = gate_px.clamp(1, SENSOR_RESOLUTION);
        resp.gate_x_size = gate_px;
        resp.gate_y_size = gate_px;

        // Per CIGI v3.3, gate_x_pos/gate_y_pos are the tracking gate centroid
        // position on the image plane, not gimbal pan/tilt angles.
        // In track mode, use the fractional track target offsets (-0.5 to +0.5).
        // In all other modes, the gate is bore-sighted (0.0, 0.0).
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
        let sc = crate::cigi::messages::SensorControl {
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
        let sc = crate::cigi::messages::SensorControl {
            sensor_state: 1,
            gain: 0.75,
            level: 0.5,
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        for _ in 0..100 {
            sim.tick(0.1);
        }
        let sr = sim.to_sensor_extended_response();
        // Position mode: gate fields should be bore-sighted (0.0).
        assert_eq!(sr.gate_x_pos, 0.0, "gate_x_pos should be 0 in position mode");
        assert_eq!(sr.gate_y_pos, 0.0, "gate_y_pos should be 0 in position mode");
    }

    #[test]
    fn gate_pos_track_mode_uses_track_target() {
        let mut sim = GimbalSimulator::default();
        // Put simulator in track mode
        let sc = crate::cigi::messages::SensorControl {
            sensor_state: 2, // Track
            gain: 0.7,       // track_target[0] = 0.7 - 0.5 = 0.2
            level: 0.6,      // track_target[1] = 0.6 - 0.5 = 0.1
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        sim.tick(0.02);
        let sr = sim.to_sensor_extended_response();
        assert!((sr.gate_x_pos - 0.2).abs() < 1e-5, "gate_x_pos={}", sr.gate_x_pos);
        assert!((sr.gate_y_pos - 0.1).abs() < 1e-5, "gate_y_pos={}", sr.gate_y_pos);
    }

    #[test]
    fn gate_pos_position_mode_is_boresighted() {
        let mut sim = GimbalSimulator::default();
        let sc = crate::cigi::messages::SensorControl {
            sensor_state: 1, // Position mode
            gain: 0.75,
            level: 0.5,
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        sim.tick(0.02);
        let sr = sim.to_sensor_extended_response();
        assert_eq!(sr.gate_x_pos, 0.0);
        assert_eq!(sr.gate_y_pos, 0.0);
    }

    #[test]
    fn angle_limits_enforced() {
        let mut sim = GimbalSimulator::default();
        let sc = crate::cigi::messages::SensorControl {
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
        let sc = crate::cigi::messages::SensorControl {
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
        let sc = crate::cigi::messages::SensorControl {
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
        let sc = crate::cigi::messages::SensorControl {
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
        let sc = crate::cigi::messages::SensorControl {
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
        let mut cfg = Config::default();
        cfg.platform_alt = 1000.0; // elevated so ray-cast yields valid slant range
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.1, 0.0]; // well within 0.45 threshold

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
        // With a higher threshold, an offset of 0.46 should NOT cause track loss.
        let mut cfg = Config::default();
        cfg.track_loss_threshold = 0.5;
        cfg.platform_alt = 1000.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.1, 0.0]; // well within 0.5 threshold

        sim.tick(0.02);
        assert!(sim.track_active, "track should stay active with higher threshold");

        // With a lower threshold, an offset of 0.3 should cause track loss (FOV-edge).
        let mut cfg2 = Config::default();
        cfg2.track_loss_threshold = 0.25;
        cfg2.platform_alt = 1000.0;
        let mut sim2 = GimbalSimulator::with_config(cfg2);
        sim2.pos_alt = 1000.0;
        sim2.mode = OrionMode::OrionModeTrack;
        sim2.track_active = true;
        sim2.track_target = [0.3, 0.0]; // exceeds 0.25 threshold

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
        let sc = crate::cigi::messages::SensorControl {
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

    // ── Laser rangefinder ──────────────────────────────────────────────

    #[test]
    fn slant_range_nadir_equals_altitude() {
        // Platform at altitude, looking straight down → slant range ≈ altitude.
        let mut cfg = Config::default();
        cfg.stabilization_quality = 0.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pos_lat = 37.0_f64.to_radians();
        sim.pos_lon = -122.0_f64.to_radians();
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModePosition;
        sim.target_pan = 0.0;
        sim.target_tilt = std::f32::consts::FRAC_PI_2; // straight down

        // Converge position
        for _ in 0..200 {
            sim.tick(0.02);
        }

        assert!(
            (sim.slant_range_m - 1000.0).abs() < 50.0,
            "slant range should be near altitude (1000 m): got {}",
            sim.slant_range_m
        );
    }

    #[test]
    fn laser_fault_clears_range_source_in_telemetry() {
        let mut cfg = Config::default();
        cfg.laser_max_range_m = 50000.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pos_lat = 37.0_f64.to_radians();
        sim.pos_lon = -122.0_f64.to_radians();
        sim.pos_alt = 1000.0;
        sim.mode = OrionMode::OrionModePosition;
        sim.target_tilt = std::f32::consts::FRAC_PI_2;

        for _ in 0..200 {
            sim.tick(0.02);
        }

        // Without fault: range_source should be laser.
        let telem = sim.to_telemetry();
        assert_eq!(
            telem.range_source,
            RangeDataSrc::RangeSrcLaser,
            "expected laser range source without fault"
        );

        // With fault: range_source should be None.
        sim.faults.inject_laser_fault();
        let telem_fault = sim.to_telemetry();
        assert_eq!(
            telem_fault.range_source,
            RangeDataSrc::RangeSrcNone,
            "expected no range source with laser fault"
        );
    }

    #[test]
    fn beyond_max_range_no_laser_source() {
        let mut cfg = Config::default();
        cfg.laser_max_range_m = 500.0; // very short max range
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.pos_lat = 37.0_f64.to_radians();
        sim.pos_lon = -122.0_f64.to_radians();
        sim.pos_alt = 5000.0; // 5 km altitude, well beyond 500 m max
        sim.mode = OrionMode::OrionModePosition;
        sim.target_tilt = std::f32::consts::FRAC_PI_2; // straight down

        for _ in 0..200 {
            sim.tick(0.02);
        }

        // Slant range should be computed but exceed max.
        assert!(
            sim.slant_range_m > 500.0,
            "slant range should exceed max: {}",
            sim.slant_range_m
        );

        let telem = sim.to_telemetry();
        assert_eq!(
            telem.range_source,
            RangeDataSrc::RangeSrcNone,
            "expected no laser source when beyond max range"
        );
    }

    // ── Gate size from zoom ──────────────────────────────────────────────

    #[test]
    fn gate_size_default_zoom_wide_fov() {
        // Default config: hfov_wide = 30 deg, track_gate_size_deg = 1.0
        // gate_px = 1.0 / 30.0 * 640 ≈ 21
        let sim = GimbalSimulator::default();
        let sr = sim.to_sensor_extended_response();
        assert_eq!(sr.gate_x_size, 21, "gate_x_size at wide FOV");
        assert_eq!(sr.gate_y_size, 21, "gate_y_size at wide FOV");
    }

    #[test]
    fn gate_size_full_zoom_narrow_fov() {
        // Default config: hfov_narrow = 3 deg, track_gate_size_deg = 1.0
        // gate_px = 1.0 / 3.0 * 640 ≈ 213
        let mut cfg = Config::default();
        cfg.track_gate_size_deg = 1.0;
        let mut sim = GimbalSimulator::with_config(cfg);
        // Set zoom to 1.0 (full narrow)
        let sc = crate::cigi::messages::SensorControl {
            sensor_state: 1,
            ac_coupling: 1.0, // zoom = 1.0
            ..Default::default()
        };
        sim.apply_sensor_control(&sc);
        sim.tick(0.02);
        let sr = sim.to_sensor_extended_response();
        assert_eq!(sr.gate_x_size, 213, "gate_x_size at narrow FOV");
        assert_eq!(sr.gate_y_size, 213, "gate_y_size at narrow FOV");
    }

    #[test]
    fn gate_size_clamped_minimum() {
        // With a very large FOV relative to gate size, result should clamp to 1.
        let mut cfg = Config::default();
        cfg.track_gate_size_deg = 0.01;
        cfg.hfov_wide = 90.0_f32.to_radians();
        let sim = GimbalSimulator::with_config(cfg);
        let sr = sim.to_sensor_extended_response();
        // 0.01 / 90.0 * 640 ≈ 0.07 → clamped to 1
        assert_eq!(sr.gate_x_size, 1, "gate_x_size should clamp to 1");
        assert_eq!(sr.gate_y_size, 1, "gate_y_size should clamp to 1");
    }

    #[test]
    fn gate_size_clamped_maximum() {
        // With a very small FOV relative to gate size, result should clamp to 640.
        let mut cfg = Config::default();
        cfg.track_gate_size_deg = 100.0;
        cfg.hfov_wide = 0.1_f32.to_radians();
        let sim = GimbalSimulator::with_config(cfg);
        let sr = sim.to_sensor_extended_response();
        // 100.0 / 0.1 * 640 = 640000 → clamped to 640
        assert_eq!(sr.gate_x_size, 640, "gate_x_size should clamp to 640");
        assert_eq!(sr.gate_y_size, 640, "gate_y_size should clamp to 640");
    }

    // ── Dynamic track confidence and target size ─────────────────────────

    #[test]
    fn track_data_short_range_high_confidence_large_size() {
        // At 100m slant range with default 2m target size, angular size is large.
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.0, 0.0]; // centered
        sim.slant_range_m = 100.0;

        let telem = sim.to_telemetry();
        let td = telem.primary_track_data.as_ref().unwrap();

        // angular_size = 2.0 / 100.0 = 0.02 rad; hfov_wide ~ 0.5236 rad
        // size = 0.02 / 0.5236 ~ 0.038 → clamped above 0.01
        assert!(td.size > 0.01, "size should be above minimum: {}", td.size);
        // Confidence should be high (centered target, large angular size).
        assert!(td.confidence > 0.8, "confidence should be high at short range: {}", td.confidence);
    }

    #[test]
    fn track_data_long_range_low_size_reduced_confidence() {
        // At 50km, 2m target is tiny: angular_size = 2/50000 = 0.00004 rad.
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.0, 0.0];
        sim.slant_range_m = 50_000.0;

        let telem = sim.to_telemetry();
        let td = telem.primary_track_data.as_ref().unwrap();

        // size = 0.00004 / 0.5236 ~ 0.00008 → clamped to 0.01
        assert!((td.size - 0.01).abs() < 1e-6, "size should clamp to min at long range: {}", td.size);
        // size_factor = 0.00004 / (0.5236 * 0.005) ~ 0.015 → very low
        // confidence = 0.95 * 1.0 * 0.015 ~ 0.014 → low
        assert!(td.confidence < 0.3, "confidence should be low at extreme range: {}", td.confidence);
    }

    #[test]
    fn track_data_near_fov_edge_low_confidence() {
        // Target near the edge of the FOV should have lower confidence.
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.slant_range_m = 100.0; // short range for good size_factor
        sim.track_target = [0.4, 0.0]; // near threshold (0.45)

        let telem = sim.to_telemetry();
        let td = telem.primary_track_data.as_ref().unwrap();

        // offset_fraction = 0.4 / 0.45 ~ 0.889
        // (1 - 0.889^2) ~ 0.210
        // confidence ~ 0.95 * 0.210 * size_factor
        assert!(td.confidence < 0.5, "confidence should be low near FOV edge: {}", td.confidence);

        // Compare with centered target at same range.
        let mut sim_center = GimbalSimulator::default();
        sim_center.mode = OrionMode::OrionModeTrack;
        sim_center.track_active = true;
        sim_center.slant_range_m = 100.0;
        sim_center.track_target = [0.0, 0.0];

        let telem_center = sim_center.to_telemetry();
        let td_center = telem_center.primary_track_data.as_ref().unwrap();
        assert!(td_center.confidence > td.confidence,
            "centered target should have higher confidence: {} vs {}",
            td_center.confidence, td.confidence);
    }

    #[test]
    fn track_loss_triggered_by_low_confidence() {
        // At extreme range, target is unresolvable → confidence < 0.3 → track loss.
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.0, 0.0]; // centered, so FOV-edge threshold won't trigger
        sim.slant_range_m = 50_000.0; // extreme range

        sim.tick(0.02);

        assert!(!sim.track_active,
            "track should be lost when confidence < 0.3 at extreme range");
    }
}
