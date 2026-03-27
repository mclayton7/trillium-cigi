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
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode, PrimaryTrackData};

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
            noise_seed: 0xDEAD_BEEF,
            pan_jitter: 0.0,
            tilt_jitter: 0.0,
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
        if !self.faults.motor_fault {
            match self.mode {
                OrionMode::OrionModePosition
                | OrionMode::OrionModePositionNoLimits => {
                    tick_axis_trap(
                        &mut self.pan,
                        &mut self.pan_rate,
                        self.target_pan,
                        dt,
                        self.config.max_slew_rate,
                        self.config.max_accel,
                    );
                    tick_axis_trap(
                        &mut self.tilt,
                        &mut self.tilt_rate,
                        self.target_tilt,
                        dt,
                        self.config.max_slew_rate,
                        self.config.max_accel,
                    );
                }

                OrionMode::OrionModeRate => {
                    // Ramp actual rate toward commanded rate, limited by max_accel.
                    self.pan_rate = ramp_rate(self.pan_rate, self.pan_rate_cmd, self.config.max_accel, dt);
                    self.tilt_rate = ramp_rate(self.tilt_rate, self.tilt_rate_cmd, self.config.max_accel, dt);

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
                        self.config.max_slew_rate,
                        self.config.max_accel,
                    );
                    tick_axis_trap(
                        &mut self.tilt,
                        &mut self.tilt_rate,
                        self.target_tilt,
                        dt,
                        self.config.max_slew_rate,
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

        // ── Geolocation ───────────────────────────────────────────
        if let Some([ll, lo, la]) = geo::compute_look_point(
            self.pos_lat, self.pos_lon, self.pos_alt,
            self.platform_yaw,
            self.platform_roll, self.platform_pitch,
            self.config.stabilization_quality,
            self.pan, self.tilt,
        ) {
            self.look_lat = ll;
            self.look_lon = lo;
            self.look_alt = la;
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
        let mut pkt = GeolocateTelemetryCorePacket::default();

        pkt.system_time = self.system_time_ms;
        pkt.pos_lat = if self.faults.gps_loss { 0.0 } else { self.pos_lat };
        pkt.pos_lon = if self.faults.gps_loss { 0.0 } else { self.pos_lon };
        pkt.pos_alt = if self.faults.gps_loss { 0.0 } else { self.pos_alt };

        // Report pan/tilt with vibration superimposed.
        // Normalise to (−π, π] so the wire value is always in range.
        pkt.pan = wrap_angle(self.pan + self.pan_jitter);
        pkt.tilt = self.tilt + self.tilt_jitter;
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

        // Track data.
        if matches!(self.mode, OrionMode::OrionModeTrack) {
            pkt.has_track_data = 1;
            pkt.primary_track_data = Some(PrimaryTrackData {
                pos: self.track_target,
                size: 0.05,
                confidence: if self.track_active { 0.9 } else { 0.0 },
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
        // Pan target was 0.5π rad ≈ 90°; tilt target was 0 rad.
        assert!((sr.gate_x_pos - 90.0).abs() < 1.0, "gate_x_pos={}", sr.gate_x_pos);
        assert!(sr.gate_y_pos.abs() < 1.0, "gate_y_pos={}", sr.gate_y_pos);
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
        let mut sim = GimbalSimulator::default();
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.4, 0.0]; // below 0.45

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
        let mut sim = GimbalSimulator::with_config(cfg);
        sim.mode = OrionMode::OrionModeTrack;
        sim.track_active = true;
        sim.track_target = [0.46, 0.0];

        sim.tick(0.02);
        assert!(sim.track_active, "track should stay active with higher threshold");

        // With a lower threshold, an offset of 0.3 should cause track loss.
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
}
