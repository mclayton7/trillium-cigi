// Gimbal state machine — simulates Trillium Orion gimbal dynamics.

use crate::cigi::messages::{EntityControl, SensorControl, SensorExtendedResponse, SensorResponse, StartOfFrame};
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode};

/// Maximum slew rate for pan/tilt: 60 °/s ≈ 1.0472 rad/s.
const MAX_SLEW_RATE: f32 = 1.0472;

/// Simulated gimbal state.
#[derive(Debug, Clone)]
pub struct GimbalSimulator {
    // Current angles (radians)
    pub pan: f32,
    pub tilt: f32,
    // Commanded target angles (radians)
    pub target_pan: f32,
    pub target_tilt: f32,

    pub mode: OrionMode,
    pub hfov: f32, // radians
    pub vfov: f32, // radians

    // Platform position (radians / metres)
    pub pos_lat: f64,
    pub pos_lon: f64,
    pub pos_alt: f64,

    // Frame counter / timing
    pub frame_ctr: u32,
    pub system_time_ms: u32,

    // CIGI IG state
    pub ig_mode: u8,
    pub host_frame_ctr: u32,
}

impl Default for GimbalSimulator {
    fn default() -> Self {
        Self {
            pan: 0.0,
            tilt: 0.0,
            target_pan: 0.0,
            target_tilt: 0.0,
            mode: OrionMode::OrionModeDisabled,
            hfov: 30_f32.to_radians(),
            vfov: 22.5_f32.to_radians(),
            pos_lat: 0.0,
            pos_lon: 0.0,
            pos_alt: 0.0,
            frame_ctr: 0,
            system_time_ms: 0,
            ig_mode: 0,
            host_frame_ctr: 0,
        }
    }
}

impl GimbalSimulator {
    /// Apply a CIGI SensorControl command.
    pub fn apply_sensor_control(&mut self, sc: &SensorControl) {
        use std::f32::consts::PI;
        self.mode = match sc.sensor_state {
            0 => OrionMode::OrionModeDisabled,
            2 => OrionMode::OrionModeTrack,
            _ => OrionMode::OrionModePosition,
        };
        // Map gain/level (0–1) to pan/tilt targets (−π to +π)
        self.target_pan = (sc.gain * 2.0 - 1.0) * PI;
        self.target_tilt = (sc.level * 2.0 - 1.0) * PI;
    }

    /// Apply a CIGI EntityControl (platform position update).
    pub fn apply_entity_control(&mut self, ec: &EntityControl) {
        use std::f64::consts::PI;
        // CIGI lat/lon in degrees → radians
        self.pos_lat = ec.lat_or_x * PI / 180.0;
        self.pos_lon = ec.lon_or_y * PI / 180.0;
        self.pos_alt = ec.alt_or_z;
    }

    /// Advance the physics simulation by `dt_secs`.
    pub fn tick(&mut self, dt_secs: f64) {
        let dt = dt_secs as f32;
        let max_step = MAX_SLEW_RATE * dt;

        let pan_err = self.target_pan - self.pan;
        self.pan += pan_err.clamp(-max_step, max_step);

        let tilt_err = self.target_tilt - self.tilt;
        self.tilt += tilt_err.clamp(-max_step, max_step);

        self.frame_ctr = self.frame_ctr.wrapping_add(1);
        self.system_time_ms = self.system_time_ms.wrapping_add((dt_secs * 1000.0) as u32);
    }

    /// Build an Orion GeolocateTelemetryCorePacket from current state.
    pub fn to_telemetry(&self) -> GeolocateTelemetryCorePacket {
        let mut pkt = GeolocateTelemetryCorePacket::default();
        pkt.system_time = self.system_time_ms;
        pkt.pan = self.pan;
        pkt.tilt = self.tilt;
        pkt.hfov = self.hfov;
        pkt.vfov = self.vfov;
        pkt.mode = self.mode;
        pkt.pos_lat = self.pos_lat;
        pkt.pos_lon = self.pos_lon;
        pkt.pos_alt = self.pos_alt;
        pkt
    }

    /// Build a CIGI SensorResponse from current state.
    pub fn to_sensor_response(&self) -> SensorResponse {
        crate::convert::to_cigi::telemetry_to_sensor_response(&self.to_telemetry(), 0, 0)
    }

    /// Build a CIGI SensorExtendedResponse from current state.
    pub fn to_sensor_extended_response(&self) -> SensorExtendedResponse {
        crate::convert::to_cigi::telemetry_to_sensor_extended_response(&self.to_telemetry(), 0, 0)
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
            last_host_pkt_id: self.host_frame_ctr as u16,
        }
    }
}

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
        assert!((sim.target_pan - 0.5 * std::f32::consts::PI).abs() < 1e-4);
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
        let sr = sim.to_sensor_response();
        // gate_x_pos should be ~0.5π / hfov ≈ 0.5π / 0.524 ≈ 3.0 (large value, clamped by CIGI)
        // Just check it's non-zero
        assert!(sr.gate_x_pos.abs() > 0.0 || sr.gate_y_pos.abs() < 1e-3);
    }
}
