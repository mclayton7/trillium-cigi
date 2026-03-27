// Convert Orion commands / platform state → CIGI packets sent to the scene generator.

use crate::cigi::messages::{EntityControl, IgControl, SensorControl, SensorExtendedResponse};
use crate::orion::{GeolocateTelemetryCorePacket, OrionCmdPacket, OrionMode};
use crate::platform::PlatformState;

// ── Simulator compatibility ────────────────────────────────────────────────

/// Map Orion `GeolocateTelemetryCorePacket` → CIGI `SensorExtendedResponse`.
///
/// Called by `GimbalSimulator::to_sensor_extended_response()` in fallback mode.
pub fn telemetry_to_sensor_extended_response(
    telem: &GeolocateTelemetryCorePacket,
    view_id: u16,
    sensor_id: u8,
) -> SensorExtendedResponse {
    let deg = 180.0 / std::f64::consts::PI;
    SensorExtendedResponse {
        view_id,
        sensor_id,
        sensor_status: orion_mode_to_sensor_status(telem.mode),
        gate_x_size: 20,
        gate_y_size: 20,
        gate_x_pos: 0.0,
        gate_y_pos: 0.0,
        frame_ctr: telem.system_time,
        entity_id_valid: false,
        entity_id: 0,
        entity_lat: telem.pos_lat * deg,
        entity_lon: telem.pos_lon * deg,
        entity_alt: telem.pos_alt,
    }
}

fn orion_mode_to_sensor_status(mode: OrionMode) -> u8 {
    match mode {
        OrionMode::OrionModeDisabled | OrionMode::OrionModeFault => 3, // Breaklock
        OrionMode::OrionModeTrack => 1,                                // Tracking
        _ => 0,                                                        // Searching/Active
    }
}

/// Build an `IgControl` packet for the given host frame counter.
pub fn make_ig_control(host_frame: u32) -> IgControl {
    IgControl {
        ig_mode: 0, // Normal operation
        frame_ctr: host_frame,
        last_rcvd_ig_frame_ctr: 0,
        timestamp_valid: false,
        extrapolation_enable: false,
        minor_version: 3,
        db_number: 1,
        timestamp: 0.0,
    }
}

/// Build an `EntityControl` packet from the current platform state.
///
/// `entity_id` should be the IG entity that represents the sensor platform.
pub fn platform_to_entity_control(platform: &PlatformState, entity_id: u16) -> EntityControl {
    EntityControl {
        entity_id,
        entity_state: 1, // Active
        roll: platform.roll_rad.to_degrees(),
        pitch: platform.pitch_rad.to_degrees(),
        yaw: platform.yaw_rad.to_degrees(),
        lat_or_x: platform.lat_rad.to_degrees(),
        lon_or_y: platform.lon_rad.to_degrees(),
        alt_or_z: platform.alt_m,
        ..EntityControl::default()
    }
}

/// Convert an `OrionCmdPacket` to a `SensorControl` packet for the scene generator.
///
/// Mode mapping:
/// - `OrionModeDisabled`  → sensor_state 0
/// - `OrionModePosition`  → sensor_state 1
/// - `OrionModeRate`      → sensor_state 1, track_mode bit 0 set
/// - `OrionModeTrack`     → sensor_state 2
/// - `OrionModeGeopoint`  → sensor_state 4 (proprietary extension)
///
/// Pan/tilt target mapping (Position mode):  target[0..1] (rad) → gain/level via ±π scale.
/// Pan/tilt rate mapping (Rate mode):        target[0..1] (rad/s) → gain/level via ±MAX_RATE scale.
pub fn orion_cmd_to_sensor_control(cmd: &OrionCmdPacket) -> SensorControl {
    use std::f32::consts::PI;
    let max_rate = crate::convert::MAX_SLEW_RATE;

    let (sensor_state, track_mode) = match cmd.cmd.mode {
        OrionMode::OrionModeDisabled => (0u8, 0u8),
        OrionMode::OrionModePosition => (1, 0),
        OrionMode::OrionModeRate => (1, 0x01), // track_mode bit 0 → rate
        OrionMode::OrionModeTrack => (2, 0),
        OrionMode::OrionModeGeopoint => (4, 0),
        OrionMode::OrionModeFault => (0, 0),
        _ => (0, 0),
    };

    let (gain, level) = match cmd.cmd.mode {
        OrionMode::OrionModeRate => {
            // Scale ±max_rate → 0..1 for CIGI gain/level
            let g = (cmd.cmd.target[0] / max_rate + 1.0) * 0.5;
            let l = (cmd.cmd.target[1] / max_rate + 1.0) * 0.5;
            (g.clamp(0.0, 1.0), l.clamp(0.0, 1.0))
        }
        _ => {
            // Scale ±π → 0..1 for CIGI gain/level
            let g = (cmd.cmd.target[0] / PI + 1.0) * 0.5;
            let l = (cmd.cmd.target[1] / PI + 1.0) * 0.5;
            (g.clamp(0.0, 1.0), l.clamp(0.0, 1.0))
        }
    };

    SensorControl {
        sensor_id: 0, // camera 0 by default; override from cmd if available
        view_id: 0,
        sensor_state,
        track_mode,
        gain,
        level,
        ..SensorControl::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orion::{GeolocateTelemetryCorePacket, OrionCmdPacket, OrionMode};
    use crate::platform::PlatformState;
    use std::f32::consts::PI as PI32;
    use std::f64::consts::PI as PI64;

    // ── telemetry_to_sensor_extended_response ────────────────────────────────

    #[test]
    fn telem_gate_pos_boresighted() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.pan = PI32 / 4.0;
        t.tilt = -PI32 / 6.0;
        let r = telemetry_to_sensor_extended_response(&t, 0, 0);
        // Per CIGI v3.3, gate positions are image-plane centroids, not pan/tilt.
        // The base conversion sets them to 0.0 (bore-sighted); the simulator
        // overrides them for track mode in to_sensor_extended_response().
        assert_eq!(r.gate_x_pos, 0.0);
        assert_eq!(r.gate_y_pos, 0.0);
    }

    #[test]
    fn telem_lat_lon_rad_to_deg() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.pos_lat = PI64 / 2.0;
        t.pos_lon = -PI64 / 4.0;
        let r = telemetry_to_sensor_extended_response(&t, 0, 0);
        assert!((r.entity_lat - 90.0).abs() < 1e-10, "entity_lat={}", r.entity_lat);
        assert!((r.entity_lon - (-45.0)).abs() < 1e-10, "entity_lon={}", r.entity_lon);
    }

    #[test]
    fn telem_alt_passes_through() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.pos_alt = 1234.5;
        let r = telemetry_to_sensor_extended_response(&t, 0, 0);
        assert!((r.entity_alt - 1234.5).abs() < 1e-9);
    }

    #[test]
    fn telem_view_and_sensor_id_passed_through() {
        let r = telemetry_to_sensor_extended_response(&GeolocateTelemetryCorePacket::default(), 7, 3);
        assert_eq!(r.view_id, 7);
        assert_eq!(r.sensor_id, 3);
    }

    #[test]
    fn telem_sensor_status_track() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModeTrack;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0).sensor_status, 1);
    }

    #[test]
    fn telem_sensor_status_disabled() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModeDisabled;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0).sensor_status, 3);
    }

    #[test]
    fn telem_sensor_status_position() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModePosition;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0).sensor_status, 0);
    }

    #[test]
    fn telem_frame_ctr_from_system_time() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.system_time = 9876;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0).frame_ctr, 9876);
    }

    #[test]
    fn telem_gate_sizes_are_20() {
        let r = telemetry_to_sensor_extended_response(&GeolocateTelemetryCorePacket::default(), 0, 0);
        assert_eq!(r.gate_x_size, 20);
        assert_eq!(r.gate_y_size, 20);
    }

    // ── make_ig_control ──────────────────────────────────────────────────────

    #[test]
    fn ig_control_fields() {
        let ig = make_ig_control(42);
        assert_eq!(ig.ig_mode, 0);
        assert_eq!(ig.frame_ctr, 42);
        assert_eq!(ig.minor_version, 3);
        assert_eq!(ig.db_number, 1);
        assert!(!ig.timestamp_valid);
    }

    // ── platform_to_entity_control ───────────────────────────────────────────

    #[test]
    fn entity_control_entity_id_passed_through() {
        let ec = platform_to_entity_control(&PlatformState::default(), 5);
        assert_eq!(ec.entity_id, 5);
    }

    #[test]
    fn entity_control_lat_lon_rad_to_deg() {
        let mut p = PlatformState::default();
        p.lat_rad = PI64 / 4.0;
        p.lon_rad = -PI64 / 3.0;
        let ec = platform_to_entity_control(&p, 0);
        assert!((ec.lat_or_x - 45.0).abs() < 1e-10, "lat_or_x={}", ec.lat_or_x);
        assert!((ec.lon_or_y - (-60.0)).abs() < 1e-10, "lon_or_y={}", ec.lon_or_y);
    }

    #[test]
    fn entity_control_alt_and_attitude() {
        let mut p = PlatformState::default();
        p.alt_m = 500.0;
        p.roll_rad = PI32 / 6.0;
        p.pitch_rad = PI32 / 4.0;
        p.yaw_rad = PI32 / 2.0;
        let ec = platform_to_entity_control(&p, 0);
        assert!((ec.alt_or_z - 500.0).abs() < 1e-9);
        assert!((ec.roll - 30.0).abs() < 1e-4, "roll={}", ec.roll);
        assert!((ec.pitch - 45.0).abs() < 1e-4, "pitch={}", ec.pitch);
        assert!((ec.yaw - 90.0).abs() < 1e-4, "yaw={}", ec.yaw);
        assert_eq!(ec.entity_state, 1);
    }

    // ── orion_cmd_to_sensor_control ──────────────────────────────────────────

    fn make_cmd(mode: OrionMode, target: [f32; 2]) -> OrionCmdPacket {
        let mut cmd = OrionCmdPacket::default();
        cmd.cmd.mode = mode;
        cmd.cmd.target = target;
        cmd
    }

    #[test]
    fn sc_disabled_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeDisabled, [0.0, 0.0]));
        assert_eq!(sc.sensor_state, 0);
    }

    #[test]
    fn sc_fault_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeFault, [0.0, 0.0]));
        assert_eq!(sc.sensor_state, 0);
    }

    #[test]
    fn sc_position_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [PI32 / 2.0, 0.0]));
        assert_eq!(sc.sensor_state, 1);
        assert_eq!(sc.track_mode, 0);
        assert!((sc.gain - 0.75).abs() < 1e-5, "gain={}", sc.gain);
        assert!((sc.level - 0.5).abs() < 1e-5, "level={}", sc.level);
    }

    #[test]
    fn sc_position_centre_is_half() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [0.0, 0.0]));
        assert!((sc.gain - 0.5).abs() < 1e-5);
        assert!((sc.level - 0.5).abs() < 1e-5);
    }

    #[test]
    fn sc_position_full_range() {
        let sc_max = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [PI32, 0.0]));
        let sc_min = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [-PI32, 0.0]));
        assert!((sc_max.gain - 1.0).abs() < 1e-5);
        assert!((sc_min.gain - 0.0).abs() < 1e-5);
    }

    #[test]
    fn sc_rate_mode() {
        let max = crate::convert::MAX_SLEW_RATE;
        let sc_pos = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeRate, [max, 0.0]));
        let sc_neg = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeRate, [-max, 0.0]));
        let sc_zero = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeRate, [0.0, 0.0]));
        assert_eq!(sc_pos.sensor_state, 1);
        assert_eq!(sc_pos.track_mode, 0x01);
        assert!((sc_pos.gain - 1.0).abs() < 1e-5);
        assert!((sc_neg.gain - 0.0).abs() < 1e-5);
        assert!((sc_zero.gain - 0.5).abs() < 1e-5);
    }

    #[test]
    fn sc_track_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeTrack, [0.0, 0.0]));
        assert_eq!(sc.sensor_state, 2);
    }

    #[test]
    fn sc_geopoint_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeGeopoint, [0.0, 0.0]));
        assert_eq!(sc.sensor_state, 4);
    }
}
