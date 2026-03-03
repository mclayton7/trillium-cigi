// Convert CIGI commands → Orion packet types.

use crate::cigi::messages::{EntityControl, SensorControl};
use crate::orion::{GpsDataPacket, OrionCmdPacket, OrionMode};

/// Map CIGI `SensorControl` → `OrionCmdPacket` (ORION_PKT_CMD).
///
/// Mapping:
/// - `sensor_state` 0 (Inactive)  → `OrionModeDisabled`
/// - `sensor_state` 1 (Active)    → `OrionModePosition`
/// - `sensor_state` 2 (Tracking)  → `OrionModeTrack`
/// - `gain` / `level` → pan/tilt targets (as fractions of ±π)
pub fn sensor_control_to_orion_cmd(sc: &SensorControl) -> OrionCmdPacket {
    use std::f32::consts::PI;

    let mode = match sc.sensor_state {
        0 => OrionMode::OrionModeDisabled,
        2 => OrionMode::OrionModeTrack,
        _ => OrionMode::OrionModePosition,
    };

    // CIGI gain/level are 0.0-1.0 — map to ±π radians for pan/tilt targets
    let pan_target = (sc.gain * 2.0 - 1.0) * PI;
    let tilt_target = (sc.level * 2.0 - 1.0) * PI;

    let mut pkt = OrionCmdPacket::default();
    pkt.cmd.target = [pan_target, tilt_target];
    pkt.cmd.mode = mode;
    pkt.cmd.stabilized = 1; // always stabilized
    pkt.cmd.impulse_time = 0.0;
    pkt
}

/// Map CIGI `EntityControl` (platform position) → `GpsDataPacket`.
///
/// CIGI uses degrees for lat/lon; Orion stores lat/lon in radians internally
/// (the generated struct has the f64 fields in whatever unit the XML declares).
pub fn entity_control_to_gps_data(ec: &EntityControl) -> GpsDataPacket {
    use std::f64::consts::PI;
    let mut pkt = GpsDataPacket::default();
    // EntityControl lat/lon are in degrees; GpsData struct stores radians internally.
    pkt.latitude = ec.lat_or_x * PI / 180.0;
    pkt.longitude = ec.lon_or_y * PI / 180.0;
    pkt.altitude = ec.alt_or_z;
    pkt
}
