// Convert CIGI commands → Orion packet types.

use crate::cigi::messages::{EntityControl, SensorControl};
use crate::orion::{GeopointCmdPacket, GpsDataPacket, OrionCmdPacket, OrionMode};

/// Map CIGI `SensorControl` → `OrionCmdPacket` (ORION_PKT_CMD).
///
/// Mapping:
/// - `sensor_state` 0 (Inactive)  → `OrionModeDisabled`
/// - `sensor_state` 2 (Tracking)  → `OrionModeTrack`
/// - `sensor_state` 4 (extension) → `OrionModeGeopoint`
/// - `sensor_state` 1 + `track_mode` bit 0 → `OrionModeRate`; else `OrionModePosition`
/// - `gain` / `level` → pan/tilt targets (position) or rates (rate mode)
pub fn sensor_control_to_orion_cmd(sc: &SensorControl) -> OrionCmdPacket {
    use std::f32::consts::PI;

    // Maximum slew rate (rad/s) — mirrors simulator default.
    const MAX_RATE: f32 = 1.047_198;

    let mode = match sc.sensor_state {
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

    let target = match mode {
        OrionMode::OrionModeRate => {
            // gain/level → ±max_rate (rad/s)
            [
                (sc.gain * 2.0 - 1.0) * MAX_RATE,
                (sc.level * 2.0 - 1.0) * MAX_RATE,
            ]
        }
        _ => {
            // gain/level → ±π (rad) position targets
            [
                (sc.gain * 2.0 - 1.0) * PI,
                (sc.level * 2.0 - 1.0) * PI,
            ]
        }
    };

    let mut pkt = OrionCmdPacket::default();
    pkt.cmd.target = target;
    pkt.cmd.mode = mode;
    pkt.cmd.stabilized = 1; // always stabilised
    pkt.cmd.impulse_time = 0.0;
    pkt
}

/// Map CIGI `EntityControl` (platform position) → `GpsDataPacket`.
///
/// CIGI uses degrees for lat/lon; Orion stores lat/lon in radians internally.
pub fn entity_control_to_gps_data(ec: &EntityControl) -> GpsDataPacket {
    use std::f64::consts::PI;
    let mut pkt = GpsDataPacket::default();
    pkt.latitude = ec.lat_or_x * PI / 180.0;
    pkt.longitude = ec.lon_or_y * PI / 180.0;
    pkt.altitude = ec.alt_or_z;
    pkt
}

/// Build a `GeopointCmdPacket` for geopoint mode.
///
/// `ac_coupling`/`noise` fields from `SensorControl` encode lat/lon as 0–1 fractions
/// covering −90°…+90° and −180°…+180° respectively.
pub fn sensor_control_to_geopoint_cmd(sc: &SensorControl) -> GeopointCmdPacket {
    use std::f64::consts::PI;
    let lat_deg = sc.ac_coupling as f64 * 180.0 - 90.0;
    let lon_deg = sc.noise as f64 * 360.0 - 180.0;
    let mut pkt = GeopointCmdPacket::default();
    pkt.target_lat = lat_deg * PI / 180.0;
    pkt.target_lon = lon_deg * PI / 180.0;
    pkt.target_alt = 0.0;
    pkt
}
