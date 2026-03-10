// Convert Orion telemetry → CIGI response packets.

use crate::cigi::messages::SensorExtendedResponse;
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode};

/// Map Orion `GeolocateTelemetryCorePacket` → CIGI `SensorExtendedResponse`.
///
/// gate_x_pos/gate_y_pos are pan/tilt in degrees per ICD.
/// entity_lat/lon are the Orion position fields converted from radians to degrees.
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
        gate_x_pos: telem.pan.to_degrees(),
        gate_y_pos: telem.tilt.to_degrees(),
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
