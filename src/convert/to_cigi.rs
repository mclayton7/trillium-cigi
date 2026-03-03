// Convert Orion telemetry → CIGI response packets.

use crate::cigi::messages::{SensorExtendedResponse, SensorResponse};
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode};

/// Map Orion `GeolocateTelemetryCorePacket` → CIGI `SensorResponse`.
///
/// pan/tilt (radians) are normalized by hfov/vfov and stored in gate_x_pos/gate_y_pos.
pub fn telemetry_to_sensor_response(
    telem: &GeolocateTelemetryCorePacket,
    view_id: u8,
    sensor_id: u8,
) -> SensorResponse {
    let hfov = if telem.hfov > 0.001 { telem.hfov } else { 1.0 };
    let vfov = if telem.vfov > 0.001 { telem.vfov } else { 1.0 };
    let gate_x_pos = telem.pan / hfov;
    let gate_y_pos = telem.tilt / vfov;
    let sensor_status = orion_mode_to_sensor_status(telem.mode);

    SensorResponse {
        view_id,
        sensor_id,
        sensor_status,
        gate_x_size: 0.1,
        gate_y_size: 0.1,
        gate_x_pos,
        gate_y_pos,
        frame_ctr: telem.system_time,
        entity_id_valid: false,
        entity_id: 0,
    }
}

/// Map Orion `GeolocateTelemetryCorePacket` → CIGI `SensorExtendedResponse`.
///
/// Adds entity lat/lon/alt from Orion position fields (stored in radians, converted to degrees).
pub fn telemetry_to_sensor_extended_response(
    telem: &GeolocateTelemetryCorePacket,
    view_id: u8,
    sensor_id: u8,
) -> SensorExtendedResponse {
    let base = telemetry_to_sensor_response(telem, view_id, sensor_id);
    let deg = 180.0 / std::f64::consts::PI;
    SensorExtendedResponse {
        view_id: base.view_id,
        sensor_id: base.sensor_id,
        sensor_status: base.sensor_status,
        gate_x_size: base.gate_x_size,
        gate_y_size: base.gate_y_size,
        gate_x_pos: base.gate_x_pos,
        gate_y_pos: base.gate_y_pos,
        frame_ctr: base.frame_ctr,
        entity_id_valid: base.entity_id_valid,
        entity_id: base.entity_id,
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
