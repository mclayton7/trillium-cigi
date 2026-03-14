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
