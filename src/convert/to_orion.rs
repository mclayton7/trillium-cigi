// Convert CIGI responses → Orion telemetry packets sent back to the Trillium controller.

use crate::cigi::messages::SensorExtendedResponse;
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode};
use crate::platform::PlatformState;

/// Map a CIGI `SensorExtendedResponse` (received from the scene generator) plus
/// the current `PlatformState` into an Orion `GeolocateTelemetryCorePacket`
/// to send back to the Trillium controller.
///
/// Field mapping:
/// | CIGI field          | Orion field   | Conversion      |
/// |---------------------|---------------|-----------------|
/// | entity_lat (deg)    | pos_lat (rad) | × π/180         |
/// | entity_lon (deg)    | pos_lon (rad) | × π/180         |
/// | entity_alt (m)      | pos_alt (m)   | direct          |
/// | sensor_status       | mode          | 0→Position, 1→Track, 3→Disabled |
///
/// Note: gate_x_pos/gate_y_pos are CIGI v3.3 tracking gate centroid positions,
/// NOT gimbal pan/tilt angles. Pan/tilt on the scene-generator path come from
/// the IG itself; the simulator fallback path populates them in `to_telemetry()`.
pub fn sensor_response_to_telemetry(
    resp: &SensorExtendedResponse,
    _platform: &PlatformState,
) -> GeolocateTelemetryCorePacket {
    let rad_per_deg = std::f64::consts::PI / 180.0;
    let mut pkt = GeolocateTelemetryCorePacket::default();
    // pan and tilt are left at default (0.0) — they are not derived from gate fields.
    pkt.pos_lat = resp.entity_lat * rad_per_deg;
    pkt.pos_lon = resp.entity_lon * rad_per_deg;
    pkt.pos_alt = resp.entity_alt;
    pkt.mode = match resp.sensor_status & 0x03 {
        1 => OrionMode::OrionModeTrack,
        3 => OrionMode::OrionModeDisabled,
        _ => OrionMode::OrionModePosition,
    };
    pkt
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cigi::messages::SensorExtendedResponse;
    use crate::platform::PlatformState;

    #[test]
    fn sensor_response_to_telemetry_basic() {
        let resp = SensorExtendedResponse {
            gate_x_pos: 0.25,  // tracking gate centroid, not pan
            gate_y_pos: -0.1,  // tracking gate centroid, not tilt
            entity_lat: 38.8977,
            entity_lon: -77.0365,
            entity_alt: 100.0,
            sensor_status: 0, // Searching/Active
            ..SensorExtendedResponse::default()
        };
        let telem = sensor_response_to_telemetry(&resp, &PlatformState::default());
        assert!((telem.pos_alt - 100.0).abs() < 0.1);
        assert_eq!(telem.mode, crate::orion::OrionMode::OrionModePosition);
    }

    #[test]
    fn sensor_response_does_not_extract_pan_tilt_from_gate() {
        let resp = SensorExtendedResponse {
            gate_x_pos: 0.3,
            gate_y_pos: -0.2,
            ..SensorExtendedResponse::default()
        };
        let telem = sensor_response_to_telemetry(&resp, &PlatformState::default());
        // Pan and tilt must remain at default (0.0), not derived from gate fields.
        assert_eq!(telem.pan, 0.0);
        assert_eq!(telem.tilt, 0.0);
    }

    #[test]
    fn sensor_response_tracking_status() {
        let resp = SensorExtendedResponse {
            sensor_status: 1, // Tracking
            ..SensorExtendedResponse::default()
        };
        let telem = sensor_response_to_telemetry(&resp, &PlatformState::default());
        assert_eq!(telem.mode, crate::orion::OrionMode::OrionModeTrack);
    }
}
