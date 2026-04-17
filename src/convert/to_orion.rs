// Convert CIGI responses → Orion telemetry packets sent back to the Trillium controller.

use sim_core::cigi::messages::SensorExtendedResponse;
use crate::orion::{GeolocateTelemetryCorePacket, OrionMode};
use sim_core::platform::PlatformState;

/// Map a CIGI `SensorExtendedResponse` (received from the scene generator) plus
/// the current `PlatformState` into an Orion `GeolocateTelemetryCorePacket`
/// to send back to the Trillium controller.
///
/// Per CIGI v3.3, `gate_x_pos`/`gate_y_pos` are the image-plane tracking gate
/// centroid position, not gimbal pan/tilt. Pan/tilt are carried on the Orion
/// TCP telemetry path (from `to_telemetry()`) and are not recovered here.
///
/// Field mapping:
/// | CIGI field          | Orion field   | Conversion      |
/// |---------------------|---------------|-----------------|
/// | entity_lat (deg)    | pos_lat (rad) | × π/180         |
/// | entity_lon (deg)    | pos_lon (rad) | × π/180         |
/// | entity_alt (m)      | pos_alt (m)   | direct          |
/// | sensor_status       | mode          | 0 Locked → Position, 1 → Track, 2 Slewing → Position, 3 → Disabled |
pub fn sensor_response_to_telemetry(
    resp: &SensorExtendedResponse,
    _platform: &PlatformState,
) -> GeolocateTelemetryCorePacket {
    let rad_per_deg = std::f64::consts::PI / 180.0;
    let mut pkt = GeolocateTelemetryCorePacket::default();
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
    use sim_core::cigi::messages::SensorExtendedResponse;
    use sim_core::platform::PlatformState;

    #[test]
    fn sensor_response_to_telemetry_basic() {
        let resp = SensorExtendedResponse {
            entity_lat: 38.8977,
            entity_lon: -77.0365,
            entity_alt: 100.0,
            sensor_status: 0, // Locked
            ..SensorExtendedResponse::default()
        };
        let telem = sensor_response_to_telemetry(&resp, &PlatformState::default());
        assert!((telem.pos_alt - 100.0).abs() < 0.1);
        assert_eq!(telem.mode, crate::orion::OrionMode::OrionModePosition);
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
