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
/// | gate_x_pos (deg)    | pan (rad)     | × π/180         |
/// | gate_y_pos (deg)    | tilt (rad)    | × π/180         |
/// | entity_lat (deg)    | pos_lat (rad) | × π/180         |
/// | entity_lon (deg)    | pos_lon (rad) | × π/180         |
/// | entity_alt (m)      | pos_alt (m)   | direct          |
/// | sensor_status       | mode          | 0→Position, 1→Track, 3→Disabled |
pub fn sensor_response_to_telemetry(
    resp: &SensorExtendedResponse,
    _platform: &PlatformState,
) -> GeolocateTelemetryCorePacket {
    let rad_per_deg = std::f64::consts::PI / 180.0;
    let mut pkt = GeolocateTelemetryCorePacket::default();
    pkt.pan = resp.gate_x_pos.to_radians();
    pkt.tilt = resp.gate_y_pos.to_radians();
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

/// Map CIGI `SensorControl` → `OrionCmdPacket` (kept for reference / testing).
///
/// Used in the original simulator-only mode when forwarding CIGI commands to
/// real Orion hardware over a TCP bridge.  No longer called in the bridge
/// architecture (OrionCmdPacket arrives directly from the Trillium controller),
/// but retained so that existing tests continue to compile.
#[allow(dead_code)]
pub fn sensor_control_to_orion_cmd(
    sc: &crate::cigi::messages::SensorControl,
) -> crate::orion::OrionCmdPacket {
    use std::f32::consts::PI;
    let max_rate = crate::convert::MAX_SLEW_RATE;

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
        OrionMode::OrionModeRate => [
            (sc.gain * 2.0 - 1.0) * max_rate,
            (sc.level * 2.0 - 1.0) * max_rate,
        ],
        _ => [
            (sc.gain * 2.0 - 1.0) * PI,
            (sc.level * 2.0 - 1.0) * PI,
        ],
    };

    let mut pkt = crate::orion::OrionCmdPacket::default();
    pkt.cmd.target = target;
    pkt.cmd.mode = mode;
    pkt.cmd.stabilized = 1;
    pkt.cmd.impulse_time = 0.0;
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
            gate_x_pos: 45.0,  // degrees pan
            gate_y_pos: -10.0, // degrees tilt
            entity_lat: 38.8977,
            entity_lon: -77.0365,
            entity_alt: 100.0,
            sensor_status: 0, // Searching/Active
            ..SensorExtendedResponse::default()
        };
        let platform = PlatformState {
            lat_rad: 0.0,
            lon_rad: 0.0,
            alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
        };
        let telem = sensor_response_to_telemetry(&resp, &platform);
        assert!((telem.pan - 45.0_f32.to_radians()).abs() < 1e-5);
        assert!((telem.tilt - (-10.0_f32).to_radians()).abs() < 1e-5);
        assert!((telem.pos_alt - 100.0).abs() < 0.1);
        assert_eq!(telem.mode, crate::orion::OrionMode::OrionModePosition);
    }

    #[test]
    fn sensor_response_tracking_status() {
        let resp = SensorExtendedResponse {
            sensor_status: 1, // Tracking
            ..SensorExtendedResponse::default()
        };
        let platform = PlatformState {
            lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0,
            roll_rad: 0.0, pitch_rad: 0.0, yaw_rad: 0.0,
        };
        let telem = sensor_response_to_telemetry(&resp, &platform);
        assert_eq!(telem.mode, crate::orion::OrionMode::OrionModeTrack);
    }
}
