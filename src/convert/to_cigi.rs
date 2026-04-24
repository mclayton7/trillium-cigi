// Convert Orion commands / platform state → CIGI packets sent to the scene generator.
//
// Payload-agnostic CIGI builders (`make_ig_control`, `platform_to_entity_control`,
// `build_view_control`, `build_wire_sensor_control`) live in
// `sim_core::cigi::build`. This file holds only the Orion-specific adapters.

use sim_core::cigi::build::build_view_control;
use sim_core::cigi::messages::{SensorControl, SensorExtendedResponse, ViewControl};
use crate::orion::{GeolocateTelemetryCorePacket, OrionCmdPacket, OrionMode};
use sim_core::geo::GimbalMount;

// ── Simulator compatibility ────────────────────────────────────────────────

/// Map Orion `GeolocateTelemetryCorePacket` → CIGI `SensorExtendedResponse`.
///
/// Called by `GimbalSimulator::to_sensor_extended_response()` in fallback mode.
/// `settled` reports whether Position/Geopoint has converged on its target
/// (error < 0.01 rad on both axes); `track_active` is only meaningful in
/// `OrionModeTrack` and drives Tracking vs Breaklock for that mode.
///
/// Per CIGI v3.3, `gate_x_pos`/`gate_y_pos` carry the image-plane gate centroid
/// (not gimbal pan/tilt). The base conversion sets them to 0.0; the simulator
/// overrides them for track mode.
pub fn telemetry_to_sensor_extended_response(
    telem: &GeolocateTelemetryCorePacket,
    view_id: u16,
    sensor_id: u8,
    settled: bool,
    track_active: bool,
) -> SensorExtendedResponse {
    let deg = 180.0 / std::f64::consts::PI;
    SensorExtendedResponse {
        view_id,
        sensor_id,
        sensor_status: orion_mode_to_sensor_status(telem.mode, settled, track_active),
        // Default gate size; the simulator overrides this with a value derived
        // from the current FOV and `track_gate_size_deg`.
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

fn orion_mode_to_sensor_status(mode: OrionMode, settled: bool, track_active: bool) -> u8 {
    match mode {
        OrionMode::OrionModePosition
        | OrionMode::OrionModeGeopoint
        | OrionMode::OrionModePositionNoLimits => {
            if settled { 0 } else { 2 } // Locked vs Slewing
        }
        OrionMode::OrionModeRate => 2,                                 // Slewing
        OrionMode::OrionModeTrack => if track_active { 1 } else { 3 }, // Tracking / Breaklock
        _ => 3,                                                        // Breaklock
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
pub fn orion_cmd_to_sensor_control(
    cmd: &OrionCmdPacket,
    camera_index: i8,
    zoom_level: f32,
) -> SensorControl {
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

    // In Geopoint mode `ac_coupling` carries the lat encoding, so leave it at
    // its default. In Position/Rate modes, forward the zoom level.
    let ac_coupling = match cmd.cmd.mode {
        OrionMode::OrionModeGeopoint => 0.0,
        _ => zoom_level.clamp(0.0, 1.0),
    };

    SensorControl {
        // Three cameras (0=EO wide, 1=EO narrow, 2=IR) populate `camera_table`.
        // Out-of-range inputs are clamped rather than rejected to keep the
        // wire packet well-formed; an upstream validation layer should catch
        // bad indices before they reach this conversion.
        sensor_id: camera_index.clamp(0, 2) as u8,
        view_id: 0,
        sensor_state,
        track_mode,
        gain,
        level,
        ac_coupling,
        ..SensorControl::default()
    }
}

// ══════════════════════════════════════════════════════════════════════════
// Wire-side CIGI packets (Host → IG scene generator)
// ══════════════════════════════════════════════════════════════════════════
//
// Important: the existing `orion_cmd_to_sensor_control` above is an **internal**
// protocol between this bridge and its own fallback `GimbalSimulator` — it
// packs pan/tilt targets into `gain`/`level` and zoom into `ac_coupling`. The
// scene generator (camera-simulator, UE5) does not speak that protocol: it
// reads `Gain` as a FOV preset index (see Step 1 verification) and expects
// gimbal pose on CIGI ViewControl (type 16) attached to the platform entity.
//
// The helpers below produce packets for the **on-the-wire** path, using CIGI
// semantics that match the camera-simulator CCL decoder.

/// Build a CIGI ViewControl (type 16) that attaches the view to the platform
/// entity, positions its origin at the gimbal base (mount translation), and
/// points it with the composed `R_mount · R_gimbal` rotation (mount boresight
/// offset + current gimbal pan/tilt), expressed in entity body frame as
/// ZYX Euler angles.
///
/// - `view_id`: CIGI view identifier (single-sensor scenes may use 1)
/// - `entity_id`: CIGI entity the view is attached to (the platform)
///
/// The pan/tilt used come from the Orion command in Position mode; for other
/// modes (Rate, Track, Geopoint) the Rust-side simulator is authoritative —
/// callers should read current `GimbalSimulator::pan/tilt` instead of using
/// this single-shot conversion. See `main.rs` for the tick-loop wiring.
pub fn orion_cmd_to_view_control(
    cmd: &OrionCmdPacket,
    mount: &GimbalMount,
    view_id: u16,
    entity_id: u16,
) -> ViewControl {
    match cmd.cmd.mode {
        OrionMode::OrionModePosition => {
            build_view_control(cmd.cmd.target[0], cmd.cmd.target[1], mount, view_id, entity_id)
        }
        _ => build_view_control(0.0, 0.0, mount, view_id, entity_id),
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::orion::{GeolocateTelemetryCorePacket, OrionCmdPacket, OrionMode};
    use std::f32::consts::PI as PI32;
    use std::f64::consts::PI as PI64;

    // ── telemetry_to_sensor_extended_response ────────────────────────────────

    #[test]
    fn telem_gate_pos_boresighted() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.pan = PI32 / 4.0;
        t.tilt = -PI32 / 6.0;
        let r = telemetry_to_sensor_extended_response(&t, 0, 0, false, false);
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
        let r = telemetry_to_sensor_extended_response(&t, 0, 0, false, false);
        assert!((r.entity_lat - 90.0).abs() < 1e-10, "entity_lat={}", r.entity_lat);
        assert!((r.entity_lon - (-45.0)).abs() < 1e-10, "entity_lon={}", r.entity_lon);
    }

    #[test]
    fn telem_alt_passes_through() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.pos_alt = 1234.5;
        let r = telemetry_to_sensor_extended_response(&t, 0, 0, false, false);
        assert!((r.entity_alt - 1234.5).abs() < 1e-9);
    }

    #[test]
    fn telem_view_and_sensor_id_passed_through() {
        let r = telemetry_to_sensor_extended_response(&GeolocateTelemetryCorePacket::default(), 7, 3, false, false);
        assert_eq!(r.view_id, 7);
        assert_eq!(r.sensor_id, 3);
    }

    #[test]
    fn telem_sensor_status_track_active() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModeTrack;
        // track_active=true → Tracking (1)
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, false, true).sensor_status, 1);
    }

    #[test]
    fn telem_sensor_status_track_lost_is_breaklock() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModeTrack;
        // Track mode with track_active=false → Breaklock (3)
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, false, false).sensor_status, 3);
    }

    #[test]
    fn telem_sensor_status_disabled() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModeDisabled;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, false, false).sensor_status, 3);
    }

    #[test]
    fn telem_sensor_status_position_settled() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModePosition;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, true, false).sensor_status, 0);
    }

    #[test]
    fn telem_sensor_status_position_slewing() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModePosition;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, false, false).sensor_status, 2);
    }

    #[test]
    fn telem_sensor_status_rate_always_slewing() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.mode = OrionMode::OrionModeRate;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, false, false).sensor_status, 2);
        // Even with settled=true, rate mode is always slewing.
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, true, false).sensor_status, 2);
    }

    #[test]
    fn telem_frame_ctr_from_system_time() {
        let mut t = GeolocateTelemetryCorePacket::default();
        t.system_time = 9876;
        assert_eq!(telemetry_to_sensor_extended_response(&t, 0, 0, false, false).frame_ctr, 9876);
    }

    #[test]
    fn telem_gate_sizes_are_20() {
        let r = telemetry_to_sensor_extended_response(&GeolocateTelemetryCorePacket::default(), 0, 0, false, false);
        assert_eq!(r.gate_x_size, 20);
        assert_eq!(r.gate_y_size, 20);
    }

    // Note: `make_ig_control`, `platform_to_entity_control`, `build_view_control`,
    // and `build_wire_sensor_control` moved to `sim_core::cigi::build` in
    // Phase 3c Item 2. Their unit tests now live in
    // `sim-core/src/cigi/build.rs`.

    // ── orion_cmd_to_sensor_control ──────────────────────────────────────────

    fn make_cmd(mode: OrionMode, target: [f32; 2]) -> OrionCmdPacket {
        let mut cmd = OrionCmdPacket::default();
        cmd.cmd.mode = mode;
        cmd.cmd.target = target;
        cmd
    }

    #[test]
    fn sc_disabled_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeDisabled, [0.0, 0.0]), 0, 0.0);
        assert_eq!(sc.sensor_state, 0);
    }

    #[test]
    fn sc_fault_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeFault, [0.0, 0.0]), 0, 0.0);
        assert_eq!(sc.sensor_state, 0);
    }

    #[test]
    fn sc_position_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [PI32 / 2.0, 0.0]), 0, 0.0);
        assert_eq!(sc.sensor_state, 1);
        assert_eq!(sc.track_mode, 0);
        assert!((sc.gain - 0.75).abs() < 1e-5, "gain={}", sc.gain);
        assert!((sc.level - 0.5).abs() < 1e-5, "level={}", sc.level);
    }

    #[test]
    fn sc_position_centre_is_half() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [0.0, 0.0]), 0, 0.0);
        assert!((sc.gain - 0.5).abs() < 1e-5);
        assert!((sc.level - 0.5).abs() < 1e-5);
    }

    #[test]
    fn sc_position_full_range() {
        let sc_max = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [PI32, 0.0]), 0, 0.0);
        let sc_min = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModePosition, [-PI32, 0.0]), 0, 0.0);
        assert!((sc_max.gain - 1.0).abs() < 1e-5);
        assert!((sc_min.gain - 0.0).abs() < 1e-5);
    }

    #[test]
    fn sc_rate_mode() {
        let max = crate::convert::MAX_SLEW_RATE;
        let sc_pos = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeRate, [max, 0.0]), 0, 0.0);
        let sc_neg = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeRate, [-max, 0.0]), 0, 0.0);
        let sc_zero = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeRate, [0.0, 0.0]), 0, 0.0);
        assert_eq!(sc_pos.sensor_state, 1);
        assert_eq!(sc_pos.track_mode, 0x01);
        assert!((sc_pos.gain - 1.0).abs() < 1e-5);
        assert!((sc_neg.gain - 0.0).abs() < 1e-5);
        assert!((sc_zero.gain - 0.5).abs() < 1e-5);
    }

    #[test]
    fn sc_track_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeTrack, [0.0, 0.0]), 0, 0.0);
        assert_eq!(sc.sensor_state, 2);
    }

    #[test]
    fn sc_geopoint_mode() {
        let sc = orion_cmd_to_sensor_control(&make_cmd(OrionMode::OrionModeGeopoint, [0.0, 0.0]), 0, 0.0);
        assert_eq!(sc.sensor_state, 4);
    }

    // ── orion_cmd_to_view_control / build_view_control ────────────────────

    #[test]
    fn view_control_position_mode_zero_mount_passes_pan_tilt_through() {
        // Pan = 30°, tilt = -15° (negative depression = look up 15°). With zero
        // mount, ViewControl yaw should equal pan and pitch should equal +15°
        // (nose-up corresponds to tilt = -depression).
        let cmd = make_cmd(
            OrionMode::OrionModePosition,
            [30.0_f32.to_radians(), -15.0_f32.to_radians()],
        );
        let vc = orion_cmd_to_view_control(&cmd, &GimbalMount::default(), 1, 1);
        assert_eq!(vc.view_id, 1);
        assert_eq!(vc.entity_id, 1);
        assert!(vc.x_off_en && vc.y_off_en && vc.z_off_en);
        assert!(vc.roll_en && vc.pitch_en && vc.yaw_en);
        assert!((vc.yaw_deg - 30.0).abs() < 1e-4, "yaw {}", vc.yaw_deg);
        assert!((vc.pitch_deg - 15.0).abs() < 1e-4, "pitch {}", vc.pitch_deg);
        assert!(vc.roll_deg.abs() < 1e-4, "roll {}", vc.roll_deg);
    }

    #[test]
    fn view_control_mount_yaw_offset_adds_to_pan() {
        // 3° mount yaw offset + 10° pan → composed yaw = 13°.
        let cmd = make_cmd(OrionMode::OrionModePosition, [10.0_f32.to_radians(), 0.0]);
        let mount = GimbalMount {
            translation_body_m: [0.0; 3],
            rotation_body_rad: [0.0, 0.0, 3.0_f64.to_radians()],
        };
        let vc = orion_cmd_to_view_control(&cmd, &mount, 1, 1);
        assert!((vc.yaw_deg - 13.0).abs() < 1e-4, "yaw {}", vc.yaw_deg);
        assert!(vc.pitch_deg.abs() < 1e-4);
    }

    #[test]
    fn view_control_mount_translation_emitted_in_offsets() {
        let cmd = make_cmd(OrionMode::OrionModePosition, [0.0, 0.0]);
        let mount = GimbalMount {
            translation_body_m: [0.8, -0.1, 0.25],
            rotation_body_rad: [0.0; 3],
        };
        let vc = orion_cmd_to_view_control(&cmd, &mount, 7, 3);
        assert_eq!(vc.view_id, 7);
        assert_eq!(vc.entity_id, 3);
        assert!((vc.x_off_m - 0.8).abs() < 1e-6);
        assert!((vc.y_off_m + 0.1).abs() < 1e-6);
        assert!((vc.z_off_m - 0.25).abs() < 1e-6);
    }

    #[test]
    fn view_control_non_position_mode_emits_zero_pan_tilt() {
        // Rate / Track / Geopoint don't carry raw pan/tilt in target[0..2].
        // orion_cmd_to_view_control zeros pan/tilt; callers should use
        // build_view_control with sim state for those modes.
        let cmd = make_cmd(OrionMode::OrionModeRate, [1.0, 0.5]);
        let vc = orion_cmd_to_view_control(&cmd, &GimbalMount::default(), 1, 1);
        assert!(vc.yaw_deg.abs() < 1e-4);
        assert!(vc.pitch_deg.abs() < 1e-4);
    }

}
