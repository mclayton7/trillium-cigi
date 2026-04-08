// STANAG 4586 Edition 2 Amendment 1 — DLI platform source.
//
// Joins a UDP multicast group and parses three message types:
//   101  Inertial States         — lat/lon/alt/attitude/velocity (primary source)
//   102  Air & Ground Relative   — airspeed/wind/altitude variants (parsed, no new state)
//   103  Body-Relative Sensed    — body-axis rates/accel (parsed, no new state)
//
// All fields are big-endian (MSB-first) per Section 1.7.1 of the standard.
//
// Wrapper layout (34 bytes before body, 4-byte checksum appended after body):
//   [0..10]   IDD Version (ASCII, null-padded; "9" for Ed2 Amend1)
//   [10..14]  Message Instance ID (u32)
//   [14..18]  Spare (u32)
//   [18..22]  Message Type (u32)
//   [22..26]  Message Length (u32) — body bytes only
//   [26..30]  Stream ID (u32)
//   [30..34]  Packet Sequence Number (u32)
//   [34..]    Message Data
//   [last 4]  Checksum (u32) — wrapping byte sum of all bytes before this field

use crate::platform::{PlatformSource, PlatformState};
use std::net::Ipv4Addr;
use tokio::sync::watch;

// ── Frame constants ────────────────────────────────────────────────────────

const HDR_LEN: usize = 34;
const CHKSUM_LEN: usize = 4;
const MIN_FRAME: usize = HDR_LEN + CHKSUM_LEN;

const MSG_INERTIAL: u32 = 101;
const MSG_AIR_GROUND: u32 = 102;
const MSG_BODY_SENSED: u32 = 103;

// Minimum body sizes (bytes) for each supported message.
const BODY_MIN_101: usize = 89;
const BODY_MIN_102: usize = 76;
const BODY_MIN_103: usize = 40;

// ── Checksum ───────────────────────────────────────────────────────────────

/// Verify the 4-byte wrapping byte-sum appended at the end of `frame`.
fn verify_checksum(frame: &[u8]) -> bool {
    let n = frame.len();
    if n < CHKSUM_LEN {
        return false;
    }
    let computed: u32 = frame[..n - CHKSUM_LEN]
        .iter()
        .fold(0u32, |acc, &b| acc.wrapping_add(b as u32));
    let wire = u32::from_be_bytes([frame[n - 4], frame[n - 3], frame[n - 2], frame[n - 1]]);
    computed == wire
}

// ── Parsed message enum ────────────────────────────────────────────────────

#[derive(Debug)]
enum Stanag4586Msg {
    /// Message 101 — Inertial States.  All platform state fields in one message.
    InertialStates {
        lat_rad: f64,
        lon_rad: f64,
        alt_m: f32,
        vel_north_m_s: f32,
        vel_east_m_s: f32,
        vel_down_m_s: f32,
        roll_rad: f32,
        pitch_rad: f32,
        yaw_rad: f32,
    },
    /// Message 102 — Air and Ground Relative States.  Parsed; no new PlatformState fields.
    AirGroundRelative,
    /// Message 103 — Body-Relative Sensed States.  Parsed; no new PlatformState fields.
    BodySensed,
}

// ── Frame parser ───────────────────────────────────────────────────────────

/// Parse one STANAG 4586 DLI wrapper from `buf[..len]`.
/// Returns `None` on length underrun, bad checksum, or filtered vehicle ID.
fn parse_frame(buf: &[u8], len: usize, vehicle_id_filter: i32) -> Option<Stanag4586Msg> {
    let buf = &buf[..len];
    if buf.len() < MIN_FRAME {
        return None;
    }

    let msg_type = u32::from_be_bytes(buf[18..22].try_into().ok()?);
    let body_len = u32::from_be_bytes(buf[22..26].try_into().ok()?) as usize;
    let frame_end = HDR_LEN + body_len + CHKSUM_LEN;

    if buf.len() < frame_end {
        return None;
    }
    if !verify_checksum(&buf[..frame_end]) {
        return None;
    }

    let body = &buf[HDR_LEN..HDR_LEN + body_len];

    // Vehicle ID sits at body[8..12] for all three message types.
    if body.len() < 16 {
        return None;
    }
    let vehicle_id = i32::from_be_bytes(body[8..12].try_into().ok()?);
    if vehicle_id_filter != 0 && vehicle_id != vehicle_id_filter {
        return None;
    }

    match msg_type {
        MSG_INERTIAL   => decode_101(body),
        MSG_AIR_GROUND => decode_102(body),
        MSG_BODY_SENSED => decode_103(body),
        _ => None,
    }
}

// ── Message decoders ───────────────────────────────────────────────────────

/// Message 101 body field offsets:
///   [0..8]   Time Stamp (f64)
///   [8..12]  Vehicle ID (i32)
///   [12..16] CUCS ID (i32)
///   [16..24] Latitude (f64, rad)
///   [24..32] Longitude (f64, rad)
///   [32..36] Altitude (f32, m)
///   [36]     Altitude Type (u8)
///   [37..41] U_Speed (f32, m/s north)
///   [41..45] V_Speed (f32, m/s east)
///   [45..49] W_Speed (f32, m/s down +ve)
///   [49..53] U_Accel, [53..57] V_Accel, [57..61] W_Accel
///   [61..65] Phi / roll (f32, rad)
///   [65..69] Theta / pitch (f32, rad)
///   [69..73] Psi / yaw (f32, rad)
///   [73..77] Phi_dot, [77..81] Theta_dot, [81..85] Psi_dot
///   [85..89] Magnetic Variation (f32, rad)
fn decode_101(body: &[u8]) -> Option<Stanag4586Msg> {
    if body.len() < BODY_MIN_101 {
        return None;
    }
    Some(Stanag4586Msg::InertialStates {
        lat_rad:      f64::from_be_bytes(body[16..24].try_into().ok()?),
        lon_rad:      f64::from_be_bytes(body[24..32].try_into().ok()?),
        alt_m:        f32::from_be_bytes(body[32..36].try_into().ok()?),
        vel_north_m_s: f32::from_be_bytes(body[37..41].try_into().ok()?),
        vel_east_m_s:  f32::from_be_bytes(body[41..45].try_into().ok()?),
        vel_down_m_s:  f32::from_be_bytes(body[45..49].try_into().ok()?),
        roll_rad:     f32::from_be_bytes(body[61..65].try_into().ok()?),
        pitch_rad:    f32::from_be_bytes(body[65..69].try_into().ok()?),
        yaw_rad:      f32::from_be_bytes(body[69..73].try_into().ok()?),
    })
}

/// Message 102 body field offsets:
///   [0..8]  Time Stamp, [8..12] Vehicle ID, [12..16] CUCS ID
///   [16..20] Angle of Attack, [20..24] Angle of Sideslip
///   [24..28] True Airspeed, [28..32] Indicated Airspeed
///   [32..36] Outside Air Temp, [36..40] U_Wind, [40..44] V_Wind
///   [44..48] Altimeter Setting, [48..52] Baro Alt, [52..56] Baro Alt Rate
///   [56..60] Pressure Alt, [60..64] AGL Alt, [64..68] WGS-84 Alt
///   [68..72] U_Ground (m/s north), [72..76] V_Ground (m/s east)
fn decode_102(body: &[u8]) -> Option<Stanag4586Msg> {
    if body.len() < BODY_MIN_102 {
        return None;
    }
    Some(Stanag4586Msg::AirGroundRelative)
}

/// Message 103 body field offsets:
///   [0..8]  Time Stamp, [8..12] Vehicle ID, [12..16] CUCS ID
///   [16..20] X_Body_Accel, [20..24] Y_Body_Accel, [24..28] Z_Body_Accel
///   [28..32] Roll_Rate, [32..36] Pitch_Rate, [36..40] Yaw_Rate
fn decode_103(body: &[u8]) -> Option<Stanag4586Msg> {
    if body.len() < BODY_MIN_103 {
        return None;
    }
    Some(Stanag4586Msg::BodySensed)
}

// ── Stanag4586Source ───────────────────────────────────────────────────────

pub struct Stanag4586Source {
    port: u16,
    multicast_group: String,
    vehicle_id_filter: i32,
}

impl Stanag4586Source {
    pub fn new(port: u16, multicast_group: String, vehicle_id_filter: i32) -> Self {
        Self { port, multicast_group, vehicle_id_filter }
    }
}

impl PlatformSource for Stanag4586Source {
    async fn run(self, tx: watch::Sender<PlatformState>) {
        // Parse the multicast group address.
        let group: Ipv4Addr = match self.multicast_group.parse() {
            Ok(a) => a,
            Err(e) => {
                eprintln!("[stanag4586] invalid multicast group '{}': {e}", self.multicast_group);
                return;
            }
        };
        if !group.is_multicast() {
            eprintln!("[stanag4586] '{}' is not a multicast address", self.multicast_group);
            return;
        }

        // Bind and join the multicast group using std socket, then hand to tokio.
        let std_sock = match std::net::UdpSocket::bind(format!("0.0.0.0:{}", self.port)) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[stanag4586] failed to bind 0.0.0.0:{}: {e}", self.port);
                return;
            }
        };
        if let Err(e) = std_sock.join_multicast_v4(&group, &Ipv4Addr::UNSPECIFIED) {
            eprintln!("[stanag4586] failed to join multicast {group}: {e}");
            return;
        }
        if let Err(e) = std_sock.set_nonblocking(true) {
            eprintln!("[stanag4586] set_nonblocking failed: {e}");
            return;
        }
        let socket = match tokio::net::UdpSocket::from_std(std_sock) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[stanag4586] tokio socket init failed: {e}");
                return;
            }
        };

        let filter_str = if self.vehicle_id_filter == 0 {
            "all vehicles".to_string()
        } else {
            format!("vehicle_id={}", self.vehicle_id_filter)
        };
        println!(
            "[stanag4586] joined {}:{} ({filter_str})",
            self.multicast_group, self.port
        );

        let mut buf = [0u8; 2048];
        loop {
            let Ok((len, _src)) = socket.recv_from(&mut buf).await else {
                continue;
            };
            if let Some(msg) = parse_frame(&buf, len, self.vehicle_id_filter) {
                if let Stanag4586Msg::InertialStates {
                    lat_rad, lon_rad, alt_m,
                    vel_north_m_s, vel_east_m_s, vel_down_m_s,
                    roll_rad, pitch_rad, yaw_rad,
                } = msg
                {
                    let _ = tx.send(PlatformState {
                        lat_rad,
                        lon_rad,
                        alt_m: alt_m as f64,
                        roll_rad,
                        pitch_rad,
                        yaw_rad,
                        vel_north_m_s,
                        vel_east_m_s,
                        vel_down_m_s,
                    });
                }
                // AirGroundRelative and BodySensed are parsed but don't update
                // PlatformState — they can be wired up to additional fields here
                // in the future.
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a complete STANAG 4586 DLI wrapper around `body` for `msg_type`.
    fn make_frame(msg_type: u32, body: &[u8]) -> Vec<u8> {
        let mut frame = Vec::with_capacity(HDR_LEN + body.len() + CHKSUM_LEN);
        // IDD Version: "9" null-padded to 10 bytes
        let mut idd = [0u8; 10];
        idd[0] = b'9';
        frame.extend_from_slice(&idd);
        frame.extend_from_slice(&1u32.to_be_bytes());           // Message Instance ID
        frame.extend_from_slice(&0u32.to_be_bytes());           // Spare
        frame.extend_from_slice(&msg_type.to_be_bytes());       // Message Type
        frame.extend_from_slice(&(body.len() as u32).to_be_bytes()); // Message Length
        frame.extend_from_slice(&0u32.to_be_bytes());           // Stream ID
        frame.extend_from_slice(&0xFFFF_FFFFu32.to_be_bytes()); // Sequence Number
        frame.extend_from_slice(body);
        // Checksum: wrapping byte sum of everything so far
        let sum: u32 = frame.iter().fold(0u32, |acc, &b| acc.wrapping_add(b as u32));
        frame.extend_from_slice(&sum.to_be_bytes());
        frame
    }

    /// Build an 89-byte message 101 body with the given fields; remaining fields zeroed.
    fn msg101_body(
        vehicle_id: i32,
        lat: f64, lon: f64, alt: f32,
        u_spd: f32, v_spd: f32, w_spd: f32,
        roll: f32, pitch: f32, yaw: f32,
    ) -> Vec<u8> {
        let mut b = vec![0u8; BODY_MIN_101];
        // [0..8]   Time Stamp — leave zero
        b[8..12].copy_from_slice(&vehicle_id.to_be_bytes());   // Vehicle ID
        // [12..16] CUCS ID — leave zero
        b[16..24].copy_from_slice(&lat.to_be_bytes());
        b[24..32].copy_from_slice(&lon.to_be_bytes());
        b[32..36].copy_from_slice(&alt.to_be_bytes());
        b[36] = 3; // Altitude Type: WGS-84
        b[37..41].copy_from_slice(&u_spd.to_be_bytes());
        b[41..45].copy_from_slice(&v_spd.to_be_bytes());
        b[45..49].copy_from_slice(&w_spd.to_be_bytes());
        // [49..61] Accelerations — leave zero
        b[61..65].copy_from_slice(&roll.to_be_bytes());
        b[65..69].copy_from_slice(&pitch.to_be_bytes());
        b[69..73].copy_from_slice(&yaw.to_be_bytes());
        b
    }

    fn msg102_body(vehicle_id: i32) -> Vec<u8> {
        let mut b = vec![0u8; BODY_MIN_102];
        b[8..12].copy_from_slice(&vehicle_id.to_be_bytes());
        b
    }

    fn msg103_body(vehicle_id: i32) -> Vec<u8> {
        let mut b = vec![0u8; BODY_MIN_103];
        b[8..12].copy_from_slice(&vehicle_id.to_be_bytes());
        b
    }

    #[test]
    fn test_parse_inertial_states() {
        let lat = 0.6614f64; // ~37.9°
        let lon = 0.4887f64; // ~28.0°
        let body = msg101_body(1, lat, lon, 500.0, 10.0, -5.0, 1.0, 0.1, -0.1, 1.57);
        let frame = make_frame(MSG_INERTIAL, &body);
        let msg = parse_frame(&frame, frame.len(), 0).expect("should parse");
        match msg {
            Stanag4586Msg::InertialStates {
                lat_rad, lon_rad, alt_m,
                vel_north_m_s, vel_east_m_s, vel_down_m_s,
                roll_rad, pitch_rad, yaw_rad,
            } => {
                assert!((lat_rad - lat).abs() < 1e-10, "lat mismatch");
                assert!((lon_rad - lon).abs() < 1e-10, "lon mismatch");
                assert!((alt_m - 500.0).abs() < 0.001, "alt mismatch");
                assert!((vel_north_m_s - 10.0).abs() < 1e-5, "vel_north");
                assert!((vel_east_m_s - (-5.0)).abs() < 1e-5, "vel_east");
                assert!((vel_down_m_s - 1.0).abs() < 1e-5, "vel_down");
                assert!((roll_rad - 0.1).abs() < 1e-5, "roll");
                assert!((pitch_rad - (-0.1)).abs() < 1e-5, "pitch");
                assert!((yaw_rad - 1.57).abs() < 1e-5, "yaw");
            }
            other => panic!("unexpected variant: {other:?}"),
        }
    }

    #[test]
    fn test_parse_air_ground_relative() {
        let body = msg102_body(1);
        let frame = make_frame(MSG_AIR_GROUND, &body);
        let msg = parse_frame(&frame, frame.len(), 0).expect("should parse");
        assert!(matches!(msg, Stanag4586Msg::AirGroundRelative));
    }

    #[test]
    fn test_parse_body_sensed() {
        let body = msg103_body(1);
        let frame = make_frame(MSG_BODY_SENSED, &body);
        let msg = parse_frame(&frame, frame.len(), 0).expect("should parse");
        assert!(matches!(msg, Stanag4586Msg::BodySensed));
    }

    #[test]
    fn test_checksum_rejects_corrupt_frame() {
        let body = msg101_body(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let mut frame = make_frame(MSG_INERTIAL, &body);
        // Flip a byte in the body
        frame[HDR_LEN] ^= 0xFF;
        assert!(parse_frame(&frame, frame.len(), 0).is_none(), "corrupt frame must be rejected");
    }

    #[test]
    fn test_vehicle_id_filter() {
        let body = msg101_body(42, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let frame = make_frame(MSG_INERTIAL, &body);

        // Accept all (filter = 0)
        assert!(parse_frame(&frame, frame.len(), 0).is_some());
        // Accept matching ID
        assert!(parse_frame(&frame, frame.len(), 42).is_some());
        // Reject non-matching ID
        assert!(parse_frame(&frame, frame.len(), 7).is_none());
    }

    #[test]
    fn test_short_frame_rejected() {
        let frame = vec![0u8; MIN_FRAME - 1];
        assert!(parse_frame(&frame, frame.len(), 0).is_none());
    }
}
