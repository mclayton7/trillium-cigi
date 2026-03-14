// MAVLink platform source — hand-written MAVLink v1/v2 frame parser.
//
// Handles two message types:
//   ATTITUDE (msg_id=30)              → roll, pitch, yaw (f32 rad)
//   GLOBAL_POSITION_INT (msg_id=33)   → lat/lon (i32 degE7), alt (i32 mm), vx/vy/vz (i16 cm/s)
//
// Frames with bad X.25 CRC are silently dropped.
// If system_id == 0, all vehicles are accepted; otherwise filtered to that sysid.

use crate::config::Config;
use crate::platform::{PlatformSource, PlatformState};
use tokio::net::UdpSocket;
use tokio::sync::watch;

// ── MAVLink message IDs ────────────────────────────────────────────────────

const MSG_ATTITUDE: u32 = 30;
const MSG_GLOBAL_POSITION_INT: u32 = 33;

// CRC_EXTRA values per MAVLink spec
const CRC_EXTRA_ATTITUDE: u8 = 39;
const CRC_EXTRA_GLOBAL_POSITION_INT: u8 = 104;

// ── X.25 CRC ──────────────────────────────────────────────────────────────

fn x25_crc(data: &[u8]) -> u16 {
    data.iter().fold(0xFFFF_u16, |crc, &b| x25_crc_accumulate(crc, b))
}

// ── Parsed message types ───────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct AttitudeData {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

#[derive(Debug, Clone)]
struct GlobalPosData {
    lat_rad: f64,
    lon_rad: f64,
    alt_m: f64,
    vel_north_m_s: f32,
    vel_east_m_s: f32,
    vel_down_m_s: f32,
}

#[derive(Debug)]
enum MavMsg {
    Attitude(AttitudeData),
    GlobalPositionInt(GlobalPosData),
}

// ── Frame parsing ──────────────────────────────────────────────────────────

/// Parse all valid MAVLink v1/v2 frames from `buf` (up to `len` bytes),
/// filtering to `system_id` (0 = accept all), and returning decoded messages.
fn parse_frames(buf: &[u8], len: usize, system_id: u8) -> Vec<MavMsg> {
    let buf = &buf[..len];
    let mut msgs = Vec::new();
    let mut i = 0;

    while i < buf.len() {
        let magic = buf[i];
        match magic {
            0xFE => {
                // MAVLink v1: [magic][len][seq][sysid][compid][msgid][payload...][crc_lo][crc_hi]
                if i + 6 > buf.len() {
                    break;
                }
                let payload_len = buf[i + 1] as usize;
                let frame_len = 6 + payload_len + 2;
                if i + frame_len > buf.len() {
                    break;
                }
                let sysid = buf[i + 3];
                let msg_id = buf[i + 5] as u32;

                // CRC covers bytes 1..frame_len-2 plus CRC_EXTRA
                let crc_extra = crc_extra_for(msg_id);
                if let Some(extra) = crc_extra {
                    let crc_data = &buf[i + 1..i + frame_len - 2];
                    let mut crc = x25_crc(crc_data);
                    crc = x25_crc_accumulate(crc, extra);
                    let wire_crc = u16::from_le_bytes([buf[i + frame_len - 2], buf[i + frame_len - 1]]);
                    if crc == wire_crc && (system_id == 0 || sysid == system_id) {
                        let payload = &buf[i + 6..i + 6 + payload_len];
                        if let Some(msg) = decode_payload(msg_id, payload) {
                            msgs.push(msg);
                        }
                    }
                }
                i += frame_len;
            }
            0xFD => {
                // MAVLink v2: [magic][len][incompat][compat][seq][sysid][compid][msgid(3B)][payload...][crc_lo][crc_hi]
                // (no signature support — incompat_flags must be 0)
                if i + 10 > buf.len() {
                    break;
                }
                let payload_len = buf[i + 1] as usize;
                let incompat_flags = buf[i + 2];
                if incompat_flags != 0 {
                    // Signed frame or other extension — skip
                    i += 1;
                    continue;
                }
                let frame_len = 10 + payload_len + 2;
                if i + frame_len > buf.len() {
                    break;
                }
                let sysid = buf[i + 5];
                let msg_id = u32::from_le_bytes([buf[i + 7], buf[i + 8], buf[i + 9], 0]);

                let crc_extra = crc_extra_for(msg_id);
                if let Some(extra) = crc_extra {
                    let crc_data = &buf[i + 1..i + frame_len - 2];
                    let mut crc = x25_crc(crc_data);
                    crc = x25_crc_accumulate(crc, extra);
                    let wire_crc = u16::from_le_bytes([buf[i + frame_len - 2], buf[i + frame_len - 1]]);
                    if crc == wire_crc && (system_id == 0 || sysid == system_id) {
                        let payload = &buf[i + 10..i + 10 + payload_len];
                        if let Some(msg) = decode_payload(msg_id, payload) {
                            msgs.push(msg);
                        }
                    }
                }
                i += frame_len;
            }
            _ => {
                i += 1;
            }
        }
    }
    msgs
}

fn crc_extra_for(msg_id: u32) -> Option<u8> {
    match msg_id {
        MSG_ATTITUDE => Some(CRC_EXTRA_ATTITUDE),
        MSG_GLOBAL_POSITION_INT => Some(CRC_EXTRA_GLOBAL_POSITION_INT),
        _ => None,
    }
}

fn x25_crc_accumulate(crc: u16, byte: u8) -> u16 {
    let mut tmp = byte ^ (crc as u8);
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4)
}

fn decode_payload(msg_id: u32, payload: &[u8]) -> Option<MavMsg> {
    match msg_id {
        MSG_ATTITUDE => decode_attitude(payload),
        MSG_GLOBAL_POSITION_INT => decode_global_position_int(payload),
        _ => None,
    }
}

/// ATTITUDE payload (28 bytes):
///   [0..4]   time_boot_ms (u32)
///   [4..8]   roll (f32 rad)
///   [8..12]  pitch (f32 rad)
///   [12..16] yaw (f32 rad)
///   [16..20] rollspeed (f32 rad/s)
///   [20..24] pitchspeed (f32 rad/s)
///   [24..28] yawspeed (f32 rad/s)
fn decode_attitude(p: &[u8]) -> Option<MavMsg> {
    if p.len() < 28 {
        return None;
    }
    Some(MavMsg::Attitude(AttitudeData {
        roll: f32::from_le_bytes(p[4..8].try_into().ok()?),
        pitch: f32::from_le_bytes(p[8..12].try_into().ok()?),
        yaw: f32::from_le_bytes(p[12..16].try_into().ok()?),
    }))
}

/// GLOBAL_POSITION_INT payload (28 bytes):
///   [0..4]   time_boot_ms (u32)
///   [4..8]   lat (i32 degE7)
///   [8..12]  lon (i32 degE7)
///   [12..16] alt (i32 mm MSL)
///   [16..20] relative_alt (i32 mm)
///   [20..22] vx (i16 cm/s N)
///   [22..24] vy (i16 cm/s E)
///   [24..26] vz (i16 cm/s D)
///   [26..28] hdg (u16 cdeg)
fn decode_global_position_int(p: &[u8]) -> Option<MavMsg> {
    if p.len() < 28 {
        return None;
    }
    let lat_dege7 = i32::from_le_bytes(p[4..8].try_into().ok()?);
    let lon_dege7 = i32::from_le_bytes(p[8..12].try_into().ok()?);
    let alt_mm = i32::from_le_bytes(p[12..16].try_into().ok()?);
    let vx_cm_s = i16::from_le_bytes(p[20..22].try_into().ok()?);
    let vy_cm_s = i16::from_le_bytes(p[22..24].try_into().ok()?);
    let vz_cm_s = i16::from_le_bytes(p[24..26].try_into().ok()?);

    Some(MavMsg::GlobalPositionInt(GlobalPosData {
        lat_rad: (lat_dege7 as f64 * 1e-7).to_radians(),
        lon_rad: (lon_dege7 as f64 * 1e-7).to_radians(),
        alt_m: alt_mm as f64 * 1e-3,
        vel_north_m_s: vx_cm_s as f32 * 0.01,
        vel_east_m_s: vy_cm_s as f32 * 0.01,
        vel_down_m_s: vz_cm_s as f32 * 0.01,
    }))
}

// ── PartialState ───────────────────────────────────────────────────────────

/// Accumulates attitude and position independently.
/// Emits a complete PlatformState only once both have been received.
#[derive(Default)]
struct PartialState {
    attitude: Option<AttitudeData>,
    position: Option<GlobalPosData>,
}

impl PartialState {
    fn apply(&mut self, msg: MavMsg) {
        match msg {
            MavMsg::Attitude(a) => self.attitude = Some(a),
            MavMsg::GlobalPositionInt(g) => self.position = Some(g),
        }
    }

    fn complete(&self) -> Option<PlatformState> {
        let a = self.attitude.as_ref()?;
        let g = self.position.as_ref()?;
        Some(PlatformState {
            lat_rad: g.lat_rad,
            lon_rad: g.lon_rad,
            alt_m: g.alt_m,
            roll_rad: a.roll,
            pitch_rad: a.pitch,
            yaw_rad: a.yaw,
            vel_north_m_s: g.vel_north_m_s,
            vel_east_m_s: g.vel_east_m_s,
            vel_down_m_s: g.vel_down_m_s,
        })
    }
}

// ── MavLinkSource ──────────────────────────────────────────────────────────

pub struct MavLinkSource {
    port: u16,
    system_id: u8,
}

impl MavLinkSource {
    pub fn from_config(cfg: &Config) -> Self {
        Self {
            port: cfg.mavlink_listen_port,
            system_id: cfg.mavlink_system_id,
        }
    }
}

impl PlatformSource for MavLinkSource {
    async fn run(self, tx: watch::Sender<PlatformState>) {
        let addr = format!("0.0.0.0:{}", self.port);
        let socket = match UdpSocket::bind(&addr).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[mavlink] failed to bind {addr}: {e}");
                return;
            }
        };
        println!("[mavlink] listening on {addr}");

        let mut partial = PartialState::default();
        let mut buf = [0u8; 2048];

        loop {
            let Ok((len, _src)) = socket.recv_from(&mut buf).await else {
                continue;
            };
            for msg in parse_frames(&buf, len, self.system_id) {
                partial.apply(msg);
                if let Some(state) = partial.complete() {
                    let _ = tx.send(state);
                }
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a valid MAVLink v1 frame for the given msg_id, payload, and sysid.
    fn make_v1_frame(msg_id: u8, payload: &[u8], sysid: u8) -> Vec<u8> {
        let mut frame = vec![
            0xFE,              // magic
            payload.len() as u8, // payload length
            0x00,              // seq
            sysid,             // system id
            0x01,              // component id
            msg_id,            // message id
        ];
        frame.extend_from_slice(payload);
        // CRC covers bytes 1..end-of-payload + CRC_EXTRA
        let crc_data = &frame[1..];
        let extra = crc_extra_for(msg_id as u32).unwrap();
        let mut crc = x25_crc(crc_data);
        crc = x25_crc_accumulate(crc, extra);
        frame.push((crc & 0xFF) as u8);
        frame.push((crc >> 8) as u8);
        frame
    }

    /// Build the 28-byte ATTITUDE payload.
    fn attitude_payload(roll: f32, pitch: f32, yaw: f32) -> Vec<u8> {
        let mut p = vec![0u8; 28];
        p[0..4].copy_from_slice(&0u32.to_le_bytes()); // time_boot_ms
        p[4..8].copy_from_slice(&roll.to_le_bytes());
        p[8..12].copy_from_slice(&pitch.to_le_bytes());
        p[12..16].copy_from_slice(&yaw.to_le_bytes());
        // remaining fields zero
        p
    }

    /// Build the 28-byte GLOBAL_POSITION_INT payload.
    fn gpi_payload(lat_dege7: i32, lon_dege7: i32, alt_mm: i32, vx: i16, vy: i16, vz: i16) -> Vec<u8> {
        let mut p = vec![0u8; 28];
        p[0..4].copy_from_slice(&0u32.to_le_bytes());
        p[4..8].copy_from_slice(&lat_dege7.to_le_bytes());
        p[8..12].copy_from_slice(&lon_dege7.to_le_bytes());
        p[12..16].copy_from_slice(&alt_mm.to_le_bytes());
        p[16..20].copy_from_slice(&0i32.to_le_bytes()); // relative_alt
        p[20..22].copy_from_slice(&vx.to_le_bytes());
        p[22..24].copy_from_slice(&vy.to_le_bytes());
        p[24..26].copy_from_slice(&vz.to_le_bytes());
        p[26..28].copy_from_slice(&0u16.to_le_bytes()); // hdg
        p
    }

    #[test]
    fn test_parse_attitude_v1() {
        let payload = attitude_payload(0.1, 0.2, 0.3);
        let frame = make_v1_frame(30, &payload, 1);
        let msgs = parse_frames(&frame, frame.len(), 0);
        assert_eq!(msgs.len(), 1);
        if let MavMsg::Attitude(a) = &msgs[0] {
            assert!((a.roll - 0.1).abs() < 1e-6);
            assert!((a.pitch - 0.2).abs() < 1e-6);
            assert!((a.yaw - 0.3).abs() < 1e-6);
        } else {
            panic!("expected Attitude");
        }
    }

    #[test]
    fn test_parse_global_position_int_v1() {
        // lat = 37.7749° N → 377_749_000 degE7; lon = -122.4194° → -1_224_194_000 degE7
        let lat_dege7 = 377_749_000i32;
        let lon_dege7 = -1_224_194_00i32; // ~-122.4194°
        let alt_mm = 15_000i32; // 15 m
        let payload = gpi_payload(lat_dege7, lon_dege7, alt_mm, 100, -50, 10);
        let frame = make_v1_frame(33, &payload, 1);
        let msgs = parse_frames(&frame, frame.len(), 0);
        assert_eq!(msgs.len(), 1);
        if let MavMsg::GlobalPositionInt(g) = &msgs[0] {
            let expected_lat = (lat_dege7 as f64 * 1e-7).to_radians();
            assert!((g.lat_rad - expected_lat).abs() < 1e-10);
            assert!((g.alt_m - 15.0).abs() < 1e-6);
            assert!((g.vel_north_m_s - 1.0).abs() < 1e-5);
            assert!((g.vel_east_m_s - (-0.5)).abs() < 1e-5);
            assert!((g.vel_down_m_s - 0.1).abs() < 1e-5);
        } else {
            panic!("expected GlobalPositionInt");
        }
    }

    #[test]
    fn test_partial_state_fusion() {
        let mut ps = PartialState::default();
        // Before both messages arrive, complete() returns None
        assert!(ps.complete().is_none());

        let att = AttitudeData { roll: 0.1, pitch: 0.2, yaw: 0.3 };
        ps.apply(MavMsg::Attitude(att));
        assert!(ps.complete().is_none());

        let gpi = GlobalPosData {
            lat_rad: 0.5,
            lon_rad: -1.2,
            alt_m: 100.0,
            vel_north_m_s: 1.0,
            vel_east_m_s: 2.0,
            vel_down_m_s: -0.5,
        };
        ps.apply(MavMsg::GlobalPositionInt(gpi));
        let state = ps.complete().expect("should be complete");
        assert!((state.roll_rad - 0.1).abs() < 1e-6);
        assert!((state.lat_rad - 0.5).abs() < 1e-10);
        assert!((state.vel_east_m_s - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_crc_rejects_corrupt_frame() {
        let payload = attitude_payload(0.1, 0.2, 0.3);
        let mut frame = make_v1_frame(30, &payload, 1);
        // Flip a bit in the payload
        let payload_start = 6;
        frame[payload_start] ^= 0xFF;
        let msgs = parse_frames(&frame, frame.len(), 0);
        assert!(msgs.is_empty(), "corrupt frame should be rejected");
    }

    #[test]
    fn test_system_id_filter() {
        let payload = attitude_payload(0.0, 0.0, 0.0);
        // Frame from sysid=2
        let frame = make_v1_frame(30, &payload, 2);

        // Accept all
        let msgs = parse_frames(&frame, frame.len(), 0);
        assert_eq!(msgs.len(), 1);

        // Filter to sysid=1 — should drop
        let msgs = parse_frames(&frame, frame.len(), 1);
        assert!(msgs.is_empty());

        // Filter to sysid=2 — should accept
        let msgs = parse_frames(&frame, frame.len(), 2);
        assert_eq!(msgs.len(), 1);
    }
}
